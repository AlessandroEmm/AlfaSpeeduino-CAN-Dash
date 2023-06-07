
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//THE SOFTWARE.


#include "STM32_CAN.h" //My own STM32 CAN library
#include "STM32_PWM.h" 
#include <src/switecX12.h>
  //With the AX1201728SG, we get microstepping, so 1/12 degree micro steps. We need this value in few places
#define MICROSTEPS (315*12)

//this defines how much filtering is applied to the values to avoid needle jumping around. 0 = no filtering, 255 = max filtering.
#define filter_amount 5
//low pass filter stolen from speeduino code
#define FILTER(input, alpha, prior) (((long)input * (256 - alpha) + ((long)prior * alpha))) >> 8
// 50 Hz rate to update data from OBD2. This can be adjusted to suit the needs.
#define GaugePWMRate 50
// Custom font for the screens

#define USING_MICROS_RESOLUTION       true 

//  Voltages Levels Min/Max
#define minCltV = 7
#define maxCltV = 34
#define minOilPressureV = 7
#define maxOilPressureV = 85
#define minFuelLevelV = 3
#define maxFuelLevelV = 34


 int oilPressurePin = PB7;
TIM_TypeDef *oilPressureGaugeTimer = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(oilPressurePin), PinMap_PWM); 
 uint32_t oilPressureChannel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(oilPressurePin), PinMap_PWM));
   // Instantiate HardwareTimer object. Thanks to 'new' instantiation, HardwareTimer is not destructed when setup() function is finished.
HardwareTimer *oilPressureGauge = new HardwareTimer(oilPressureGaugeTimer);
// int previousSpeed = 0;

int fuelLevelPressurePin = PB9;
TIM_TypeDef *fuelLevelGaugeTimer = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(fuelLevelPressurePin), PinMap_PWM); 
uint32_t fuelLevelChannel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(fuelLevelPressurePin), PinMap_PWM));
HardwareTimer *fuelLevelGauge = new HardwareTimer(fuelLevelGaugeTimer);


int cltPin = PB8;
TIM_TypeDef *cltGaugeTimer = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(cltPin), PinMap_PWM);
uint32_t cltChannel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(cltPin), PinMap_PWM));
HardwareTimer *cltGauge = new HardwareTimer(cltGaugeTimer);



static CAN_message_t CAN_outMsg;
static CAN_message_t CAN_inMsg;

static uint32_t RPM_timeout=millis();   // for the RPM timeout
//CAN rx buffer size increased, because it might be needed in busy CAN bus.
STM32_CAN Can1( CAN1, DEF, RX_SIZE_64, TX_SIZE_16 );

#define FLASH_BASEADRESS     0x801D400UL

SwitecX12 RPMGauge(MICROSTEPS, PB6, PB7, 1);

// to keep track of if OBD2 requests have been sent.
bool RPM_Request=true;
bool CLT_Request=true;
bool oilPressure_Request=true;
uint16_t RPM,RPMsteps,CLTpos,fuelLevelPos,oilPressurePos;

int8_t oneSec;
// Extra values to keep track of.
uint8_t CLT, IAT;
uint8_t fuelLevel, oilPressure;
uint8_t TPS;
// for now program memory flash is used to store odometer/trip. It has wear limit, so we try to avoid writing to it as much as possible. So this makes the flash to be written only once on every power up.
bool notCommitted;
// push buttons on the instrument cluster
const int leftButton = PC13;
const int rightButton = PB5;
// the instrument cluster can be kept on using this pin as output to drive home the needles.
const int powerPin = PA0;
// instrument cluster lights.
const int oilPressLight = PB1;
const int fuelReserveLight = PB0;
const int brakeLight = PA1;

#if ((STM32_CORE_VERSION_MINOR<=8) & (STM32_CORE_VERSION_MAJOR==1))
void requestData(HardwareTimer*){void requestData();}
void updateGauges(HardwareTimer*){void updateGauges();}
#endif

void updateGauges()
{
  // the gauges only moves when update is called. So we call the update on this fuction for every 100uS.
  RPMGauge.update();
}


void setup(void)
{
  Serial.begin(115200); // for debugging
  pinMode(LED_BUILTIN, OUTPUT);
  // Init CAN
  Can1.begin();
  Can1.setBaudRate(500000);
  // Filter out unwanted CAN messages.
  Can1.setMBFilterProcessing( MB0, 0x316, 0x1FFFFFFF );
  Can1.setMBFilterProcessing( MB1, 0x329, 0x1FFFFFFF );
  Can1.setMBFilterProcessing( MB2, 0x153, 0x1FFFFFFF );
  Can1.setMBFilterProcessing( MB3, 0x7E8, 0x1FFFFFFF );

  CAN_inMsg.len = 8;
  
  RPM = 0;
  RPMsteps = 0;
  oneSec = 0;
  oilPressure = 0;
  CLT = 0;
  TPS = 0;
  notCommitted = true;
  RPMGauge.setPosition(0);

  // setup hardwaretimer to request obd data in 50Hz pace. Otherwise the obd2 requests can be too fast. This timer is also used to calculate odometer and trip in 1sec intervals.
#if defined(TIM1)
  TIM_TypeDef *Instance1 = TIM1;
#else
  TIM_TypeDef *Instance1 = TIM2;
#endif
  TIM_TypeDef *Instance2 = TIM3;
  HardwareTimer *gaugeTimer = new HardwareTimer(Instance2);
  gaugeTimer->setOverflow(10000, HERTZ_FORMAT);
  gaugeTimer->attachInterrupt(updateGauges);
  gaugeTimer->resume();

 
  // oilPressureGauge->setPWM(channel, oilPressurePin, 200, 10);
  // cltGauge->setPWM(channel, oilPressurePin, 200, 10);
  // fuelLevelGauge->setPWM(channel, oilPressurePin, 200, 10);
  // Serial.println("Setup done");
}


void CalcRPMgaugeSteps()
{
  uint16_t tempRPMsteps = 0;
  // RPM gauge face is not linear. Below 1000 RPM has different scale than above it.
  if ( RPM < 6400 ) // < 1000rpm
  {
    tempRPMsteps = map(RPM, 0, 6400, 0, 50);
  }
  else if ( RPM < 63616 ) // >= 1000rpm
  {
    tempRPMsteps = map(RPM, 6400, 63616, 50, MICROSTEPS);
  }
  else if ( RPM >= 63616 )// limit to max steps
  {
    tempRPMsteps = MICROSTEPS;
  }

  // low pass filter the step value to prevent the needle from jumping.
  RPMsteps = FILTER(tempRPMsteps, filter_amount, RPMsteps);
  RPMGauge.setPosition(RPMsteps);
}



void CalcCLTGaugePos()
{
  uint16_t tempCLTPos = 0;
  tempCLTPos = map(CLT, 60, 120, 0, MICROSTEPS);
  // low pass filter the step value to prevent the needle from jumping.
  CLTpos = FILTER(tempCLTPos, filter_amount, CLTpos);
  cltGauge->setCaptureCompare(cltChannel, CLTpos, PERCENT_COMPARE_FORMAT);
}
void CalcFuelLevelGaugePos()
{
  uint16_t tempFuelPos= 0;
  tempFuelPos = map(fuelLevel, 0, 255, 0, 100);
  // low pass filter the step value to prevent the needle from jumping.
  fuelLevelPos = FILTER(tempFuelPos, filter_amount, fuelLevelPos);
  fuelLevelGauge->setCaptureCompare(fuelLevelChannel, oilPressurePos, PERCENT_COMPARE_FORMAT);
}
void CalcOilPressureGaugePos()
{
  uint16_t tempOilPressurePos = 0;
  tempOilPressurePos = map(oilPressure, 0, 255, 0, 100);
  // low pass filter the step value to prevent the needle from jumping.
  oilPressurePos = FILTER(tempOilPressurePos, filter_amount, oilPressurePos);
  oilPressureGauge->setCaptureCompare(oilPressureChannel, oilPressurePos, PERCENT_COMPARE_FORMAT);
}


void readCanMessage()
{
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); 
  switch (CAN_inMsg.id)
  {
    case 0x316: // RPM in e39/e46 etc.
      RPM = ((CAN_inMsg.buf[3] << 8) | (CAN_inMsg.buf[2]));
      CalcRPMgaugeSteps();
      uint32_t tempRPM;
      tempRPM = RPM;
      // convert the e39/e46 RPM data to real RPM reading
      tempRPM = (tempRPM * 10) / 64;
      RPM_timeout = millis();             // zero the timeout
    break;
    case 0x329: // CLT in e39/e46 etc.
      CLT = (CAN_inMsg.buf[1]);
      IAT = (CAN_inMsg.buf[3]);
      Serial.print("IAT ");Serial.println(IAT);
      Serial.print("CLT ");Serial.println(CLT);
      TPS = (CAN_inMsg.buf[5]);
          Serial.print("TPS ");Serial.println(TPS);
      //CalcCLTgaugeSteps();
    break;
    case 0x324: // CLT in e39/e46 etc.
      CLT = (CAN_inMsg.buf[1]);
     
      TPS = (CAN_inMsg.buf[5]);
      //CalcCLTgaugeSteps();
    break;
    case 0x339: // CLT in e39/e46 etc.
      CLT = (CAN_inMsg.buf[1]);
      TPS = (CAN_inMsg.buf[5]);
      //CalcCLTgaugeSteps();
    break;
    default:
      // nothing to do here
    break;
  }
}




void clusterShutdown()
{
  // Set the needles back to zero
  RPMGauge.setPosition(0);
  if (notCommitted) {
	notCommitted = false;
    
  }
  // wait until zero
  while ( (RPMGauge.currentStep != 0) && (RPMGauge.currentStep != 0) ) {
    //Serial.println(RPMGauge.currentStep);
  }
  // turn off the cluster
  digitalWrite(powerPin, LOW);
  Serial.println("Shutdown completed");
}

void writeDataToFlash()
{
  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t pageError = 0;
  
  /* ERASING page */
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.Banks = 1;
  EraseInitStruct.PageAddress = FLASH_BASEADRESS;
  EraseInitStruct.NbPages = 1;

  //Clear any flash errors before try writing to flash to prevent write failures.
  if(__HAL_FLASH_GET_FLAG(FLASH_FLAG_WRPERR) != RESET) __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_WRPERR);
  if(__HAL_FLASH_GET_FLAG(FLASH_FLAG_PGERR) != RESET) __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGERR);

  HAL_FLASH_Unlock();
  if (HAL_FLASHEx_Erase(&EraseInitStruct, &pageError) == HAL_OK){Serial.println("Flash page erased");}
  HAL_FLASH_Lock();
}

void loop(void)
{
  if ( (millis()-RPM_timeout) > 500) { // timeout, because no RPM data from CAN bus
    RPM_timeout = millis();
    RPM = 0;
    RPMGauge.setPosition(0);
    RPM_Request = true;
    Serial.println ("RPM timeout");
	// no RPM data, so we assume that the car has shut down. So proceed to shut down the cluster too.
	Serial.println ("Shutdown started");
	clusterShutdown();
  }
  

  // see if there is messages available on can bus to read.
  while (Can1.read(CAN_inMsg) ) {
    readCanMessage();
  }
}