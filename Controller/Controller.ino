
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

static bool forward = true;

#define LED_BUILTIN             PB2

  //With the AX1201728SG, we get microstepping, so 1/12 degree micro steps. We need this value in few places
#define MICROSTEPS (315*12)

//this defines how much filtering is applied to the values to avoid needle jumping around. 0 = no filtering, 255 = max filtering.
#define filter_amount 5
//low pass filter stolen from speeduino code
#define FILTER(input, alpha, prior) (((long)input * (256 - alpha) + ((long)prior * alpha))) >> 8
// 50 Hz rate to update data from OBD2. This can be adjusted to suit the needs.
#define GaugePWMRate 50

#define MAXRPM 7000.0
#define MAXPOSITION 2923.0
#define MINRPM 500.0
#define MAXPOSITIONOFFSET MINRPM / 7000.0
#define MINRPMPOSITION 0.0

//  Voltages Levels Min/Max
#define minCltV 8
#define maxCltV 29
#define minOilPressureV 7
#define maxOilPressureV 85
#define minFuelLevelV 3
#define maxFuelLevelV 34


 int oilPressurePin = PB8;
TIM_TypeDef *oilPressureGaugeTimer = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(oilPressurePin), PinMap_PWM); 
 uint32_t oilPressureChannel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(oilPressurePin), PinMap_PWM));
   // Instantiate HardwareTimer object. Thanks to 'new' instantiation, HardwareTimer is not destructed when setup() function is finished.
HardwareTimer *oilPressureGauge = new HardwareTimer(oilPressureGaugeTimer);

int fuelLevelPin = PB9;
TIM_TypeDef *fuelLevelGaugeTimer = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(fuelLevelPin), PinMap_PWM); 
uint32_t fuelLevelChannel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(fuelLevelPin), PinMap_PWM));
HardwareTimer *fuelLevelGauge = new HardwareTimer(fuelLevelGaugeTimer);


int cltPin = PB7;
TIM_TypeDef *cltGaugeTimer = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(cltPin), PinMap_PWM);
uint32_t cltChannel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(cltPin), PinMap_PWM));
HardwareTimer *cltGauge = new HardwareTimer(cltGaugeTimer);



static CAN_message_t CAN_outMsg;
static CAN_message_t CAN_inMsg;

static uint32_t RPM_timeout=millis();   // for the RPM timeout
//CAN rx buffer size increased, because it might be needed in busy CAN bus.
STM32_CAN Can1( CAN1, DEF, RX_SIZE_64, TX_SIZE_16 );

#define FLASH_BASEADRESS     0x801D400UL

SwitecX12 RPMGauge(MICROSTEPS, PB10, PB11, 0);

// to keep track of if OBD2 requests have been sent.
bool RPM_Request=true;
bool CLT_Request=true;
bool oilPressure_Request=true;
uint16_t RPM,RPMsteps,fuelLevelPos,oilPressurePos;
float CLTpos;
int8_t oneSec;
// Extra values to keep track of.
uint8_t CLT, IAT;
uint8_t fuelLevel, oilPressure;
uint8_t TPS;
// for now program memory flash is used to store odometer/trip. It has wear limit, so we try to avoid writing to it as much as possible. So this makes the flash to be written only once on every power up.
bool notCommitted;
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

void zero(void)
{
 //  RPMGauge.zero();
   Serial.println("Reset Gauge");
   RPMGauge.setPosition(MICROSTEPS);
   delay(2000);
   RPMGauge.setPosition(0);
delay(2000);
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
  Can1.setMBFilterProcessing( MB2, 0x324, 0x1FFFFFFF );
  Can1.setMBFilterProcessing( MB3, 0x339, 0x1FFFFFFF );

  CAN_inMsg.len = 8;
  pinMode(PA0, INPUT);
  pinMode(PB12, OUTPUT);
  digitalWrite(PB12, HIGH);
  attachInterrupt(digitalPinToInterrupt(PA0), zero, RISING);
  RPM = 0;
  RPMsteps = 0;
  oneSec = 0;
  oilPressure = 0;
  CLT = 0;
  TPS = 0;
  notCommitted = true;

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

  oilPressureGauge->setPWM(oilPressureChannel, oilPressurePin, 200, minOilPressureV);
  cltGauge->setPWM(cltChannel, cltPin, 200, 20);
  delay(5000);
  fuelLevelGauge->setPWM(fuelLevelChannel, fuelLevelPin, 200, minFuelLevelV);
}

int determineTachPos(int currentRPM) {
  if (currentRPM < MINRPM) return determineTachPos(MINRPM);
  else if (currentRPM > MAXRPM) return determineTachPos(MAXRPM);
  else  {
    int rpmPosition =  ((MAXPOSITION / MAXRPM) * currentRPM) * (1.0 + (MAXPOSITIONOFFSET - (MINRPM / float(currentRPM))))  ;
    return rpmPosition;
  }
}

void CalcRPMgaugeSteps()
{
  RPMGauge.setPosition(determineTachPos(RPM));
}


void CalcCLTGaugePos()
{
  CLT = 52;
int tempCLTPos = 0;
if ( CLT >= 10 && CLT <= 44 ) // < 1000rpm
  {
    tempCLTPos = 3;
  }
 
 else if ( CLT >= 45 && CLT <= 51 ) // < 1000rpm
  {
    tempCLTPos = 9;
  }
  else if ( CLT >= 52 && CLT <= 55 ) // < 10
  {
    tempCLTPos = 11;
  }
  else if ( CLT > 55 && CLT <= 60 ) // >= 1000rpm
  {
    tempCLTPos = 15;
  }
  else if ( CLT > 60 && CLT <= 65  )// limit to max steps
  {
   tempCLTPos = 19;
  }
  else if ( CLT > 66 && CLT <= 70  )// limit to max steps
  {
   tempCLTPos = 23;
  }
    else if ( CLT > 71 && CLT <= 75  )// limit to max steps
  {
   tempCLTPos = 27;
  }
     else if ( CLT > 76 && CLT <= 78  )// limit to max steps
  {
   tempCLTPos = 30;
  }
      else if ( CLT > 79 && CLT <= 81  )// limit to max steps
  {
   tempCLTPos = 32;
  }
      else if ( CLT > 82 && CLT <= 84  )// limit to max steps
  {
   tempCLTPos = 35;
  }
   else if ( CLT > 85 && CLT <= 89  )// limit to max steps
  {
   tempCLTPos = 39;
  }
          else if ( CLT > 90 && CLT <= 94  )// limit to max steps
  {
   tempCLTPos = 43;
  }
           else if ( CLT > 95 && CLT <= 99  )// limit to max steps
  {
   tempCLTPos = 48;
  }
          else if ( CLT > 100 && CLT <= 105  )// limit to max steps
  {
   tempCLTPos = 55;
  }
            else if ( CLT > 106 && CLT <= 110  )// limit to max steps
  {
   tempCLTPos = 64;
  }
              else if ( CLT > 110 && CLT <= 120  )// limit to max steps
  {
   tempCLTPos = 100;
  }
  else 
  { tempCLTPos = maxCltV; }

   //  Serial.print (tempCLTPos); Serial.print (" "); Serial.println (CLT );
  cltGauge->setCaptureCompare(cltChannel, tempCLTPos, RESOLUTION_8B_COMPARE_FORMAT);
}
void CalcFuelLevelGaugePos()
{
  uint16_t tempFuelPos= 0;
  tempFuelPos = map(fuelLevel, 0, 255, 0, 100);
  // low pass filter the step value to prevent the needle from jumping.
  fuelLevelPos = FILTER(tempFuelPos, filter_amount, fuelLevelPos);
  fuelLevelGauge->setCaptureCompare(fuelLevelChannel, oilPressurePos, RESOLUTION_8B_COMPARE_FORMAT);
}
void CalcOilPressureGaugePos()
{
  uint16_t tempOilPressurePos = 0;
  tempOilPressurePos = map(oilPressure, 0, 255, 0, 100);
  // low pass filter the step value to prevent the needle from jumping.
  oilPressurePos = FILTER(tempOilPressurePos, filter_amount, oilPressurePos);
  oilPressureGauge->setCaptureCompare(oilPressureChannel, oilPressurePos, RESOLUTION_8B_COMPARE_FORMAT);
}


void readCanMessage()
{
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); 
  switch (CAN_inMsg.id)
  {
    case 0x316: // RPM in e39/e46 etc.
      RPM = ((CAN_inMsg.buf[3] << 8) | (CAN_inMsg.buf[2]));
      CalcRPMgaugeSteps();
      RPM_timeout = millis();             // zero the timeout
    break;
    case 0x329: 
      CLT = (CAN_inMsg.buf[1]);
      IAT = (CAN_inMsg.buf[3]);
      TPS = (CAN_inMsg.buf[5]);
      CalcCLTGaugePos();
    break;
    case 0x324: 
      //CalcCLTgaugeSteps();
    break;
    case 0x339: 
        // Serial.print("o2 ");Serial.println(CAN_inMsg.buf[1]);
        // Serial.print("egocorrection ");Serial.println(CAN_inMsg.buf[2]);
        // Serial.print("gammacorrection ");Serial.println(CAN_inMsg.buf[3]);
    break;
    default:
      // nothing to do here
    break;
  }
}




void clusterShutdown()
{
  // Set the needles back to zero
  RPMGauge.setPosition(determineTachPos(0));
  if (notCommitted) {
	notCommitted = false;
    
  }
  // wait until zero
  while ( (RPMGauge.currentStep != 0) && (RPMGauge.currentStep != 0) ) {
    Serial.println(RPMGauge.currentStep);
  } 
  Serial.println("Shutdown completed");
}



void loop(void)
{

  if ( (millis()-RPM_timeout) > 500) { // timeout, because no RPM data from CAN bus
    RPM_timeout = millis();
    RPM = 0;
    RPM_Request = true;
    Serial.print ("RPM timeout");
    // no RPM data, so we assume that the car has shut down. So proceed to shut down the cluster too.
    Serial.println ("Shutdown started");
    clusterShutdown();
  }


  // see if there is messages available on can bus to read.
  while (Can1.read(CAN_inMsg) ) {
    readCanMessage();
  }
}


