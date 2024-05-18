
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//THE SOFTWARE.


#include "STM32_CAN.h" //My own STM32 CAN library
#include "STM32_PWM.h" 
#include <src/SwitecX12.h>


  //With the AX1201728SG, we get microstepping, so 1/12 degree micro steps. We need this value in few places
#define STEPS (315*12)
#define LED_BUILTIN PB2

#define MAXRPM 7000.0
#define MAXPOSITION 2923.0
#define MINRPM 500.0
#define MAXPOSITIONOFFSET MINRPM / 7000.0
#define MINRPMPOSITION 0.0


int RPM;

static CAN_message_t CAN_outMsg;
static CAN_message_t CAN_inMsg;

static uint32_t RPM_timeout=millis();   // for the RPM timeout
//CAN rx buffer size increased, because it might be needed in busy CAN bus.
STM32_CAN Can1( CAN1, DEF, RX_SIZE_64, TX_SIZE_16 );


SwitecX12 RPMGauge(STEPS, PB12, PB13); 

// to keep track of if OBD2 requests have been sent.


 #if ((STM32_CORE_VERSION_MINOR<=8) & (STM32_CORE_VERSION_MAJOR==1))
 void updateGauges(HardwareTimer*){void updateGauges();}
#endif

void updateGauges()
{
  //the gauges only moves when update is called. So we call the update on this fuction for every 100uS.
  RPMGauge.update();
  RPMGauge.setPosition(determineTachPos(RPM));
}

void zero(void)
{
  Serial.println("Zeroing");
   RPMGauge.zero();
   NVIC_SystemReset();
}

void setup(void)
{
  //RPMGauge.zero();
  Serial.begin(115200); // for debugging
  pinMode(LED_BUILTIN, OUTPUT);
  //Init CAN
  Can1.begin();
  Can1.setBaudRate(500000);
  // Filter out unwanted CAN messages.
  Can1.setMBFilterProcessing( MB0, 0x520, 0x1FFFFFFF );

  CAN_inMsg.len = 8;
  pinMode(PA0, INPUT_PULLDOWN);
  pinMode(PB14, OUTPUT);
  digitalWrite(PB14, HIGH);
  attachInterrupt(digitalPinToInterrupt(PA0), zero, RISING);
  RPMGauge.zero();

  TIM_TypeDef *Instance2 = TIM3;
  HardwareTimer *stepperTimer = new HardwareTimer(Instance2);
  stepperTimer->setOverflow(10000, HERTZ_FORMAT);
  stepperTimer->attachInterrupt(updateGauges);
  stepperTimer->resume();

}

int determineTachPos(int currentRPM) {
  if (currentRPM < MINRPM) return determineTachPos(MINRPM);
  else if (currentRPM > MAXRPM) return determineTachPos(MAXRPM);
  else  {
    int rpmPosition =  ((MAXPOSITION / MAXRPM) * currentRPM) * (1.0 + (MAXPOSITIONOFFSET - (MINRPM / float(currentRPM))))  ;
    return rpmPosition;
  }
}


void readCanMessage() 
{
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); 
  switch (CAN_inMsg.id)
  {
    case 0x520: 
      RPM = ((CAN_inMsg.buf[1] << 8) | (CAN_inMsg.buf[0]));
      RPM_timeout = millis();             // zero the timeout
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
    
  // wait until zero
  while ( (RPMGauge.currentStep != 0) && (RPMGauge.currentStep != 0) ) {
  } 
  Serial.println("Shutdown completed");
}



void loop(void)
{

  if ( (millis()-RPM_timeout) > 1000) { // timeout, because no RPM data from CAN bus
    RPM_timeout = millis();
    RPM = 0;
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


