// This code is meant to read real time data from Speeduino EFI using serial3 connection in speeduino and convert that to CAN messages for BMW e39/e46 instrument clusters
// The hardware that the code is meant to be used is STM32F103C8T6 STM32 Development Board (BluePill) with MCP2551 transceiver.
// Created by pazi88 and there is no guarantee at all that any of this will work.
// Use Platform IO in VScode to compile this. The platformio.ini is at the folder above this. Open that folder to your workspace in PIO.

//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//THE SOFTWARE.


#include "STM32_CAN.h"

#define NOTHING_RECEIVED        0
#define N_MESSAGE               1
#define LED_BUILTIN             PB2


#define ClusterUpdateRate 50  // 50 Hz Frequency for the cars instrument cluster




static CAN_message_t CAN_msg_RPM;
static CAN_message_t CAN_msg_CLT_TPS;
static CAN_message_t CAN_msg_O2;
static CAN_message_t CAN_msg_IAT;
static CAN_message_t CAN_msg_Pressures;
static CAN_message_t CAN_msg_Statuses;

static CAN_message_t CAN_msg_BatteryV;

static CAN_message_t CAN_inMsg;

STM32_CAN Can1( CAN1, DEF, RX_SIZE_64, TX_SIZE_16 );

// This struct gathers data read from speeduino. This is really just direct copy of what speeduino has internally
struct statuses {
  uint8_t secl; // secl is simply a counter that increments each second.
  uint8_t status1; // status1 Bitfield, inj1Status(0), inj2Status(1), inj3Status(2), inj4Status(3), DFCOOn(4), boostCutFuel(5), toothLog1Ready(6), toothLog2Ready(7)
  uint8_t engine; // Engine Status Bitfield, running(0), crank(1), ase(2), warmup(3), tpsaccaen(4), tpsacden(5), mapaccaen(6), mapaccden(7)
  uint8_t dwell; // Dwell in ms * 10
  uint16_t MAP; // 2 bytes for MAP
  uint8_t IAT;
  uint8_t CLT;
  uint8_t batCorrection; // Battery voltage correction (%)
  uint8_t battery10; // battery voltage
  uint8_t O2; // O2
  uint8_t egoCorrection; // Exhaust gas correction (%)
  uint8_t iatCorrection; // Air temperature Correction (%)
  uint8_t wueCorrection; // Warmup enrichment (%)
  uint16_t RPM; // rpm
  uint8_t AEamount; // acceleration enrichment (%)
  uint8_t corrections; // Total GammaE (%)
  uint8_t VE; // Current VE 1 (%)
  uint8_t afrTarget;
  uint16_t PW1; // Pulsewidth 1 multiplied by 10 in ms. Have to convert from uS to mS.
  uint8_t tpsDOT; // TPS DOT
  int8_t advance;
  uint8_t TPS; // TPS (0% to 100%)
  uint16_t loopsPerSecond;
  uint16_t freeRAM;
  uint8_t boostTarget; // boost target divided by 2 to fit in a byte
  uint8_t boostDuty;
  uint8_t spark; // Spark related bitfield, launchHard(0), launchSoft(1), hardLimitOn(2), softLimitOn(3), boostCutSpark(4), error(5), idleControlOn(6), sync(7)
  uint16_t rpmDOT;
  uint8_t ethanolPct; // Flex sensor value (or 0 if not used)
  uint8_t flexCorrection; // Flex fuel correction (% above or below 100)
  uint8_t flexIgnCorrection; // Ignition correction (Increased degrees of advance) for flex fuel
  uint8_t idleLoad;
  uint8_t testOutputs; // testEnabled(0), testActive(1)
  uint8_t O2_2; // O2
  uint8_t baro; // Barometer value
  uint16_t CANin_1;
  uint16_t CANin_2;
  uint16_t CANin_3;
  uint16_t CANin_4;
  uint16_t CANin_5;
  uint16_t CANin_6;
  uint16_t CANin_7;
  uint16_t CANin_8;
  uint16_t CANin_9;
  uint16_t CANin_10;
  uint16_t CANin_11;
  uint16_t CANin_12;
  uint16_t CANin_13;
  uint16_t CANin_14;
  uint16_t CANin_15;
  uint16_t CANin_16;
  uint8_t tpsADC;
  uint8_t getNextError;
  uint8_t launchCorrection;
  uint8_t fuelPressure;
  uint8_t oilPressure;
  uint8_t status3;
};

statuses currentStatus;

static uint32_t oldtime=millis();   // for the timeout
uint32_t channel; // timer channel for PWM fan
uint8_t SpeedyResponse[150]; //The data buffer for the serial3 data. This is longer than needed, just in case
uint8_t rpmLSB;   // Least significant byte for RPM message
uint8_t rpmMSB;  // Most significant byte for RPM message
uint8_t pwLSB;   // Least significant byte for PW message
uint8_t pwMSB;  // Most significant byte for PW message
uint8_t CEL;   //timer for how long CEL light be kept on
uint32_t updatePW;
uint8_t odometerLSB;
uint8_t odometerMSB;
uint8_t FuelLevel;
uint8_t ambientTemp;
int CLT, IAT; // to store coolant temp
uint8_t oilPressure;
uint8_t fuelPressure;
uint8_t engineStatus[8] = {0}; 
uint8_t status ;
uint32_t PWcount;
uint8_t O2;
uint8_t egoCorrection, gammaEnrichment;
uint8_t TPS,tempLight; // TPS value and overheat light on/off
bool data_error; //indicator for the data from speeduino being ok.
bool responseSent; // to keep track if we have responded to data request or not.
bool newData; // This tells if we have new data available from speeduino or not.
bool ascMSG; // ASC message received.
bool doRequest; // when true, it's ok to reques more data from speeduino serial
uint8_t SerialState,canin_channel,currentCommand;
uint16_t CanAddress,runningClock;
uint16_t VSS,VSS1,VSS2,VSS3,VSS4;
uint8_t MSGcounter; //this keeps track of which multiplexed info is sent in 0x329 byte 0
uint8_t radOutletTemp;
uint8_t oilTemp;


// define hardwaretimers
TIM_TypeDef *Instance1 = TIM1;
TIM_TypeDef *Instance2 = TIM3;
HardwareTimer *SendTimer = new HardwareTimer(Instance1);
HardwareTimer *requestTimer = new HardwareTimer(Instance2);
 
void requestData() {
  if (doRequest){
    Serial3.write("n"); // Send A to request real time data
    doRequest = false;
}}

void SendData()   // Send can messages in 50Hz phase from timer interrupt. This is important to be high enough Hz rate to make cluster work smoothly.
{
  CAN_msg_RPM.buf[2]= rpmLSB; // RPM LSB
  CAN_msg_RPM.buf[3]= rpmMSB; // RPM MSB
  
  if ( Can1.write(CAN_msg_RPM) ){
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // Just to see with internal led that CAN messages are being sent
  }

    //Send CLT and TPS
  CAN_msg_CLT_TPS.buf[1]= CLT; // Coolant temp
  CAN_msg_CLT_TPS.buf[3]= IAT; // Coolant temp
  CAN_msg_CLT_TPS.buf[5]= TPS; // TPS value.
  Can1.write(CAN_msg_CLT_TPS);

    //Send Fuel and Oil Pressure
  CAN_msg_Pressures.buf[1]= oilPressure; // Coolant temp
  CAN_msg_Pressures.buf[4]= fuelPressure; // TPS value.
  Can1.write(CAN_msg_Pressures);

      //Send Fuel and Oil Pressure
  CAN_msg_O2.buf[1]=  O2; // O2
  CAN_msg_O2.buf[2]= egoCorrection; // TPS value.
  CAN_msg_O2.buf[3]= gammaEnrichment; // TPS value.
  Can1.write(CAN_msg_O2);

 
  // Fuel consumption counter is 2-bytes so if the current value is higher than that, we roll over the counter.

  pwMSB = highByte(uint16_t(PWcount));  // split to high and low byte
  pwLSB = lowByte(uint16_t(PWcount));

  
  MSGcounter++;
  if (MSGcounter >= 3)
  {
    MSGcounter = 0;
  }
}

void setup(){
  Serial3.begin(115200);  // baudrate for Speeduino is 115200
  Serial.begin(115200); // for debugging
  
  pinMode(LED_BUILTIN, OUTPUT);

  doRequest = false;
  Can1.begin();
  Can1.setBaudRate(500000);
  Can1.setMBFilterProcessing( MB0, 0x153, 0x1FFFFFFF );
  Can1.setMBFilterProcessing( MB1, 0x613, 0x1FFFFFFF );
  Can1.setMBFilterProcessing( MB2, 0x615, 0x1FFFFFFF );
  Can1.setMBFilterProcessing( MB3, 0x1F0, 0x1FFFFFFF );

  CAN_msg_RPM.len = 8; // 8 bytes in can message
  CAN_msg_CLT_TPS.len = 7;
  CAN_msg_Pressures.len = 4;
  CAN_msg_O2.len = 3;

  CAN_msg_RPM.id = 0x316; // CAN ID for RPM message is 0x316
  CAN_msg_CLT_TPS.id = 0x329; // CAN ID for CLT and TSP message is 0x329
  CAN_msg_Pressures.id = 0x324; // CAN ID for CLT and TSP message is 0x329
  CAN_msg_O2.id = 0x339; // CAN ID for CLT and TSP message is 0x329

  // set the static values for the other two messages
  CAN_msg_RPM.buf[0]= 0x01;  //bitfield, Bit0 = 1 = terminal 15 on detected, Bit2 = 1 = 1 = the ASC message ASC1 was received within the last 500 ms and contains no plausibility errors
  CAN_msg_RPM.buf[1]= 0x0C;  //Indexed Engine Torque in % of C_TQ_STND TBD do torque calculation!!
  CAN_msg_RPM.buf[4]= 0x0C;  //Indicated Engine Torque in % of C_TQ_STND TBD do torque calculation!! Use same as for byte 1
  CAN_msg_RPM.buf[5]= 0x15;  //Engine Torque Loss (due to engine friction, AC compressor and electrical power consumption)
  CAN_msg_RPM.buf[6]= 0x00;  //not used
  CAN_msg_RPM.buf[7]= 0x35;  //Theorethical Engine Torque in % of C_TQ_STND after charge intervention

  CAN_msg_CLT_TPS.buf[0]= 0x11;  //Multiplexed Information
  CAN_msg_CLT_TPS.buf[2]= 0xB2;  //CLT temp
  CAN_msg_CLT_TPS.buf[3]= 0x00;  //IAT
  CAN_msg_CLT_TPS.buf[4]= 0x08;  //bitfield, Bit0 = 0 = Clutch released, Bit 3 = 1 = engine running
  CAN_msg_CLT_TPS.buf[6]= 0x00;  //TPS_VIRT_CRU_CAN (Not used)
  CAN_msg_CLT_TPS.buf[7]= 0x00;  //not used, but set to zero just in case.

  CAN_msg_BatteryV.buf[0]= 0x11;  //Multiplexed Information
  CAN_msg_BatteryV.buf[2]= 0xB2;  //CLT temp
  CAN_msg_BatteryV.buf[3]= 0x00;  //Baro
  CAN_msg_BatteryV.buf[4]= 0x08;  //bitfield, Bit0 = 0 = Clutch released, Bit 3 = 1 = engine running
  CAN_msg_BatteryV.buf[6]= 0x00;  //TPS_VIRT_CRU_CAN (Not used)
  CAN_msg_BatteryV.buf[7]= 0x00;  //not used, but set to zero just in case.

  CAN_msg_Pressures.buf[0]= 0x11;  //Multiplexed Information
  CAN_msg_Pressures.buf[2]= 0xB2;  //CLT temp
  CAN_msg_Pressures.buf[3]= 0x00;  //Baro
  CAN_msg_Pressures.buf[4]= 0x08;  //bitfield, Bit0 = 0 = Clutch released, Bit 3 = 1 = engine running
  CAN_msg_Pressures.buf[6]= 0x00;  //TPS_VIRT_CRU_CAN (Not used)
  CAN_msg_Pressures.buf[7]= 0x00;  //not used, but set to zero just in case.

  CAN_msg_Statuses.buf[0]= 0x11;  //Multiplexed Information
  CAN_msg_Statuses.buf[2]= 0xB2;  //CLT temp
  CAN_msg_Statuses.buf[3]= 0x00;  //Baro
  CAN_msg_Statuses.buf[4]= 0x08;  //bitfield, Bit0 = 0 = Clutch released, Bit 3 = 1 = engine running
  CAN_msg_Statuses.buf[6]= 0x00;  //TPS_VIRT_CRU_CAN (Not used)
  CAN_msg_Statuses.buf[7]= 0x00;  //not used, but set to zero just in case.

  // Start with sensible values for some of these variables.
  CLT = 60;
  IAT = 20;
  oilPressure = 0;
  fuelPressure = 0;
  currentStatus.PW1 = 0;
  updatePW = 0;
  rpmLSB = 0;
  rpmMSB = 0;
  pwLSB = 0;
  pwMSB = 0;
  SerialState = NOTHING_RECEIVED;
  data_error = false;
  responseSent = false;
  newData = false;
  MSGcounter = 0;
  ascMSG = false;
  radOutletTemp = 0;
  oilTemp = 0;
  O2=147;
  egoCorrection=0;
  gammaEnrichment=100;


  SendTimer->setOverflow(ClusterUpdateRate, HERTZ_FORMAT);
  requestTimer->attachInterrupt(requestData);
  SendTimer->attachInterrupt(SendData); 
  requestTimer->resume();
  SendTimer->resume();

  Serial.println ("Version date: 3.4.2023"); // To see from debug serial when is used code created.
  doRequest = true; // all set. Start requesting data from speeduino
  //rRequestCounter = SerialUpdateRate;
}


// display the needed values in serial monitor for debugging
void displayData(){
  Serial.print ("RPM-"); Serial.print (currentStatus.RPM); Serial.print("\t");
  Serial.print ("CLT-"); Serial.print (CLT); Serial.print("\t");
  Serial.print ("TPS-"); Serial.print (TPS); Serial.println("\t");
  Serial.print ("OilPressure-"); Serial.print (oilPressure); Serial.println("\t");
  Serial.print ("FuelPressure-"); Serial.print (fuelPressure); Serial.println("\t");
    Serial.print ("O2-"); Serial.print (O2); Serial.println("\t");
  Serial.print ("EcoCrrection-"); Serial.print (egoCorrection); Serial.println("\t");

}

void processData(){   // necessary conversion for the data before sending to CAN BUS
  unsigned int tempRPM;
  data_error = false; // set the received data as ok

  currentStatus.secl = SpeedyResponse[0];
  currentStatus.status1 = SpeedyResponse[1];
  currentStatus.engine = SpeedyResponse[2];
  currentStatus.dwell = SpeedyResponse[3];
  currentStatus.MAP = ((SpeedyResponse [5] << 8) | (SpeedyResponse [4]));
  currentStatus.IAT = SpeedyResponse[6];
  currentStatus.CLT = SpeedyResponse[7];
  currentStatus.batCorrection = SpeedyResponse[8];
  currentStatus.battery10 = SpeedyResponse[9];
  currentStatus.O2 = SpeedyResponse[10];
  currentStatus.egoCorrection = SpeedyResponse[11];
  currentStatus.iatCorrection = SpeedyResponse[12];
  currentStatus.wueCorrection = SpeedyResponse[13];
  currentStatus.RPM = ((SpeedyResponse [15] << 8) | (SpeedyResponse [14])); // RPM low & high (Int) TBD: probaply no need to split high and low bytes etc. this could be all simpler
  currentStatus.AEamount = SpeedyResponse[16];
  currentStatus.corrections = SpeedyResponse[17];
  currentStatus.VE = SpeedyResponse[18];
  currentStatus.afrTarget = SpeedyResponse[19];
  currentStatus.PW1 = ((SpeedyResponse [21] << 8) | (SpeedyResponse [20])); // PW low & high (Int) TBD: probaply no need to split high and low bytes etc. this could be all simpler
  currentStatus.tpsDOT = SpeedyResponse[22];
  currentStatus.advance = SpeedyResponse[23];
  currentStatus.TPS = SpeedyResponse[24] /2;
  currentStatus.loopsPerSecond = ((SpeedyResponse [26] << 8) | (SpeedyResponse [25]));
  currentStatus.freeRAM = ((SpeedyResponse [28] << 8) | (SpeedyResponse [27]));
  currentStatus.boostTarget = SpeedyResponse[29]; // boost target divided by 2 to fit in a byte
  currentStatus.boostDuty = SpeedyResponse[30];
  currentStatus.spark = SpeedyResponse[31]; // Spark related bitfield, launchHard(0), launchSoft(1), hardLimitOn(2), softLimitOn(3), boostCutSpark(4), error(5), idleControlOn(6), sync(7)
  currentStatus.rpmDOT = ((SpeedyResponse [33] << 8) | (SpeedyResponse [32]));
  currentStatus.ethanolPct = SpeedyResponse[34]; // Flex sensor value (or 0 if not used)
  currentStatus.flexCorrection = SpeedyResponse[35]; // Flex fuel correction (% above or below 100)
  currentStatus.flexIgnCorrection = SpeedyResponse[36]; // Ignition correction (Increased degrees of advance) for flex fuel
  currentStatus.idleLoad = SpeedyResponse[37];
  currentStatus.testOutputs = SpeedyResponse[38]; // testEnabled(0), testActive(1)
  currentStatus.O2_2 = SpeedyResponse[39]; // O2
  currentStatus.baro = SpeedyResponse[40]; // Barometer value
  currentStatus.CANin_1 = ((SpeedyResponse [42] << 8) | (SpeedyResponse [41]));
  currentStatus.CANin_2 = ((SpeedyResponse [44] << 8) | (SpeedyResponse [43]));
  currentStatus.CANin_3 = ((SpeedyResponse [46] << 8) | (SpeedyResponse [45]));
  currentStatus.CANin_4 = ((SpeedyResponse [48] << 8) | (SpeedyResponse [47]));
  currentStatus.CANin_5 = ((SpeedyResponse [50] << 8) | (SpeedyResponse [49]));
  currentStatus.CANin_6 = ((SpeedyResponse [52] << 8) | (SpeedyResponse [51]));
  currentStatus.CANin_7 = ((SpeedyResponse [54] << 8) | (SpeedyResponse [53]));
  currentStatus.CANin_8 = ((SpeedyResponse [56] << 8) | (SpeedyResponse [55]));
  currentStatus.CANin_9 = ((SpeedyResponse [58] << 8) | (SpeedyResponse [57]));
  currentStatus.CANin_10 = ((SpeedyResponse [60] << 8) | (SpeedyResponse [59]));
  currentStatus.CANin_11 = ((SpeedyResponse [62] << 8) | (SpeedyResponse [61]));
  currentStatus.CANin_12 = ((SpeedyResponse [64] << 8) | (SpeedyResponse [63]));
  currentStatus.CANin_13 = ((SpeedyResponse [66] << 8) | (SpeedyResponse [65]));
  currentStatus.CANin_14 = ((SpeedyResponse [68] << 8) | (SpeedyResponse [67]));
  currentStatus.CANin_15 = ((SpeedyResponse [70] << 8) | (SpeedyResponse [69]));
  currentStatus.CANin_16 = ((SpeedyResponse [71] << 8) | (SpeedyResponse [71]));
  currentStatus.tpsADC = SpeedyResponse[73];
  currentStatus.status3 = SpeedyResponse[82];
  currentStatus.fuelPressure = SpeedyResponse[103];
  currentStatus.oilPressure = SpeedyResponse[104];

  // check if received values makes sense and convert those if all is ok.
  if (currentStatus.RPM < 8000 && data_error == false)  // the engine will not probaply rev over 8000 RPM
  {
    tempRPM = currentStatus.RPM; // RPM conversion factor for e46/e39 cluster
    rpmMSB = tempRPM >> 8;  // split to high and low byte
    rpmLSB = tempRPM;
  }
  else
  {
    data_error = true; // data received is probaply corrupted, don't use it.
    Serial.print ("Error. RPM Received:"); Serial.print (currentStatus.RPM); Serial.print("\t");
  }

  if (currentStatus.CLT < 182 && data_error == false)  // 142 degrees Celcius is the hottest temp that fits to the conversion. 
  {
    CLT = (currentStatus.CLT -40);  // CLT conversion factor for e46/e39 cluster
  }
  else
  {
    data_error = true;  // data received is probaply corrupted, don't use it.
    Serial.print ("Error. CLT received:"); Serial.print (currentStatus.CLT); Serial.print("\t");
  }

  if (currentStatus.TPS < 201 && data_error == false)  // TPS values can only be from 0-200 (previously this was from 0-100 on speeduino)
  {
    TPS = map(currentStatus.TPS, 0, 200, 0, 254); // 0-100 TPS value mapped to 0x00 to 0xFE range.
    newData = true; // we have now new data and it passes the checks.
  }
  else
  {
    data_error = true; // data received is probaply corrupted, don't use it.
    Serial.print ("Error. TPS received:"); Serial.print (currentStatus.TPS); Serial.print("\t");
  }
  oilPressure = currentStatus.oilPressure;
  fuelPressure = currentStatus.fuelPressure;
  IAT = currentStatus.IAT - 40;
//   engineStatus[4] = currentStatus.status3[7];
//   engineStatus[2] = currentStatus.spark[7];
  O2 = currentStatus.O2;
  egoCorrection = currentStatus.egoCorrection;
  gammaEnrichment = currentStatus.corrections;
 }

void HandleN()
{
  Serial.print ("n ");
  data_error = false;

  Serial3.read(); // 0x32
  uint8_t nLength = Serial3.read();
  uint8_t bytesRead = Serial3.readBytes(SpeedyResponse, nLength);
   
  processData();                  // do the necessary processing for received data
  displayData();                  // only required for debugging
  doRequest = true;               // restart data reading
  oldtime = millis();             // zero the timeout
  SerialState = NOTHING_RECEIVED; // all done. We set state for reading what's next message.
}


void ReadSerial()
{
  currentCommand = Serial3.read();
  switch (currentCommand)
  {
    case 'n':  // Speeduino sends data in A-message
      SerialState = N_MESSAGE;
    break;
    default:
     Serial.print ("Not an N message ");
     Serial.println (currentCommand);
    break;
  }
}

// main loop
void loop() {
  switch(SerialState) {
    case NOTHING_RECEIVED:
      if (Serial3.available() > 0) { ReadSerial(); }  // read bytes from serial3 to define what message speeduino is sending.
      break;
    case N_MESSAGE:
      if (Serial3.available() >= 118) { HandleN(); }  // read and process the A-message from serial3, when it's fully received.
      break;
    default:
      break;
  }

  if ( (millis()-oldtime) > 500) { // timeout if for some reason reading serial3 fails
    oldtime = millis();
    Serial.println ("Timeout from speeduino!");
    doRequest = true;                // restart data reading
  }
}