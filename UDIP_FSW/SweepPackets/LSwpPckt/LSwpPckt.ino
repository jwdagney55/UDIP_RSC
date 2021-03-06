#define LSWP_LEN 3215

#define LSWP_POS_SYNC 0
#define LSWP_POS_COUNT 2
#define LSWP_POS_TIME 4
#define LSWP_POS_CHECKSUM 12
#define LSWP_POS_STAT 14
#define LSWP_POS_STEP 15

#define vSweep DAC0

#define noGain A0
#define oneGain A1
#define twoGain A2


void setup() {
  // put your setup code here, to run once:
  analogWriteResolution(12);
  analogReadResolution(12);

  pinMode(noGain,INPUT);
  pinMode(oneGain,INPUT);
  pinMode(twoGain,INPUT);

}

//Create Large Sweep Packet
byte lSwpPckt[LSWP_LEN];

uint8_t syncAlpha = 0x15;
uint8_t syncBravo = 0x33;
uint8_t count = 1;
uint32_t startTime;
uint32_t endTime;
uint16_t checkSum = 50;
uint8_t stats = 1;

//1: vOut  2: No Gain Stage  3: First Gain Stage  4: Second Gain Stage
uint16_t voltageStep[4];
uint16_t vOut;

void loop() {
  // put your main code here, to run repeatedly:
  //Begin up-sweep at 1.007V (1250) End at 1.974V (2450)
  vOut = 1250;
  startTime = millis();
  for(int i = 0; i < 200; i++){
    analogWrite(vSweep,vOut);
    voltageStep[0] = vOut;
    voltageStep[1] = analogRead(noGain);
    voltageStep[2] = analogRead(oneGain);
    voltageStep[3] = analogRead(twoGain);

    memcpy( &lSwpPckt[LSWP_POS_STEP + (i * 2 * 4)], &voltageStep, 2 * 4);

    //Increase Voltage Out
    vOut+=6;
  }
  
  //Begin down-sweep at 1.974V (2450) End at 1.007V (1250)
  for(int i = 0; i < 200; i++){
    analogWrite(vSweep,vOut);
    voltageStep[0] = vOut;
    voltageStep[1] = analogRead(noGain);
    voltageStep[2] = analogRead(oneGain);
    voltageStep[3] = analogRead(twoGain);

    memcpy( &lSwpPckt[LSWP_POS_STEP + (i * 2 * 4)], &voltageStep, 2 * 4);

    vOut-=6;
  }
  endTime = millis();

  memcpy( &lSwpPckt[LSWP_POS_SYNC], &syncAlpha, 1);
  memcpy( &lSwpPckt[LSWP_POS_SYNC + 1], &syncBravo, 1);
  memcpy( &lSwpPckt[LSWP_POS_COUNT], &count, 2);
  memcpy( &lSwpPckt[LSWP_POS_TIME], &startTime, 4);
  memcpy( &lSwpPckt[LSWP_POS_TIME + 4], &endTime, 4);
  memcpy( &lSwpPckt[LSWP_POS_CHECKSUM], &checkSum, 2);
  memcpy( &lSwpPckt[LSWP_POS_STAT], &stats, 1);
  
  count++;
}
