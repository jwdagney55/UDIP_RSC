#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
#include <Adafruit_Sensor.h>
#include <SD.h>

//Define Packet Types
#define TYPE_SENS 0x01
#define TYPE_MED_SWP 0x10
#define TYPE_LRG_SWP 0x11
#define TYPE_BRST_SWP 0x20

//Packet Header Positions
#define HEDR_POS_SYNC 0
#define HEDR_POS_COUNT 2
#define HEDR_POS_T_INITIAL 4
#define HEDR_POS_T_FINAL 8
#define HEDR_POS_TYPE 12

#define HEDR_LEN 13
#define PCKT_SYNC_0 0x55 //ASCII 'U'
#define PCKT_SYNC_1 0x44 //ASCII 'D'

//Sensor Payload Positions
#define SENS_POS_ACCEL_D      0
#define SENS_POS_ACCEL_A      6
#define SENS_POS_GYRO         8
#define SENS_POS_MAG         14
#define SENS_POS_TMP_D       20
#define SENS_POS_TMP_A       22
#define SENS_POS_ACCEL_SCALE 24

#define SENS_LEN 25

//Sweep Packet DAC Value Arrays
#define ZERO_VOLT_DAC 1773
//#define MED_SWP_DAC 
/*{
 *   0,  1, ... ,  25,  24, ... , 0,
 *  -1, -2, ... , -25, -24, ... , 0
 * 101 values for entire range, 0V to 6V to -6V to 0V
 */
#define N_MED_SWP_STEP 253
const uint16_t medSwpDAC[N_MED_SWP_STEP] = {
                                            ZERO_VOLT_DAC //UP to +6V
                                            ,1780,1787,1794,1802,1809,1816,1824,1831,1838,1846,1853,1860,1868,1875,1882,1890,1897,1904,1912,1919,1926,1934,1941,1948,1956,1963,1970,1978
                                            ,1985,1992,1999,2007,2014,2021,2029,2036,2043,2051,2058,2065,2073,2080,2087,2095,2102,2109,2117,2124,2131,2139,2146,2153,2161,2168,2175,2183
                                            ,2190,2197,2204,2212,2219,2226,2234
                                                          //Down to 0V
                                            ,2226,2219,2212,2204,2197,2190,2183,2175,2168,2161,2153,2146,2139,2131,2124,2117,2109,2102,2095,2087,2080,2073,2065,2058,2051,2043,2036,2029
                                            ,2021,2014,2007,1999,1992,1985,1978,1970,1963,1956,1948,1941,1934,1926,1919,1912,1904,1897,1890,1882,1875,1868,1860,1853,1846,1838,1831,1824
                                            ,1816,1809,1802,1794,1787,1780
                                            ,ZERO_VOLT_DAC
                                                          //Down to -6V
                                            ,1765,1758,1751,1743,1736,1729,1721,1714,1707,1699,1692,1685,1677,1670,1663,1655,1648,1641,1633,1626,1619,1611,1604,1597,1589,1582,1575,1568
                                            ,1560,1553,1546,1538,1531,1524,1516,1509,1502,1494,1487,1480,1472,1465,1458,1450,1443,1436,1428,1421,1414,1406,1399,1392,1384,1377,1370,1363
                                            ,1355,1348,1341,1333,1326,1319
                                                          //Up to 0V
                                            ,1311,1319,1326,1333,1341,1348,1355,1363,1370,1377,1384,1392,1399,1406,1414,1421,1428,1436,1443,1450,1458,1465,1472,1480,1487,1494,1502,1509
                                            ,1516,1524,1531,1538,1546,1553,1560,1568,1575,1582,1589,1597,1604,1611,1619,1626,1633,1641,1648,1655,1663,1670,1677,1685,1692,1699,1707,1714
                                            ,1721,1729,1736,1743,1751,1758,1765
                                            ,ZERO_VOLT_DAC
                                            };

//#define LRG_SWP_DAC 
/*{
 *   0,  1, ... ,  50,  49, ... , 0,
 *  -1, -2, ... , -50, -49, ... , 0
 * 201 values for entire range, 0V to 9V to -9V to 0V
 */
#define N_LRG_SWP_STEP 253
const uint16_t lrgSwpDAC[N_LRG_SWP_STEP] = {
                                            ZERO_VOLT_DAC //UP to +9V
                                            ,1783,1794,1805,1816,1827,1838,1849,1860,1871,1882,1893,1904,1915,1926,1937,1948,1959,1970,1981,1992,2003,2014,2025,2036,2047,2058,2069,2080
                                            ,2091,2102,2113,2124,2135,2146,2157,2168,2179,2190,2201,2212,2223,2234,2245,2256,2267,2278,2289,2300,2311,2322,2333,2344,2355,2366,2377,2388
                                            ,2398,2409,2420,2431,2442,2453,2464
                                                          //DOWN to 0V
                                            ,2453,2442,2431,2420,2409,2398,2388,2377,2366,2355,2344,2333,2322,2311,2300,2289,2278,2267,2256,2245,2234,2223,2212,2201,2190,2179,2168,2157
                                            ,2146,2135,2124,2113,2102,2091,2080,2069,2058,2047,2036,2025,2014,2003,1992,1981,1970,1959,1948,1937,1926,1915,1904,1893,1882,1871,1860,1849
                                            ,1838,1827,1816,1805,1794,1783
                                            ,ZERO_VOLT_DAC
                                                          //DOWN to -9V
                                            ,1762,1751,1740,1729,1718,1707,1696,1685,1674,1663,1652,1641,1630,1619,1608,1597,1586,1575,1564,1553,1542,1531,1520,1509,1498,1487,1476,1465
                                            ,1454,1443,1432,1421,1410,1399,1388,1377,1366,1355,1344,1333,1322,1311,1300,1289,1278,1267,1256,1245,1234,1223,1212,1201,1190,1179,1168,1158
                                            ,1147,1136,1125,1114,1103,1092
                                                          //Up to 0V
                                            ,1081,1092,1103,1114,1125,1136,1147,1158,1168,1179,1190,1201,1212,1223,1234,1245,1256,1267,1278,1289,1300,1311,1322,1333,1344,1355,1366,1377
                                            ,1388,1399,1410,1421,1432,1443,1454,1465,1476,1487,1498,1509,1520,1531,1542,1553,1564,1575,1586,1597,1608,1619,1630,1641,1652,1663,1674,1685
                                            ,1696,1707,1718,1729,1740,1751,1762
                                            ,ZERO_VOLT_DAC
                                            };

//#define BRST_SWP_DAC 
/*{
 *   0,  1,  2,  3,  4,  5, 4,  3,  2,  1,  0,
 *   -1, -2, -3, -4, -5, -4,-3, -2, -1,  0
 *}
 * 21 values for entire range, 0V to 3V to -3V to 0V
 */
#define N_BRST_SWP_STEP 127
#define N_BRST_SWP 10
const uint16_t brstSwpDAC[N_BRST_SWP_STEP] = {
                                              ZERO_VOLT_DAC //UP to +2V
                                              ,1777,1785,1792,1800,1807,1815,1822,1830,1837,1844,1852
                                              ,1859,1867,1874,1882,1889,1896,1904,1911,1919,1926
                                                            //DOWN to 0V
                                              ,1919,1911,1904,1896,1889,1882,1874,1867,1859,1852,1844
                                              ,1837,1830,1822,1815,1807,1800,1792,1785,1777
                                              ,ZERO_VOLT_DAC
                                                            //Down to -4V
                                              ,1770,1763,1755,1748,1740,1733,1725,1718,1711,1703,1696
                                              ,1688,1681,1673,1666,1658,1651,1644,1636,1629,1621,1614
                                              ,1606,1599,1591,1584,1577,1569,1562,1554,1547,1539,1532
                                              ,1525,1517,1510,1502,1495,1487,1480,1472
                                                            //Up to 0V
                                              ,1465,1472,1480,1487,1495,1502,1510,1517,1525,1532,1539
                                              ,1547,1554,1562,1569,1577,1584,1591,1599,1606,1614,1621
                                              ,1629,1636,1644,1651,1658,1666,1673,1681,1688,1696,1703
                                              ,1711,1718,1725,1733,1740,1748,1755,1763,1770
                                              ,ZERO_VOLT_DAC
                                              };

#define MAX_SWP_STEP 1270 //from 10 burst sweeps
#define MAX_SWP_LEN MAX_SWP_STEP * 2 * 3

//Define Arduino Due Pins for Voltage Sweep
#define SWEEP_PIN DAC0
#define ADC_G0 A0
#define ADC_G1 A1
#define ADC_G2 A2

//LSM9DS1
LSM9DS1 lsm;
int16_t acc[3];
int16_t gyr[3];
int16_t mag[3];
int16_t temp;

//MMA1210
#define PIN_MMA A9
uint16_t mmaInput;


//TMP36
#define PIN_TMP A8
uint16_t tmpInput;

//LED PINS
#define SWP_LED 12
#define SENS_LED 13

//Function Declarations
//Packet Functions
void makeHedr(byte *, uint16_t *, byte);
void writePckt(File, byte *, uint16_t);

//Sensor Packet Functions
void makeSensPckt(byte *, uint16_t *);
void makeSensPyld(byte *);

//Sweep Packet Functions
void makeSweepPckt(byte *, uint16_t *, byte);
void makeMedSweep(byte *, uint16_t *);
void makeLrgSweep(byte *, uint16_t *);
void makeBrstSweep(byte *, uint16_t *);
void doStep(byte *, uint16_t, uint16_t *, int);


//Global Variables
File myFile;
String fileName = "UDIP0000.DAT";
byte sensPckt[HEDR_LEN + SENS_LEN];
byte swpPckt[HEDR_LEN + MAX_SWP_LEN];
uint16_t count = 0;
unsigned long tInitial;
unsigned long tFinal;
  

void setup() {
  // put your setup code here, to run once:
  /*
  Serial.begin(9600);
  while(!Serial){
    ;
  }
  */
  
  analogWriteResolution(12);
  analogReadResolution(12);
  pinMode(ADC_G0,INPUT);
  pinMode(ADC_G1,INPUT);
  pinMode(ADC_G2,INPUT);
  
  if(!SD.begin(10)){
    //Serial.println("sd card initialization failed");
    //blink led so we know it failed
    while(1){
      digitalWrite(SWP_LED, HIGH);
      delay(100);
      digitalWrite(SWP_LED, LOW);
      delay(100); 
    }
  }

  myFile = SD.open(fileName, FILE_WRITE);
  
  Wire.begin();

  if (!lsm.begin()) // with no arguments, this uses default addresses (AG:0x6B, M:0x1E) and i2c port (Wire).
  {
    while (1){
      digitalWrite(SENS_LED, HIGH);
      delay(100);
      digitalWrite(SENS_LED, LOW);
      delay(100); 
    }
  }
  // 1.) Set the accelerometer range to 16,
  lsm.setAccelScale(16);  
  
  // 2.) Set the magnetometer sensitivity
  lsm.setMagScale(4);

  // 3.) Setup the gyroscope
  lsm.setGyroScale(2000);

  analogReadResolution(12);
}

void loop() {
  if(count>1000){
    //Serial.println("Count is greater than 50");
    myFile.close();
    while(1);    
  }
  else{
    // 3 big sweeps, 3 medium sweeps, 1 burst sweep, 1 sensor packet
    for(int i = 0; i < 3; i++){
      makeSweepPckt(swpPckt, &count, TYPE_LRG_SWP);
      writePckt(myFile, swpPckt, HEDR_LEN + (N_LRG_SWP_STEP * 2 * 3));
      digitalWrite(SWP_LED,HIGH);
      digitalWrite(SWP_LED,LOW);
    }
    for(int i = 0; i < 3; i++){
      makeSweepPckt(swpPckt, &count, TYPE_MED_SWP);
      writePckt(myFile, swpPckt, HEDR_LEN + (N_MED_SWP_STEP * 2 * 3));
      digitalWrite(SWP_LED,HIGH);
      digitalWrite(SWP_LED,LOW);
    }
    makeSweepPckt(swpPckt, &count, TYPE_BRST_SWP);
    writePckt(myFile, swpPckt, HEDR_LEN + (N_BRST_SWP_STEP * N_BRST_SWP * 2 * 3));
    digitalWrite(SWP_LED,HIGH);
    digitalWrite(SWP_LED,LOW);

    makeSensPckt(sensPckt, &count);
    digitalWrite(SENS_LED,HIGH);
    digitalWrite(SENS_LED,LOW);
  }
}

void makeHedr(byte *pckt, uint16_t *count, byte type){
  //Serial.println("Making Header");
  tInitial = millis();
  pckt[HEDR_POS_SYNC] = PCKT_SYNC_0;
  pckt[HEDR_POS_SYNC + 1] = PCKT_SYNC_1;

  memcpy(&pckt[HEDR_POS_COUNT], count, 2);
  (*count)++;

  memcpy(&pckt[HEDR_POS_T_INITIAL], &tInitial, 4);

  pckt[HEDR_POS_TYPE] = type;
  return;
}

void makeSensPckt(byte *pckt, uint16_t *count){
  makeHedr(pckt, count, TYPE_SENS);
  makeSensPyld(pckt);
  tFinal = millis();

  memcpy(&pckt[HEDR_POS_T_FINAL], &tFinal , 4);
  
  return;
}

void makeSensPyld(byte *pckt){
  //Read 9DOF
  if(lsm.accelAvailable()){
    lsm.readAccel();
  }
  acc[0] = lsm.ax; acc[1] = lsm.ay; acc[2] = lsm.az;

  if(lsm.gyroAvailable()){
    lsm.readGyro();
  }
  gyr[0] = lsm.gx; gyr[1] = lsm.gy; gyr[2] = lsm.gz;

  if(lsm.magAvailable()){
    lsm.readMag();
  }
  mag[0] = lsm.mx; mag[1] = lsm.my; mag[2] = lsm.mz;

  if(lsm.tempAvailable()){
    lsm.readTemp();
  }
  temp = lsm.temperature;

  //Read analog pins
  mmaInput = analogRead(PIN_MMA);
  tmpInput = analogRead(PIN_TMP);

  memcpy(&pckt[HEDR_LEN + SENS_POS_ACCEL_D], acc, 6);
  memcpy(&pckt[HEDR_LEN + SENS_POS_ACCEL_A], &mmaInput, 2);
  memcpy(&pckt[HEDR_LEN + SENS_POS_GYRO], gyr, 6);
  memcpy(&pckt[HEDR_LEN + SENS_POS_MAG], mag, 6);
  memcpy(&pckt[HEDR_LEN + SENS_POS_TMP_D], &temp, 2);
  memcpy(&pckt[HEDR_LEN + SENS_POS_TMP_A], &tmpInput, 2);

  return;
}

void makeSweepPckt(byte *pckt, uint16_t *count, byte sweepType){
  makeHedr(pckt, count, sweepType);
  uint16_t voltageStep[3] = {0,0,0};
  
  if(sweepType == 0x10){
    //Medium Sweep
    makeMedSweep(pckt, voltageStep);
  }
  else if(sweepType == 0x11){
    //LargeSweep
    makeLrgSweep(pckt, voltageStep);
  }
  else if(sweepType == 0x20){
    //Burst sweep
    makeBrstSweep(pckt, voltageStep);
  }
  tFinal = millis();
  memcpy(&pckt[HEDR_POS_T_FINAL], &tFinal, 4);
}

void makeMedSweep(byte *pckt, uint16_t *voltageStep){
  //Serial.println("Making med sweep packets");
  //Serial.flush();
  for(int i = 0; i < N_MED_SWP_STEP; i++){
    doStep(pckt, medSwpDAC[i], voltageStep, i * 2 *3);
    }
    return;
}

void makeLrgSweep(byte *pckt, uint16_t *voltageStep){
  //Serial.println("Making lrg sweep packets");
  //Serial.flush();
  for(int i = 0; i < N_LRG_SWP_STEP; i++){
    doStep(pckt, lrgSwpDAC[i], voltageStep, i * 2 * 3);
    }
    return;
}

void makeBrstSweep(byte *pckt, uint16_t *voltageStep){
  //Serial.println("Making brst sweep packets");
  //Serial.flush();
  for(int i = 0; i < N_BRST_SWP; i++){
    for(int j = 0; j < N_BRST_SWP_STEP; j++){
      doStep(pckt, brstSwpDAC[j], voltageStep, i * N_BRST_SWP_STEP * 2 * 3 + j * 2 * 3);
    }
  }
  return;
}

void doStep(byte *pckt, uint16_t dacLevel, uint16_t *stepBuf, int loc){
  stepBuf[0] = 0;
  stepBuf[1] = 0;
  stepBuf[2] = 0;
  //analogWrite(SWEEP_PIN,dacLevel);
  for(int i = 0; i < 4; i++){
    stepBuf[0] += analogRead(ADC_G0);
    stepBuf[1] += analogRead(ADC_G1);
    stepBuf[2] += analogRead(ADC_G2);
  }
  memcpy( &pckt[HEDR_LEN + loc], stepBuf, 2 * 3 );
  
  return;
}

void writePckt(File f , byte *pckt, uint16_t pcktLen){
  f.write(pckt, pcktLen);
  return;
}
