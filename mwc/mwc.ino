//#define DEBUG
#define LEFT 180
#define STOP 90
#define RIGHT 0
#define offALT -31.8
#define K 5.5
#define offV 0.11

#include <SoftwareSerial.h>
#include <Wire.h>
#include <Servo.h>
#include <TinyGPS++.h>
#include <HMC5883L.h>
#include <MPU6050.h>
#include <SFE_BMP180.h>

SoftwareSerial gpsCom(8, 7);              //rx,tx
SoftwareSerial pitchCom(6, 5);
TinyGPSPlus gps;
Servo pan;
HMC5883L compass;
MPU6050 mpu;
SFE_BMP180 baro;

int i, m;
int rxdata[135] = {};
int Buffer[120] = {};
char script[29] = {};
float offX, offY, minX, maxX, minY, maxY;
float declination, vt, alt, aim, dst, bat, heading; //angle int->double
double pr, la, lg, angle, baseline;

void setup() {
  Serial.begin(115200);                  //initiate serial com
#ifndef DEBUG
  Serial.println(F("Antenna tracker version 0.0.2 beta by ErcBunny."));  //print info
  Serial.println(F("Initializing sensors..."));
#else
  Serial.println(F("Debug mode. Reading baseline pressure..."));
  Serial.println();
#endif
  mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G);
  mpu.setI2CMasterModeEnabled(false);
  mpu.setI2CBypassEnabled(true);
  mpu.setSleepEnabled(false);
  compass.begin();
  compass.setRange(HMC5883L_RANGE_1_3GA);
  compass.setMeasurementMode(HMC5883L_CONTINOUS);
  compass.setDataRate(HMC5883L_DATARATE_30HZ);
  compass.setSamples(HMC5883L_SAMPLES_8);
  baro.begin();                                           //setup sensors
  double Pressure, sum = 0;
  for(i = 0; i < 300; i++){
    Pressure = getPressure();
    sum += Pressure;
  }
  baseline = sum / 300;
  Serial.print(F("Done. Baseline pressure (in mb) is "));
  Serial.println(baseline);
  Serial.print(F("Enter declination value (E+|W-): "));   //wait for input
  char a[7] = { 0 };
  i = 0;
  while (1) {
    if (Serial.available()) {
      a[i] = Serial.read();
      i++;
      if (a[i - 1] == 10) {
        break;
      }
    }
  }
  declination = atof(a);
  Serial.println(declination);
  Serial.println(F("Touch continue to start calibration.")); //wait for command
  while (1) {
    if (Serial.available()) {
      i = Serial.read();
      if (i == '0') {
        break;
      }
    }
  }
  calibrate();
  delay(500);
  Serial.println(F("Successful."));
#ifndef DEBUG
  gpsCom.begin(57600);
  Serial.println(F("Waiting for positioning service..."));  //wait till gps is ready
  i = 0;
  while (i == 0) {
    while (gpsCom.available() > 0) {
      if (gps.encode(gpsCom.read()) && gps.location.isUpdated()) {
        i = 1;
      }
    }
  }
  gpsCom.end();
#endif
  pinMode(3, INPUT);        //CLKED DATA 12
  pinMode(10, INPUT);       //CARRIER DETECT 13
  pinMode(11, INPUT);       //RXSYNC 15
#ifndef DEBUG
  Serial.println(F("System setup complete. Touch continue to start using."));
  while (1) {
    if (Serial.available()) {
      i = Serial.read();
      if (i == '0') {
        Serial.println(F("Waiting for data..."));
        break;
      }
    }
  }
  Serial.end();
#endif
}

void loop() {
#ifndef DEBUG
  while (digitalRead(10) == HIGH);
  while (digitalRead(10) == LOW);  //skip incomplete data
  input();                      //store data in rxdata[135]
  check();                      //crc check
  if (m == 0) {
    btod();                     //bin to dec
    extract();                  //store data in variable after checking domain
    calculate();                //calculate dst angle etc.
    output();                   //print info and drive servos
  }
#else
  Serial.begin(115200);
  Serial.println(F("Make sure the aircraft is connected and continue..."));
  while (1) {
    if (Serial.available()) {
      i = Serial.read();
      if (i == '0') {
        break;
      }
    }
  }
  Serial.println(F("Connecting..."));
  Serial.end();
  float sum_alt = 0;
  for (int j = 0; j < 100; j++) {
    while (digitalRead(10) == HIGH);
    while (digitalRead(10) == LOW);
    input();
    check();
    if (m == 0) {
      btod();
      extract();
      calculate();
    }
    sum_alt = alt + sum_alt;                       //sample offalt value 100 times and calculate average
    Serial.begin(115200);
    Serial.print(j);
    Serial.println(F("%"));
    Serial.end();
  }
  Serial.begin(115200);
  Serial.println(F("100%"));
  Serial.print(F("#define offALT "));
  Serial.println(-sum_alt * 0.01);
  Serial.println(F("Touch continue to move on..."));
  while (1) {
    if (Serial.available()) {
      i = Serial.read();
      if (i == '0') {
        break;
      }
    }
  }
  Serial.println(F("Analyzing mechanics..."));
  float heading_temp;
  float sum_k = 0;
  float diff;
  pitchCom.begin(115200);
  pitchCom.println(60);
  pitchCom.end();
  delay(5000);
  pan.attach(9);
  for (i = 1; i < 12; i++) {
    getHeading();
    heading_temp = heading;
    pan.write(RIGHT);
    delay(i * 150);
    pan.write(STOP);
    Serial.print(i * 9.09);
    Serial.println(F("%"));
    delay(1000);
    getHeading();
    if (heading > heading_temp) {
      diff = heading - heading_temp;
    }
    else {
      diff = 360 - (heading_temp - heading);
    }
    sum_k = (i * 150) / diff + sum_k;
    delay(1000);                                    //calculate servo speed constant k*angle(in degree)=delay time
  }                                                 //kâ†‘ delayâ†‘
  pan.detach();
  Serial.println(F("100%"));
  Serial.print(F("#define K "));
  Serial.println(sum_k / 11);
  Serial.println(F("Continue to run debug again... "));
  while (1) {
    if (Serial.available()) {
      i = Serial.read();
      if (i == '0') {
        break;
      }
    }
  }
  Serial.println();
  Serial.end();
#endif
}

void calibrate() {
  Serial.println(F("Calibrating..."));       //autonomously calibrate compass
  pan.attach(9);
  pan.write(RIGHT);
  for (i = 0; i < 1000; i++) {
    Vector mag = compass.readNormalize();
    if (mag.XAxis < minX) minX = mag.XAxis;
    if (mag.XAxis > maxX) maxX = mag.XAxis;
    if (mag.YAxis < minY) minY = mag.YAxis;
    if (mag.YAxis > maxY) maxY = mag.YAxis;
    offX = (maxX + minX) / 2;
    offY = (maxY + minY) / 2;
    delay(2);
  }
  pan.write(STOP);
  delay(500);
  pan.detach();
}

double getPressure() {                       //read barometer
  char status;
  double T, P, p0, a;
  status = baro.startTemperature();
  if (status != 0) {
    delay(status);
    status = baro.getTemperature(T);
    if (status != 0) {
      status = baro.startPressure(3);
      if (status != 0) {
        delay(status);
        status = baro.getPressure(P, T);
        if (status != 0) {
          return(P);
        }
      }
    }
  }
}

void input() {
  int bitBuffer;
  for (i = 0; i < 135; i++) {
    while (digitalRead(11) == HIGH);
    if (digitalRead(3) == HIGH) {
      bitBuffer = 1;
    }
    else {
      bitBuffer = 0;
    }
    rxdata[i] = bitBuffer;
    while (digitalRead(11) == LOW);
  }
}

void check() {
  const int D = 112;
  const int R = 8;
  const int seed[R + 1] = { 1,0,0,1,0,1,1,1,1 };
  int n;
  for (i = 0; i < 10; i++) {
    if (rxdata[i] == 1 && rxdata[i + 1] == 1 && rxdata[i + 2] == 1 && rxdata[i + 3] == 1 && rxdata[i + 4] == 1) {
      break;
    }
  }
  n = i + 5;
  for (i = 0; i < 120; i++) {
    Buffer[i] = rxdata[i + n];
  }                                                   //introns removed
  int cache[R + 1];
  for (i = 0; i < R; i++) {
    cache[i] = seed[i + 1] ^ Buffer[i + 1];
  }
  cache[R] = Buffer[R + 1];
  for (int j = 1; j < D; j++) {                               //112+8-9-1
    if (cache[0] == 0) {
      for (i = 0; i < R; i++) {
        cache[i] = 0 ^ cache[i + 1];
      }
    }
    else {
      for (i = 0; i < R; i++) {
        cache[i] = seed[i + 1] ^ cache[i + 1];
      }
    }
    if (j < D - 1) {
      cache[R] = Buffer[j + R + 1];
    }
    else {
      cache[R] = Buffer[j + R];
    }
  }
  int rem = 0;
  for (i = 0; i < R; i++) {
    rem = rem + cache[i];
  }
  if (rem == 0) {
    m = 0;
  }
  else {
    m = 1;
  }
}

void btod() {
  for (i = 0; i < 28; i++) {
    if (Buffer[i * 4] == 0 && Buffer[i * 4 + 1] == 0 && Buffer[i * 4 + 2] == 0 && Buffer[i * 4 + 3] == 0) {
      script[i] = '0';
    }
    if (Buffer[i * 4] == 0 && Buffer[i * 4 + 1] == 0 && Buffer[i * 4 + 2] == 0 && Buffer[i * 4 + 3] == 1) {
      script[i] = '1';
    }
    if (Buffer[i * 4] == 0 && Buffer[i * 4 + 1] == 0 && Buffer[i * 4 + 2] == 1 && Buffer[i * 4 + 3] == 0) {
      script[i] = '2';
    }
    if (Buffer[i * 4] == 0 && Buffer[i * 4 + 1] == 0 && Buffer[i * 4 + 2] == 1 && Buffer[i * 4 + 3] == 1) {
      script[i] = '3';
    }
    if (Buffer[i * 4] == 0 && Buffer[i * 4 + 1] == 1 && Buffer[i * 4 + 2] == 0 && Buffer[i * 4 + 3] == 0) {
      script[i] = '4';
    }
    if (Buffer[i * 4] == 0 && Buffer[i * 4 + 1] == 1 && Buffer[i * 4 + 2] == 0 && Buffer[i * 4 + 3] == 1) {
      script[i] = '5';
    }
    if (Buffer[i * 4] == 0 && Buffer[i * 4 + 1] == 1 && Buffer[i * 4 + 2] == 1 && Buffer[i * 4 + 3] == 0) {
      script[i] = '6';
    }
    if (Buffer[i * 4] == 0 && Buffer[i * 4 + 1] == 1 && Buffer[i * 4 + 2] == 1 && Buffer[i * 4 + 3] == 1) {
      script[i] = '7';
    }
    if (Buffer[i * 4] == 1 && Buffer[i * 4 + 1] == 0 && Buffer[i * 4 + 2] == 0 && Buffer[i * 4 + 3] == 0) {
      script[i] = '8';
    }
    if (Buffer[i * 4] == 1 && Buffer[i * 4 + 1] == 0 && Buffer[i * 4 + 2] == 0 && Buffer[i * 4 + 3] == 1) {
      script[i] = '9';
    }
  }
}

void extract() {
  int vt_int;
  long pr_long, la_long, lg_long;
  double la_base, lg_base;
  char pr_ch[7] = {};
  char la_ch[10] = {};
  char lg_ch[11] = {};
  char vt_ch[4] = {};
  for (i = 0; i < 6; i++) {
    pr_ch[i] = script[i];
  }
  for (i = 0; i < 8; i++) {
    la_ch[i] = script[i + 7];
  }
  for (i = 0; i < 9; i++) {
    lg_ch[i] = script[i + 16];
  }
  for (i = 0; i < 3; i++) {
    vt_ch[i] = script[i + 25];
  }
  pr_long = atol(pr_ch);
  la_long = atol(la_ch);
  lg_long = atol(lg_ch);
  vt_int = atoi(vt_ch);
  if (pr_long < 110000 && pr_long>89000) {
    pr = pr_long * 0.01;
  }
  vt = vt_int * 0.01;
  if (script[6] == '0') {
    la_long = la_long * (-1);
  }
  if (script[15] == '0') {
    lg_long = lg_long * (-1);
  }
  if (la_long >= -90000000 &&
    la_long <= 90000000 &&
    lg_long >= -180000000 &&
    lg_long <= 180000000
    ) {
    la = la_long * 0.000001;
    lg = lg_long * 0.000001;
  }
}

void calculate() {
#ifndef DEBUG
  int bat_sense = analogRead(A2);
  bat = bat_sense * (5.0 / 1023.0);
  bat += offV;
  alt = baro.altitude(pr, baseline);
  alt = alt + offALT;
  gpsCom.begin(57600);
  i = 0;
  while (i == 0) {
    while (gpsCom.available() > 0) {
      if (gps.encode(gpsCom.read()) && gps.location.isUpdated()) {
        i = 1;
      }
    }
  }
  dst = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), la, lg);
  aim = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), la, lg);
  gpsCom.end();
  angle = (atan(alt/dst) * 180) / M_PI;       //cancel compensation
#else
  alt = baro.altitude(pr, baseline);
#endif
}

void output() {
  int delta;
  if (angle >= 0) {
    pitchCom.begin(115200);
    pitchCom.println(int(angle));
    pitchCom.end();
  }
  else {
    angle = 0;
    pitchCom.begin(115200);
    pitchCom.println(0);
    pitchCom.end();
  }
  getHeading();
  if (heading < 180) {
    if (aim >= heading + 180 && aim < 360) {
      delta = K * (360 - aim + heading);
      pan.attach(9);
      pan.write(LEFT);
      delay(delta);
      pan.write(STOP);
    }
    else {
      if (aim < heading) {
        delta = K * (heading - aim);
        pan.attach(9);
        pan.write(LEFT);
        delay(delta);
        pan.write(STOP);
      }
      else {
        delta = K * (aim - heading);
        pan.attach(9);
        pan.write(RIGHT);
        delay(delta);
        pan.write(STOP);
      }
    }
  }
  else {
    if (aim >= 0 && aim < heading - 180) {
      delta = K * (360 - heading + aim);
      pan.attach(9);
      pan.write(RIGHT);
      delay(delta);
      pan.write(STOP);
    }
    else {
      if (aim < heading) {
        delta = K * (heading - aim);
        pan.attach(9);
        pan.write(LEFT);
        delay(delta);
        pan.write(STOP);
      }
      else {
        delta = K * (aim - heading);
        pan.attach(9);
        pan.write(RIGHT);
        delay(delta);
        pan.write(STOP);
      }
    }
  }
  delay(100);
  pan.detach();
  Serial.begin(115200);
  Serial.println();
  Serial.print(F("DST:"));
  Serial.print(dst, 1);
  Serial.print(F(" ALT:"));
  Serial.print(alt, 1);
  Serial.print(F(" V_air:"));
  Serial.print(vt, 1);
  Serial.print(F(" V_gnd:"));
  Serial.println(bat, 1);
  Serial.end();
}

void getHeading() {
  float Xh, Yh, roll, pitch, cosRoll, sinRoll, cosPitch, sinPitch;
  Vector mag = compass.readNormalize();
  Vector acc = mpu.readScaledAccel();
  roll = asin(acc.YAxis);
  pitch = asin(-acc.XAxis);
  if (roll > 0.78 || roll<0.78 || pitch>0.78 || pitch < 0.78) {
    cosRoll = cos(roll);
    sinRoll = sin(roll);
    cosPitch = cos(pitch);
    sinPitch = sin(pitch);
    Xh = mag.XAxis * cosPitch + mag.ZAxis * sinPitch;
    Yh = mag.XAxis * sinRoll * sinPitch + mag.YAxis * cosRoll - mag.ZAxis * sinRoll * cosPitch;
  }
  Xh = mag.XAxis - offX;
  Yh = mag.YAxis - offY;
  if (Xh > 0 && Yh > 0) {
    heading = (180 * atan2(Yh, Xh)) / M_PI + declination + 270;
  }
  else if (Xh > 0 && Yh < 0) {
    heading = (180 * atan2(Xh, abs(Yh))) / M_PI + declination + 180;
  }
  else if (Xh < 0 && Yh < 0) {
    heading = (180 * atan2(abs(Yh), abs(Xh))) / M_PI + declination + 90;
  }
  else if (Xh < 0 && Yh > 0) {
    heading = (180 * atan2(abs(Xh), Yh)) / M_PI + declination;
  }
  else if (Xh == 0 && Yh > 0) {
    heading = 0.00;
  }
  else if (Xh == 0 && Yh < 0) {
    heading = 180.00;
  }
  else if (Xh < 0 && Yh == 0) {
    heading = 90.00;
  }
  else if (Xh == 0 && Yh == 0) {
    heading = 270.00;
  }
  if (heading < 0) {
    heading += 360.00;
  }
}
