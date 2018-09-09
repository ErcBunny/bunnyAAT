#include <SFE_BMP180.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>

#define offV -0.46

SFE_BMP180 baro;                          //create objects here
TinyGPSPlus gps;
SoftwareSerial gpsCom(3, 2);               //(rx,tx)

int i, vtsense, vt_int;                     //global variables to carry data between functions
double pr, la, lg, vt;
long pr_long, la_long, lg_long;
int raw[120];
int txdata[135] = { 0,0,0,0,0,0,0,0,0,0,1,1,1,1,1 };  //preload introns
char script[29] = {};

void setup() {
  pinMode(8, OUTPUT);                      //txdata 5
  pinMode(6, INPUT);                       //txsync 3
  pinMode(7, OUTPUT);                      //txenable 6
  digitalWrite(7, HIGH);
  pinMode(9, OUTPUT);                      //rxenable 8
  digitalWrite(9, LOW);
  pinMode(10, OUTPUT);                     //clockrate 18
  digitalWrite(10, HIGH);
  pinMode(11, OUTPUT);                     //4.8kselect 17
  digitalWrite(11, HIGH);
  pinMode(12, OUTPUT);                     //1.2k&2.4kselect 16
  digitalWrite(12, LOW);
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
  baro.begin();                           //start i2c com
  for(i = 0; i < 1200; i++){
    pr = kalmanFilter(getPressure(), 0.0005, 0.03);
  }
  digitalWrite(13, HIGH);
}

void loop() {
  input();              //get data from senseors
  loadScript();         //form an array to store data
  dtob();               //convert data into binary
  loadRedund();         //add CRC hash bits
  output();             //output txdata to fsk modem
}

void input() {
  vtsense = analogRead(A3);
  vt = vtsense * (5.0 / 1023.0);
  vt += offV;
  pr = kalmanFilter(getPressure(), 0.0005, 0.01);                     //get baro data
  gpsCom.begin(57600);
  i = 0;
  while (i == 0) {
    while (gpsCom.available() > 0) {
      if (gps.encode(gpsCom.read()) && gps.location.isUpdated()) {
        i = 1;
      }
    }
  }
  la = gps.location.lat();
  lg = gps.location.lng();                //get gps data
  gpsCom.end();
}

double getPressure() {                   //measure temperature and then pressure
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

void loadScript() {
  char pr_ch[7] = {};
  char la_ch[10] = {};
  char lg_ch[11] = {};
  char vt_ch[4] = {};
  pr_long = pr * 100;
  la_long = la * 1000000;
  lg_long = lg * 1000000;                     //convert them into long
  vt_int = vt * 100;
  itoa(vt_int, vt_ch, 10);
  ltoa(pr_long, pr_ch, 10);
  ltoa(la_long, la_ch, 10);
  ltoa(lg_long, lg_ch, 10);                 //write them into char arrays
  if (pr < 1000) {                            //add a zero to line digits up
    for (i = 4; i >= 0; i--) {
      pr_ch[i + 1] = pr_ch[i];
    }
    pr_ch[0] = '0';
  }
  if (la < 0) {
    la_ch[0] = '0';
    if (la > -10) {
      for (i = 7; i >= 1; i--) {
        la_ch[i + 1] = la_ch[i];
      }
      la_ch[1] = '0';
    }
  }
  if (la > 0) {                               //sign indicator (1+|0-)
    if (la >= 10) {
      for (i = 7; i >= 0; i--) {
        la_ch[i + 1] = la_ch[i];
      }
      la_ch[0] = '1';
    }
    else {
      for (i = 6; i >= 0; i--) {
        la_ch[i + 2] = la_ch[i];
      }
      la_ch[0] = '1';
      la_ch[1] = '0';
    }
  }
  if (lg < 0) {
    lg_ch[0] = '0';
    if (lg > -10) {
      for (i = 7; i >= 1; i--) {
        lg_ch[i + 2] = lg_ch[i];
      }
      lg_ch[1] = '0';
      lg_ch[2] = '0';
    }
    if (lg > -100 && lg <= -10) {
      for (i = 8; i >= 1; i--) {
        lg_ch[i + 1] = lg_ch[i];
      }
      lg_ch[1] = '0';
    }
  }
  if (lg > 0) {
    if (lg >= 100) {
      for (i = 9; i >= 0; i--) {
        lg_ch[i + 1] = lg_ch[i];
      }
      lg_ch[0] = '1';
    }
    if (lg >= 10 && lg < 100) {
      for (i = 7; i >= 0; i--) {
        lg_ch[i + 2] = lg_ch[i];
      }
      lg_ch[0] = '1';
      lg_ch[1] = '0';
    }
    if (lg < 10) {
      for (i = 6; i >= 0; i--) {
        lg_ch[i + 3] = lg_ch[i];
      }
      lg_ch[0] = '1';
      lg_ch[1] = '0';
      lg_ch[2] = '0';
    }
  }
  for (i = 0; i < 6; i++) {                          //bind separate arrays together as a script
    script[i] = pr_ch[i];
  }
  for (i = 0; i < 9; i++) {
    script[i + 6] = la_ch[i];
  }
  for (i = 0; i < 10; i++) {
    script[i + 15] = lg_ch[i];
  }
  for (i = 0; i < 3; i++) {
    script[i + 25] = vt_ch[i];
  }
}

void dtob() {                                //decimal to binary
  for (i = 0; i < 28; i++) {
    if (script[i] == '0') {
      raw[i * 4] = 0;
      raw[i * 4 + 1] = 0;
      raw[i * 4 + 2] = 0;
      raw[i * 4 + 3] = 0;
    }
    if (script[i] == '1') {
      raw[i * 4] = 0;
      raw[i * 4 + 1] = 0;
      raw[i * 4 + 2] = 0;
      raw[i * 4 + 3] = 1;
    }
    if (script[i] == '2') {
      raw[i * 4] = 0;
      raw[i * 4 + 1] = 0;
      raw[i * 4 + 2] = 1;
      raw[i * 4 + 3] = 0;
    }
    if (script[i] == '3') {
      raw[i * 4] = 0;
      raw[i * 4 + 1] = 0;
      raw[i * 4 + 2] = 1;
      raw[i * 4 + 3] = 1;
    }
    if (script[i] == '4') {
      raw[i * 4] = 0;
      raw[i * 4 + 1] = 1;
      raw[i * 4 + 2] = 0;
      raw[i * 4 + 3] = 0;
    }
    if (script[i] == '5') {
      raw[i * 4] = 0;
      raw[i * 4 + 1] = 1;
      raw[i * 4 + 2] = 0;
      raw[i * 4 + 3] = 1;
    }
    if (script[i] == '6') {
      raw[i * 4] = 0;
      raw[i * 4 + 1] = 1;
      raw[i * 4 + 2] = 1;
      raw[i * 4 + 3] = 0;
    }
    if (script[i] == '7') {
      raw[i * 4] = 0;
      raw[i * 4 + 1] = 1;
      raw[i * 4 + 2] = 1;
      raw[i * 4 + 3] = 1;
    }
    if (script[i] == '8') {
      raw[i * 4] = 1;
      raw[i * 4 + 1] = 0;
      raw[i * 4 + 2] = 0;
      raw[i * 4 + 3] = 0;
    }
    if (script[i] == '9') {
      raw[i * 4] = 1;
      raw[i * 4 + 1] = 0;
      raw[i * 4 + 2] = 0;
      raw[i * 4 + 3] = 1;
    }
  }
}

void loadRedund() {
  const int D = 112;                                     //D stands for data length
  const int R = 8;                                       //R stands for the degree of seed polynomial                                   
  const int seed[R + 1] = { 1,0,0,1,0,1,1,1,1 };             // CRC-8 0x2F
  int cache[R + 1];
  for (i = 112; i < 120; i++) {
    raw[i] = 0;
  }
  for (i = 0; i < R; i++) {
    cache[i] = seed[i + 1] ^ raw[i + 1];
  }
  cache[R] = raw[R + 1];
  for (int j = 1; j < D; j++) {
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
      cache[R] = raw[j + R + 1];
    }
    else {
      cache[R] = raw[j + R];
    }
  }
  for (i = 0; i < R; i++) {
    raw[i + D] = cache[i];
  }
  for (i = 0; i < 120; i++) {
    txdata[i + 15] = raw[i];
  }
}

void output() {                             //(HIGH 1|LOW 0) send bits at the falling edge of CLK
  digitalWrite(7, LOW);
  for (i = 0; i < 135; i++) {
    while (digitalRead(6) == HIGH);
    if (txdata[i] == 1) {
      digitalWrite(8, HIGH);
    }
    else {
      digitalWrite(8, LOW);
    }
    while (digitalRead(6) == LOW);
  }
  digitalWrite(7, HIGH);
  digitalWrite(8, LOW);
}

double kalmanFilter(const double data, double q, double r){
  static double x_last;
  static double p_last;
  double x_mid = x_last;
  double x_now;
  double p_mid;
  double p_now;
  double gain;
  x_mid = x_last;
  p_mid = p_last + q;
  gain = p_mid / (p_mid + r);
  x_now = x_mid + gain * (data - x_mid);
  p_now = (1 - gain) * p_mid;
  p_last = p_now;
  x_last = x_now;
  return x_now;
}
