#define offPitch 0
#include <Servo.h>

Servo pitch;

int angle;
char cache[3] = { 0 };
int i, j;

void setup() {
  Serial.begin(115200);
  pitch.attach(2);
  pitch.write(0 + offPitch);
  delay(2000);
}

void loop() {
  receive();
  act();
}

void receive() {
  i = 0;
  while (1) {
    if (Serial.available()) {
      cache[i] = Serial.read();
      i++;
      if (cache[i - 1] == 10) {
        break;
      }
    }
  }
  angle = atoi(cache);
  angle += offPitch;
}

void act() {
  pitch.write(angle);
}
