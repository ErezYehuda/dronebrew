#include <Servo.h>


enum Motor { FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT};
enum Channels {THROTTLE, AILERON, ELEVATOR, RUDDER};
int motor_ports[] = {9, 10, 11, 12};

const int Xin = A0; // X Input Pin
const int Yin = A1; // Y Input Pin
const int KEYin = 3; // Push Button
const int top = 2000;
const int bott = 710; // Technically, 700 might be acceptible, but it seems to be an unreliable for the ESCs to read

int xVal,yVal;

int values[4];
Servo motors [4];

void setup() {
  pinMode (KEYin, INPUT);
  motors[FRONT_LEFT].attach(motor_ports[FRONT_LEFT]);
}

void loop() {
  //  xVal = analogRead (Xin);
  yVal = analogRead (Yin);
  //  buttonVal = digitalRead (KEYin);
  if(yVal < 512) yVal = 512;

  values[THROTTLE] = map(yVal, 512, 1023, bott, top);

  motors[FRONT_LEFT].writeMicroseconds(values[THROTTLE]);

}
