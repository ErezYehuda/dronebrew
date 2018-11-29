#include <RF24.h>


enum Channels {THROTTLE, AILERON, ELEVATOR, RUDDER};

const int rudder_in = A0; // X Input Pin of Analog 0
const int throttle_in = A1; // Y Input Pin of Analog 1
const int aileron_in = A2; // X Input Pin of Analog 2
const int elevator_in = A3; // Y Input Pin of Analog 3

//const int KEYin = 3; // Push Button
int values[4];

int size_example[4];
const size_t sVals = sizeof(size_example);

RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001";

void setup() {
  //  pinMode (KEYin, INPUT);
  Serial.begin(9600);

  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
}

void loop() {
  values[RUDDER] = analogRead (rudder_in);
  values[THROTTLE] = analogRead (throttle_in);
  values[AILERON] = analogRead(aileron_in);
  values[ELEVATOR] = analogRead(elevator_in);

  Serial.print(values[THROTTLE]);
  Serial.print(',');
  Serial.print(values[AILERON]);
  Serial.print(',');
  Serial.print(values[ELEVATOR]);
  Serial.print(',');
  Serial.print(values[RUDDER]);
  Serial.print(',');
  Serial.println(radio.write(&values, sVals));
}
