#include <RF24.h>


enum Channels {THROTTLE, AILERON, ELEVATOR, RUDDER};

const int throttle_in  = A0; // X Input Pin of Analog 0
const int rudder_in    = A1; // Y Input Pin of Analog 1
const int elevator_in  = A2; // X Input Pin of Analog 2
const int aileron_in   = A3; // Y Input Pin of Analog 3

//const int KEYin = 3; // Push Button
int joy_min[4];
int joy_max[4];
int values[4];

int size_example[4] = {1, 1, 1, 1};
const size_t sVals = sizeof(size_example);

RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001";

const bool verbose_setup = false, verbose_loop = 1;

void setup() {
  //  pinMode (KEYin, INPUT);
  for (int i = 0; i < 4; i++) {
    joy_min[i] = 512;
    joy_max[i] = 512;
  }

  if (verbose_setup || verbose_loop)
    Serial.begin(9600);

  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
}

void loop() {
  values[THROTTLE] = analogRead (throttle_in);
  values[AILERON] = analogRead(aileron_in);
  values[ELEVATOR] = analogRead(elevator_in);
  values[RUDDER] = analogRead (rudder_in);

  // Continuously calibrate; maybe I'll add in something to trim the channels, but until then...
  for (int i = 0; i < 4; i++) {
    if (values[i] > joy_max[i])
      joy_max[i] = values[i];
    else if (values[i] < joy_min[i])
      joy_min[i] = values[i];

    values[i] = map(values[i], joy_min[i], joy_max[i], 0, 1023);
  }

  radio_write();
}

void radio_write() {
  if (verbose_loop) {
    Serial.print(values[THROTTLE]);
    Serial.print(',');
    Serial.print(values[AILERON]);
    Serial.print(',');
    Serial.print(values[ELEVATOR]);
    Serial.print(',');
    Serial.print(values[RUDDER]);
    Serial.print(',');
    Serial.println(radio.write(&values, sVals));
  } else {
    radio.write(&values, sVals);
  }
}

void throttle_locker() {
  values[THROTTLE] = analogRead (throttle_in);
  values[AILERON] = analogRead(aileron_in);
  values[ELEVATOR] = analogRead(elevator_in);
  values[RUDDER] = analogRead (rudder_in);

  while (values[THROTTLE] >= 100) {
    values[THROTTLE] = 1023;
    radio_write();
  }

  while (values[THROTTLE] <= 1000) {
    values[THROTTLE] = 0;
    radio_write();
  }
}
