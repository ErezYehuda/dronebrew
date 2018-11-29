#include <RF24.h>
#include <Servo.h>

enum Channels {THROTTLE, AILERON, ELEVATOR, RUDDER};
// Throttle->All
// Aileron=Roll->Left vs. Right
// Elevator=Pitch->Front vs. Back
// Rudder=Yaw->CW vs. CCW

enum Motor { FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT};
const int motor_ports[] = {9, 10, 11, 12};
Servo motors [4];

const int top = 2000;
const int bott = 710; // Technically, 700 might be acceptible, but it seems to be unreliable for the ESCs to read

int values[4];
float distrs[4]; // Distributions of throttle

const int size_example[4];
const size_t vals_size = sizeof(size_example);

RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001";
bool verbose_setup, verbose_loop;

void setup() {
  Serial.begin(9600);
  verbose_setup = false;
  verbose_loop = true;

  for (int i = 0; i < 4; i++)
    motors[i].attach(motor_ports[i]);

  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  Serial.println("</setup>");

  //  while (!radio.available()); // Wait for radio
  for (int i = 0; i < 10; i++)
    if (radio.available())
      radio.read(&values, vals_size);
    else i -= 1;
  if (verbose_setup) {
    Serial.print("Initial Throttle: ");
    Serial.println(values[THROTTLE]);
  }
  if (values[THROTTLE] >= 1020) {
    Serial.println("Configuring ESCs");
    // If it's within 25 of the top Y-position of the left joystick...
    // Write the undistributed throttle to the motors until it gets below 4
    do {
      if (radio.available())
        radio.read(&values, vals_size);
      for (int i = 0; i < 4; i++)
        motors[i].writeMicroseconds(map(values[THROTTLE], 0, 1023, 710, 2000));
      if (verbose_setup)
        Serial.println(values[THROTTLE]);
    } while (values[THROTTLE] > 4);
  }
  Serial.println("Ending ESC configuration");
}

void loop() {
  if (radio.available()) {
    radio.read(&values, vals_size);

    if (verbose_loop) {
      Serial.print(values[THROTTLE]);
      Serial.print(',');
      Serial.print(values[AILERON]);
      Serial.print(',');
      Serial.print(values[ELEVATOR]);
      Serial.print(',');
      Serial.println(values[RUDDER]);
    }
    // Throttle->All
    // Aileron=Roll->Left vs. Right
    // Elevator=Pitch->Front vs. Back
    // Rudder=Yaw->CW vs. CCW

    float throttle = values[THROTTLE] / 1023.0;
    float aileron  = values[AILERON]  / 1023.0;
    float elevator = values[ELEVATOR] / 1023.0;
    float rudder   = values[RUDDER]   / 1023.0;

    for (int i = 0; i < 4; i++)
      distrs[i] = throttle;

    distrs[FRONT_RIGHT] *= aileron;
    distrs[BACK_RIGHT]  *= aileron;
    distrs[FRONT_LEFT]  *= 1 - aileron;
    distrs[BACK_LEFT]   *= 1 - aileron;

    distrs[BACK_LEFT]   *= elevator;
    distrs[BACK_RIGHT]  *= elevator;
    distrs[FRONT_LEFT]  *= 1 - elevator;
    distrs[FRONT_RIGHT] *= 1 - elevator;

    distrs[BACK_LEFT]   *= rudder;
    distrs[FRONT_RIGHT] *= rudder;
    distrs[FRONT_LEFT]  *= 1 - rudder;
    distrs[BACK_RIGHT]  *= 1 - rudder;

    for (int i = 0; i < 4; i++)
      motors[i].writeMicroseconds((int)(distrs[i] * 290 + 710));

    //    motors[FRONT_LEFT].writeMicroseconds(values[THROTTLE]);

  }


}
