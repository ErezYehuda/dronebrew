
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

const int esc_top = 2200;
const int esc_bott = 850; // Technically, 700 might be acceptible, but it seems to be unreliable for the ESCs to read
const int esc_range = esc_top - esc_bott;

// Some cube-root constants that allow the multiplied throttle distributions to never stray outside a ratio of 75:25
const float cubert75 = cbrt(0.75);
const float cubert25 = cbrt(0.25);
const float cubert_sum = cubert75 + cubert25;
const float inter_cubert = cubert75 - cubert25;
const float cubert_coefficient = inter_cubert / 1023;

const int size_example[4];
const size_t vals_size = sizeof(size_example);

int values[4];
float distrs[4]; // Distributions of throttle

RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001";

const bool verbose_setup = 1, verbose_loop = 1;
bool config_esc = false;

void setup() {
  if (verbose_setup || verbose_loop)
    Serial.begin(9600);

  for (int i = 0; i < 4; i++)
    motors[i].attach(motor_ports[i]);

  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  Serial.println("</setup>");

  for (int i = 0; i < 10; i++)
    if (radio.available()) {
      radio.read(&values, vals_size);
      if (values[THROTTLE] >= 1020) {
        config_esc = true;
        break;
      }
    }
    else i -= 1;

  if (verbose_setup) {
    Serial.print("Initial Throttle: ");
    Serial.println(values[THROTTLE]);
  }

  if (config_esc) {
    Serial.println("Configuring ESCs");
    // If it's within 3 of the top Y-position of the left joystick...
    // Write the undistributed throttle to the motors until it gets below 4
    do {
      if (radio.available()) {
        radio.read(&values, vals_size);
        if (verbose_setup)
          Serial.println(values[THROTTLE]);
      }
      for (int i = 0; i < 4; i++)
        motors[i].writeMicroseconds(map(values[THROTTLE], 0, 1023, esc_bott, esc_top));
    } while (values[THROTTLE] > 10);

    // Wait for them to bring the throttle stick up again
    do {
      if (radio.available())
        radio.read(&values, vals_size);
      for (int i = 0; i < 4; i++)
        motors[i].writeMicroseconds(map(values[THROTTLE], 0, 1023, esc_bott, esc_top));
      if (verbose_setup)
        Serial.println(values[THROTTLE]);
    } while (values[THROTTLE] < 1020);
    Serial.println("Ending ESC configuration");
  } else {
    Serial.println("Skipping ESC configuration");
  }
}

void loop() {
  if (radio.available()) {
    radio.read(&values, vals_size);

    //        if (verbose_loop) {
    //          print_channels();
    //        }

    // Throttle->All
    // Aileron=Roll->Left vs. Right
    // Elevator=Pitch->Front vs. Back
    // Rudder=Yaw->CW vs. CCW

    // Since my joystick defaults to the center, I'm setting it to treat mid as the bottom
    float throttle = (values[THROTTLE] - 512) * 2  / 1023.0;
    if (throttle < 0) throttle = 0;
    // Setting it up so that the product of aileron, elevator, and rudder for any given motor will be [0.25,0.75]
    float aileron  = values[AILERON]  / 1023.0 * inter_cubert + cubert25;
    float elevator = values[ELEVATOR] / 1023.0 * inter_cubert + cubert25;
    float rudder   = values[RUDDER]   / 1023.0 * inter_cubert + cubert25;

    for (int i = 0; i < 4; i++) {
      distrs[i] = throttle;
    }

    //    print_dists();

    distrs[FRONT_RIGHT] *= aileron;
    distrs[BACK_RIGHT]  *= aileron;
    distrs[FRONT_LEFT]  *= cubert_sum - aileron;
    distrs[BACK_LEFT]   *= cubert_sum - aileron;

    //  Serial.println(aileron);
    //  Serial.print("cubert_sum - ail:");
    //  Serial.print(cubert_sum);
    //  Serial.print(" - ");
    //  Serial.print(aileron);
    //  Serial.print(" = ");
    //  Serial.println(cubert_sum-aileron);

    //    print_dists();

    distrs[BACK_LEFT]   *= elevator;
    distrs[BACK_RIGHT]  *= elevator;
    distrs[FRONT_LEFT]  *= cubert_sum - elevator;
    distrs[FRONT_RIGHT] *= cubert_sum - elevator;

    distrs[BACK_LEFT]   *= rudder;
    distrs[FRONT_RIGHT] *= rudder;
    distrs[FRONT_LEFT]  *= cubert_sum - rudder;
    distrs[BACK_RIGHT]  *= cubert_sum - rudder;


    // Some reference values (at full throttle):
    // Max microsecs: 1677.5
    // Mid microsecs: 1290.5 (this is the value of esc_range)
    // Min microsecs: 1032.5

    for (int i = 0; i < 4; i++) {
      //      Serial.print((int)(distrs[i] * esc_range + esc_bott));
      //      //      Serial.print((distrs[i]) );
      //      Serial.print(",");
      motors[i].writeMicroseconds((int)(distrs[i] * esc_range + esc_bott));
    }
    //    Serial.println();
    //    Serial.println((distrs[0] + distrs[1] + distrs[2] + distrs[3]));

    if (verbose_loop) {
      //      print_dists();
      print_mico_dists();
    }

  }
}

void print_dists() {
  for (int i = 0; i < 4; i++) {
    Serial.print(distrs[i]);
    Serial.print(',');
  }
  Serial.println();
}

void print_mico_dists() {
  for (int i = 0; i < 4; i++) {
    Serial.print((int)(distrs[i] * esc_range + esc_bott));
    Serial.print(',');
  }
  Serial.println();
}

void print_channels() {
  Serial.print(values[THROTTLE]);
  Serial.print(',');
  Serial.print(values[AILERON]);
  Serial.print(',');
  Serial.print(values[ELEVATOR]);
  Serial.print(',');
  Serial.println(values[RUDDER]);
}
