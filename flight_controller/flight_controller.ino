
#include <RF24.h>
#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050.h"

// GYRO/ACCEL FEATURES

// Mostly borrowed from I2Cdev MPU6050_raw
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

int16_t ax, ay, az;
int16_t gx, gy, gz;

// uncomment "OUTPUT_READABLE_ACCELGYRO" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
#define OUTPUT_READABLE_ACCELGYRO

// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
//#define OUTPUT_BINARY_ACCELGYRO

/*
   Offsets calculated by https://www.i2cdevlib.com/forums/applications/core/interface/file/attachment.php?id=27
   Linked to from https://www.i2cdevlib.com/forums/topic/96-arduino-sketch-to-automatically-calculate-mpu6050-offsets/
   Sensor readings with offsets: -2  -10 16386 -2  -1  0
   Your offsets: -1185 301 1083  92  -137  28

   Data is printed as: acelX acelY acelZ giroX giroY giroZ
   Check that your sensor readings are close to 0 0 16384 0 0 0
*/
const int x_accel_offset = -1185;
const int y_accel_offset = 301;
const int z_accel_offset = 1083;
const int x_gyro_offset = 92;
const int y_gyro_offset = -137;
const int z_gyro_offset = 28;

// Gyro Range: [-32768, +32767]
const int gyro_min = -32768;
const int gyro_max = 32767;
const int gyro_goal_min = -8192;
const int gyro_goal_max = 8192;

// END OF GYRO/ACCEL FEATURES

void print_dists();
void print_mico_dists();
void print_channels();
void configure_escs(bool verbose = false);

enum Channels {THROTTLE, AILERON, ELEVATOR, RUDDER};
// Throttle->All
// Aileron=Roll->Left vs. Right
// Elevator=Pitch->Front vs. Back
// Rudder=Yaw->CW vs. CCW

enum Motor { FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT};
const int motor_ports[] = {5, 6, 9, 10};
Servo motors [4];

const int esc_top = 2200;
const int esc_bott = 850; // Technically, 700 might be acceptible, but it seems to be unreliable for the ESCs to read
const int esc_range = esc_top - esc_bott;

// Some cube-root constants that allow the multiplied throttle distributions to never stray outside a ratio of 75:25
const float cubert_upper = cbrt(0.75);
const float cubert_lower = cbrt(0.25);
const float cubert_sum = cubert_upper + cubert_lower;
const float inter_cubert = cubert_upper - cubert_lower;
const float cubert_scale_factor = 1 / (inter_cubert * 1023);

const int size_example[] = {1, 1, 1, 1};
const size_t vals_size = sizeof(size_example);

int values[4];
float distrs[4]; // Distributions of throttle

RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001";

const bool verbose_setup = true, verbose_loop = true;
bool config_esc = false;

void setup() {
  if (verbose_setup || verbose_loop)
    Serial.begin(9600);

  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  accelgyro.setXAccelOffset(x_accel_offset);
  accelgyro.setYAccelOffset(y_accel_offset);
  accelgyro.setZAccelOffset(z_accel_offset);
  accelgyro.setXGyroOffset(x_gyro_offset);
  accelgyro.setYGyroOffset(y_gyro_offset);
  accelgyro.setZGyroOffset(z_gyro_offset);

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
    configure_escs(verbose_setup);
  } else {
    Serial.println("Skipping ESC configuration");
  }
}

void loop() {
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

#ifdef OUTPUT_READABLE_ACCELGYRO
  // display tab-separated accel/gyro x/y/z values
  Serial.print("a/g:\t");
  Serial.print(ax); Serial.print("\t");
  Serial.print(ay); Serial.print("\t");
  Serial.print(az); Serial.print("\t");
  Serial.print(gx); Serial.print("\t");
  Serial.print(gy); Serial.print("\t");
  Serial.println(gz);
#endif

#ifdef OUTPUT_BINARY_ACCELGYRO
  Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF));
  Serial.write((uint8_t)(ay >> 8)); Serial.write((uint8_t)(ay & 0xFF));
  Serial.write((uint8_t)(az >> 8)); Serial.write((uint8_t)(az & 0xFF));
  Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 0xFF));
  Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));
  Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF));
#endif

  if (radio.available()) {
    radio.read(&values, vals_size);

    //    if (verbose_loop) {
    //      print_channels();
    //    }

    // Throttle->All
    // Aileron=Roll->Left vs. Right
    // Elevator=Pitch->Front vs. Back
    // Rudder=Yaw->CW vs. CCW

    // Since my joystick defaults to the center, I'm setting it to treat mid as the bottom
    float throttle = (values[THROTTLE] - 512) / 512.0;
    if (throttle < 0) throttle = 0;
    //    // Setting it up so that the product of aileron, elevator, and rudder for any given motor will be [0.25,0.75]
    //    float aileron  = values[AILERON]  * inter_cubert / 1023.0 + cubert_lower;
    //    float elevator = values[ELEVATOR] * inter_cubert / 1023.0 + cubert_lower;
    //    float rudder   = values[RUDDER]   * inter_cubert / 1023.0 + cubert_lower;
    int aileron  = map(values[AILERON], 0, 1023, gyro_goal_min, gyro_goal_max);
    int elevator = map(values[ELEVATOR], 0, 1023, gyro_goal_min, gyro_goal_max);

    for (int i = 0; i < 4; i++) {
      distrs[i] = throttle;
    }

    distrs[FRONT_RIGHT] *= aileron;
    distrs[BACK_RIGHT]  *= aileron;
    distrs[FRONT_LEFT]  *= cubert_sum - aileron;
    distrs[BACK_LEFT]   *= cubert_sum - aileron;

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
      int microseconds = (int)(distrs[i] * esc_range + esc_bott);
      motors[i].writeMicroseconds(microseconds);
      //      Serial.print(microseconds);
      //      //      Serial.print(distrs[i]);
      //      Serial.print(",");
    }
    Serial.println();
  } else {
    Serial.println("R--");
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

void configure_escs(bool verbose = false) {
  if (verbose)
    Serial.println("Configuring ESCs");
  int throttle;

  // If it's within 3 of the top Y-position of the left joystick...
  // Write the undistributed throttle to the motors until it gets below 4
  do {
    if (radio.available()) {
      radio.read(&values, vals_size);
    }

    throttle = esc_top;
    //throttle = map(values[THROTTLE], 0, 1023, esc_bott, esc_top);
    if (verbose) {
      Serial.print(values[THROTTLE]);
      Serial.print(" -> ");
      Serial.println(throttle);
    }
    for (int i = 0; i < 4; i++)
      motors[i].writeMicroseconds(throttle);
  } while (values[THROTTLE] > 10);

  // Wait for them to bring the throttle stick up again
  do {
    if (radio.available()) {
      radio.read(&values, vals_size);
    }

    //throttle = map(values[THROTTLE], 0, 1023, esc_bott, esc_top);
    throttle = esc_bott;

    if (verbose) {
      Serial.print(values[THROTTLE]);
      Serial.print(" -> ");
      Serial.println(throttle);
    }
    for (int i = 0; i < 4; i++)
      motors[i].writeMicroseconds(throttle);

  } while (values[THROTTLE] < 1020);

  if (verbose)
    Serial.println("Ending ESC configuration");
}
