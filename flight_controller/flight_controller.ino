//#include <PID_v1.h>
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
//#define OUTPUT_READABLE_ACCELGYRO

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
const int X_ACCEL_OFFSET = -1185;
const int Y_ACCEL_OFFSET = 301;
const int Z_ACCEL_OFFSET = 1083;
const int X_GYRO_OFFSET = 92;
const int Y_GYRO_OFFSET = -137;
const int Z_GYRO_OFFSET = 28;

// Gyro Range: [-32768, +32767]
const int GYRO_MIN = -32768;
const int GYRO_MAX = 32767;
const int GYRO_MOTOR_MIN = -16384;
const int GYRO_MOTOR_MAX = 16384;
const int GYRO_GOAL_MIN = -8192;
const int GYRO_GOAL_MAX = 8192;

// END OF GYRO/ACCEL FEATURES


// MOTOR FEATURES

enum Channels {THROTTLE, AILERON, ELEVATOR, RUDDER};
// Throttle->All
// Aileron=Roll->Left vs. Right
// Elevator=Pitch->Front vs. Back
// Rudder=Yaw->CW vs. CCW

enum Motor { FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT};
const int motor_ports[] = {5, 6, 9, 10};
Servo motors [4];

const int ESC_MAX = 2200;
const int ESC_MIN = 850; // Technically, 700 might be acceptible, but it seems to be unreliable for the ESCs to read
const int ESC_RANGE = ESC_MAX - ESC_MIN;
const float CUBERT_ESC_RANGE = cbrt(ESC_RANGE);

// END OF MOTOR FEATURES

// PID FEATURES

struct PID {
  float x_result;
  float y_result;
};

float COEFFICIENT_PROPORTIONAL = 0.05;
float COEFFICIENT_INTEGRAL = 0.25;
// float COEFFICIENT_DERIVATIVE = 0.25;
float PID_MAX_OUTPUT = ESC_RANGE;
float pid_previous_time;
float x_gyro_integral_error;
float y_gyro_integral_error;
float x_gyro_previous_value;
float y_gyro_previous_value;

// END OF PID FEATURES

void print_dists();
void print_mico_dists();
void print_channels();
void configure_escs(bool verbose);


// Some cube-root constants that allow the multiplied throttle distributions to never stray outside a ratio of 75:25
const float CUBERT_UPPER = cbrt(0.75);
const float CUBERT_LOWER = cbrt(0.25);
const float CUBERT_SUM = CUBERT_UPPER + CUBERT_LOWER;
const float INTER_CUBERT = CUBERT_UPPER - CUBERT_LOWER;
const float CUBERT_SCALE_FACTOR = 1 / (INTER_CUBERT * 1023);

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

  accelgyro.setXAccelOffset(X_ACCEL_OFFSET);
  accelgyro.setYAccelOffset(Y_ACCEL_OFFSET);
  accelgyro.setZAccelOffset(Z_ACCEL_OFFSET);
  accelgyro.setXGyroOffset(X_GYRO_OFFSET);
  accelgyro.setYGyroOffset(Y_GYRO_OFFSET);
  accelgyro.setZGyroOffset(Z_GYRO_OFFSET);

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

    for (int i = 0; i < 4; i++) {
      distrs[i] = throttle;
    }

    int controller_aileron  = map(values[AILERON], 0, 1023, GYRO_GOAL_MIN, GYRO_GOAL_MAX);
    int controller_elevator = map(values[ELEVATOR], 0, 1023, GYRO_GOAL_MIN, GYRO_GOAL_MAX);
    int controller_rudder = map(values[RUDDER], 0, 1023, 0, ESC_RANGE);

    Serial.print(values[AILERON]);
    Serial.print(",");
    Serial.print(values[ELEVATOR]);
    Serial.println();

    //calc_pid(float x_target, float x_current, float y_target, float y_current)
    PID new_pid = calc_pid(controller_aileron, gx, controller_elevator, gy);

    //    Serial.print(new_pid.x_result);
    //    Serial.print(",");
    //    Serial.println(new_pid.y_result);

    float new_aileron = map(new_pid.x_result, GYRO_MOTOR_MIN, GYRO_MOTOR_MAX, 0, ESC_RANGE);
    float cubert_aileron = cbrt(new_aileron);
    float cubert_inv_aileron = cbrt(ESC_RANGE - new_aileron);

    float new_elevator = map(new_pid.y_result, GYRO_MOTOR_MIN, GYRO_MOTOR_MAX, 0, ESC_RANGE);
    float cubert_elevator = cbrt(new_elevator);
    float cubert_inv_elevator = cbrt(ESC_RANGE - new_aileron);

    float cubert_rudder = cbrt(controller_rudder);
    float cubert_inv_rudder = cbrt(ESC_RANGE - controller_rudder);

    distrs[FRONT_LEFT]  *= cubert_aileron;
    distrs[BACK_LEFT]   *= cubert_aileron;
    distrs[FRONT_RIGHT] *= cubert_inv_aileron;
    distrs[BACK_RIGHT]  *= cubert_inv_aileron;

    distrs[BACK_LEFT]   *= cubert_elevator;
    distrs[BACK_RIGHT]  *= cubert_elevator;
    distrs[FRONT_LEFT]  *= cubert_inv_elevator;
    distrs[FRONT_RIGHT] *= cubert_inv_elevator;

    distrs[BACK_LEFT]   *= cubert_rudder;
    distrs[FRONT_RIGHT] *= cubert_rudder;
    distrs[FRONT_LEFT]  *= cubert_inv_rudder;
    distrs[BACK_RIGHT]  *= cubert_inv_rudder;

    for (int i = 0; i < 4; i++) {
      distrs[i] += ESC_MIN;
    }

    for (int i = 0; i < 4; i++) {
      int microseconds = (int)(distrs[i]);
      motors[i].writeMicroseconds(microseconds);
      //      Serial.print(microseconds);
      //      //      //      Serial.print(distrs[i]);
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
    Serial.print((int)(distrs[i] * ESC_RANGE + ESC_MIN));
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

    throttle = ESC_MAX;
    //throttle = map(values[THROTTLE], 0, 1023, ESC_MIN, ESC_MAX);
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

    //throttle = map(values[THROTTLE], 0, 1023, ESC_MIN, ESC_MAX);
    throttle = ESC_MIN;

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


/*
  float COEFFICIENT_PROPORTIONAL;
  float COEFFICIENT_INTEGRAL;
  // float COEFFICIENT_DERIVATIVE;
  float PID_MAX_OUTPUT;
  float pid_previous_time;
  float x_gyro_integral_error;
  float y_gyro_integral_error;
  float x_gyro_previous_value;
  float y_gyro_previous_value;
*/
PID calc_pid(float x_target, float x_current, float y_target, float y_current) {
  // Calculate the time since function was last called
  float now = millis();
  float dT = now - pid_previous_time;
  pid_previous_time = now;

//  Serial.print(dT);
//  Serial.print(":");
//  Serial.print(x_target);
//  Serial.print(":");
//  Serial.print(y_target);
//  Serial.println();
//
//  Serial.print(dT);
//  Serial.print(",");

  // Calculate error between target and current values
  float x_error = x_target - x_current;
  float y_error = y_target - y_current;

  // Calculate the integral term
  x_gyro_integral_error += x_error * dT;
  y_gyro_integral_error += y_error * dT;

  // Set old variable to equal new ones
  x_gyro_previous_value = x_current;
  y_gyro_previous_value = y_current;

  // Multiply each term by its constant, and add it all up
  float x_result = (x_error * COEFFICIENT_PROPORTIONAL) ;//+ (x_gyro_integral_error * COEFFICIENT_INTEGRAL);
  float y_result = (y_error * COEFFICIENT_PROPORTIONAL) ;//+ (y_gyro_integral_error * COEFFICIENT_INTEGRAL);
//  Serial.print(x_error);
//    Serial.print(",");
//    Serial.print(y_error);
//  //  Serial.print(x_gyro_integral_error);
//  Serial.println();

  // Limit PID value to maximum values
  if (x_result > GYRO_GOAL_MAX) x_result = GYRO_GOAL_MAX;
  else if (x_result < GYRO_GOAL_MIN) x_result = GYRO_GOAL_MIN;

  if (y_result > GYRO_GOAL_MAX) y_result = GYRO_GOAL_MAX;
  else if (y_result < GYRO_GOAL_MIN) y_result = -PID_MAX_OUTPUT;

  PID pid_result = {x_result, y_result};

  return pid_result;
}
