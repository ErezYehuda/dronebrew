
#include <RF24.h>
#include <Servo.h>
#include <Wire.h>

void print_dists();
void print_mico_dists();
void print_channels();
void configure_escs(bool verbose = false);
int get_i2c_address();
void calculate_IMU_error();
void configure_gyroscope(bool should_calculate_imu_error, bool verbose = false);
void read_gyro(bool verbose = false);

char f2sb[10]; // Float->String buffer

// !!Motor section!!
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


// !!Radio section!!
const int size_example[4];
const size_t vals_size = sizeof(size_example);

int values[4];
float distrs[4]; // Distributions of throttle

RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001";

// !!Gyroscope section!!
int MPU; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX = -35.26, AccErrorY = 35.26, GyroErrorX = -0.01, GyroErrorY = -0.01, GyroErrorZ = -0.01;
float elapsedTime, currentTime, previousTime;

// !!Some config stuff!!
const bool verbose_setup = true, verbose_loop = true;


void setup() {
  bool config_esc = false, find_gyro_error = true;
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
      // If left joystick is roughly at top
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


  for (int i = 0; i < 10; i++)
    if (radio.available()) {
      radio.read(&values, vals_size);
      if (values[ELEVATOR] <= 10) {
        find_gyro_error = false;
        break;
      }
    }
    else i -= 1;

  // Just set it to true if there's no input
  if (values[0] + values[1] + values[2] + values[3] == 0) {
    find_gyro_error = true;
  }

  configure_gyroscope(find_gyro_error);
}

void loop() {
  if (radio.available()) {
    radio.read(&values, vals_size);

    // Since my joystick defaults to the center, I'm setting it to treat mid as the bottom
    float throttle = (values[THROTTLE] - 512) / 512.0;
    if (throttle < 0) throttle = 0;
    // Setting it up so that the product of aileron, elevator, and rudder for any given motor will be [0.25,0.75]


    for (int i = 0; i < 4; i++) {
      distrs[i] = throttle;
    }

    for (int i = 0; i < 4; i++) {
      motors[i].writeMicroseconds((int)(distrs[i] * esc_range + esc_bott));
      //      Serial.print((int)(distrs[i] * esc_range + esc_bott));
      //      Serial.print(",");
    }
    //    Serial.println();
  }

  read_gyro(true);
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

void configure_gyroscope(bool should_calculate_imu_error, bool verbose = false) {
  MPU = get_i2c_address();
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission

  if (should_calculate_imu_error)
    calculate_IMU_error();
}

void calculate_IMU_error() {
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times

  AccErrorX = 0, AccErrorY = 0, GyroErrorX = 0, GyroErrorY = 0, GyroErrorZ = 0;

  for (byte i = 0; i < 200; i++) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;

    Serial.print(AccX);
    Serial.print("~");
    Serial.print(AccY);
    Serial.print("~");
    Serial.println(AccZ);

    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
  }
  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  // Read gyro values 200 times
  for (byte i = 0; i < 200; i++) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
  }
  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
  // Print the error values on the Serial Monitor
  Serial.print("AccErrorX: ");
  Serial.println(dtostrf(AccErrorX, 7, 4, f2sb));

  Serial.print("AccErrorY: ");
  Serial.println(dtostrf(AccErrorY, 7, 4, f2sb));

  Serial.print("GyroErrorX: ");
  Serial.println(dtostrf(GyroErrorX, 7, 4, f2sb));

  Serial.print("GyroErrorY: ");
  Serial.println(dtostrf(GyroErrorY, 7, 4, f2sb));

  Serial.print("GyroErrorZ: ");
  Serial.println(dtostrf(GyroErrorZ, 7, 4, f2sb));
}

int get_i2c_address() {
  Serial.println("Finding i2c address.");
  byte error, address;
  for (address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");

      return address;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
      return 0;
    }
    //    else {
    //          Serial.print("Looking for I2C device at address 0x");
    //          if (address < 16)
    //            Serial.print("0");
    //          Serial.print(address, HEX);
    //          Serial.println("  !");
    //        }
  }

  Serial.println("No I2C devices found\n");
  return 0;
}

void read_gyro(bool verbose = false) {
  // === Read acceleromter data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorX; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI)  - AccErrorY; // AccErrorY ~(-1.58)
  // === Read gyroscope data === //
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0 - GyroErrorX; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0 - GyroErrorY;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0 - GyroErrorZ;

  //  if (verbose) {
  //    Serial.print("~~");
  //    Serial.print(dtostrf(GyroX, 7, 4, f2sb));
  //    Serial.print("/");
  //    Serial.print(dtostrf(GyroY, 7, 4, f2sb));
  //    Serial.print("/");
  //    Serial.println(dtostrf(GyroZ, 7, 4, f2sb));
  //  }

  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  yaw =  yaw + GyroZ * elapsedTime;
  // Complementary filter - combine acceleromter and gyro angle values
  roll =  gyroAngleX + accAngleX;
  pitch = gyroAngleY +  accAngleY;

  //   Print the values on the serial monitor
  if (verbose) {
    Serial.print("~~");
    Serial.print(dtostrf(roll, 7, 4, f2sb));
    Serial.print("/");
    Serial.print(dtostrf(pitch, 7, 4, f2sb));
    Serial.print("/");
    Serial.println(dtostrf(yaw, 7, 4, f2sb));
  }
}
