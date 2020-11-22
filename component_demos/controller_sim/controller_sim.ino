#include <Servo.h>


enum Motor { FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT};
enum Channels {THROTTLE, AILERON, ELEVATOR, RUDDER};
int motor_ports[] = {9, 10, 11, 12};


const int top = 2000;
const int bott = 710; // Technically, 700 might be acceptible, but it seems to be an unreliable for the ESCs to read

int values[4];
Servo motors [4];


String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
char *p, *i;
char* c = new char[200 + 1]; // match 200 characters reserved for inputString later
char* errpt;
uint8_t ppm_cnt;

void setup() {

  inputString.reserve(200);
  Serial.begin(9600);

  Serial.println("Ready to accept serial input ~~~");

  //  String str;
  while (inputString == "") {
    if (Serial.available())
      inputString = Serial.readString();
  }

  Serial.println("Connection established ~~~");

  motors[FRONT_LEFT].attach(motor_ports[FRONT_LEFT]);
  Serial.readString();

}

void loop() {
  //  Serial.println(values[THROTTLE]);
  if (stringComplete) {
    //    Serial.println(inputString);
    //  Serial.println(values[THROTTLE]);
  }
  motors[FRONT_LEFT].writeMicroseconds(values[THROTTLE]);

  if (Serial.available()) {
    // Process string into tokens and assign values to ppm
    // The Arduino will also echo the command values that it assigned
    // to ppm
    if (stringComplete) {
      //Serial.println(inputString);
      // process string

      strcpy(c, inputString.c_str());
      p = strtok_r(c, ",", &i); // returns substring up to first "," delimiter
      ppm_cnt = 0;
      while (p != 0) {
        int val = strtol(p, &errpt, 10);
        if (!*errpt && val) {
          //        Serial.println(val);
          //        Serial.println(ppm_cnt+":"+val);
          //          Serial.print(ppm_cnt);
          //          Serial.print(":");
          //          Serial.println(val);
          values[ppm_cnt] = val;
          //          Serial.print("values[THROTTLE]:");
          Serial.println(values[THROTTLE]);
        }
        p = strtok_r(NULL, ",", &i);
        ppm_cnt += 1;
      }



      // clear the string:
      inputString = "";
      stringComplete = false;
    }

    // Read the string from the serial buffer
    while (Serial.available()) {
      // get the new byte:
      char inChar = (char)Serial.read();
      // if the incoming character is a newline, set a flag
      // so the main loop can do something about it:
      if (inChar == '\n') {
        stringComplete = true;
      }
      else {
        // add it to the inputString:
        //        Serial.println(inChar);
        inputString += inChar;
      }

    }
  }
}
