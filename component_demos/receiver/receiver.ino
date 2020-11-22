#include <RF24.h>

int xVal, yVal;//, buttonVal;

int size_example[2];
const size_t sVals = sizeof(size_example);

RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001";


void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  Serial.println("</setup>");
}

void loop() {
  int values[2];

  if (radio.available()) {
    radio.read(&values, sVals);

    if (Serial) {
      Serial.print(values[0]);
      Serial.print(",");
      Serial.print(values[1]);
      Serial.println(";");
    }
  }
}
