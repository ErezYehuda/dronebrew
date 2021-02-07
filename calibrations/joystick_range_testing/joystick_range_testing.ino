const int elevator_in  = A2; // X Input Pin of Analog 2
const int aileron_in   = A3; // Y Input Pin of Analog 3

void setup() {
  Serial.begin(9600);
}

void loop() {
  float x = (analogRead(aileron_in) - 512) / 512.0;
  float y = (analogRead(elevator_in) - 512) / 512.0;
  

  Serial.print(x);
  Serial.print(",");
  Serial.println(y);
}
