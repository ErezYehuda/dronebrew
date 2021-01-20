const int elevator_in  = A2; // X Input Pin of Analog 2
const int aileron_in   = A3; // Y Input Pin of Analog 3

float values[4];

void setup() {
  Serial.begin(9600);
}

void loop() {
  float x = (analogRead(aileron_in) - 512) / 512.0;
  float y = (analogRead(elevator_in) - 512) / 512.0;

  float z = sqrt(x * x + y * y);

  float bound_x = x / z;
  float bound_y = y / z;

  if (z > 1) {
    Serial.println("OOB: " + (String)x + "," + (String) y + " outside +-" + (String)bound_x + "," + (String)bound_y);
  }
  else {
    Serial.println("     " + (String)x + "," + (String) y + "  inside +-" + (String)bound_x + "," + (String)bound_y);
  }
}
