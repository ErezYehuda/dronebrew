const int elevator_in  = A2; // X Input Pin of Analog 2
const int aileron_in   = A3; // Y Input Pin of Analog 3
const float two_pi = 2 * M_PI;


float values[4];

void setup() {
  Serial.begin(9600);
}

void loop() {
  float x = (analogRead(aileron_in) - 512) / 512.0;
  float y = (analogRead(elevator_in) - 512) / 512.0;

  float radian = atan2(y, x);
  if (radian < 0)
    radian = two_pi + radian;

  float bound_x = cos(radian);
  float bound_y = sin(radian);

  if (x / bound_x > 1 || y / bound_y > 1) {
    Serial.print("OOB: " + (String)x + "," + (String) y + " outside +-" + (String)bound_x + "," + (String)bound_y + " (Radians " + (String)radian + ")");
    float snapradian = atan2(bound_y, bound_x);
    if (snapradian < 0)
      snapradian = two_pi + snapradian;
    Serial.println("---> " + (String)bound_x + "," + (String)bound_y + " (Radians " + (String)snapradian + ")");
  }
  else {
    Serial.println("     " + (String)x + "," + (String) y + "  inside +-" + (String)bound_x + "," + (String)bound_y + " (Radians " + (String)radian + ")");
  }


}
