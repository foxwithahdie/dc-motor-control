#define LEFT_SENSOR A0
#define RIGHT_SENSOR A1

#define ANALOG_MAX 1023

#define byte uint8_t

int ideal_analog_read = 200;

void setup() {
  Serial.begin(9600);
  pinMode(LEFT_SENSOR, INPUT);
  pinMode(RIGHT_SENSOR, INPUT);
}

void loop() {
  int left_sensor_read = analogRead(LEFT_SENSOR);
  int right_sensor_read = analogRead(RIGHT_SENSOR);

  Serial.print("left: ");
  Serial.println(left_sensor_read);
  Serial.print("right: ");
  Serial.println(right_sensor_read);
  Serial.println("--------- NEW READ --------");

  delay(350);

}
