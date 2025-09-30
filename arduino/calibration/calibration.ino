#define LEFT_SENSOR A0
#define RIGHT_SENSOR A1

#define byte uint8_t

void setup() {
  pinMode(LEFT_SENSOR, INPUT);
  pinMode(RIGHT_SENSOR, INPUT);
}

void loop() {
  int left_sensor_read = analogRead(LEFT_SENSOR);
  int right_sensor_read = analogRead(RIGHT_SENSOR);
}
