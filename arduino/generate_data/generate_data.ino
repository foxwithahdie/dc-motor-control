#include "arduino_secrets.h"
#include "direction_functions.h"
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Adafruit_MotorShield.h>
#include <WiFiS3.h>
#include <WiFiUdp.h>
#include <Wire.h>

WiFiUDP udp;
int port = 8182;
char packet[16];
int dataLength;

int left_speed_data[8];
int right_speed_data[8];
int left_sensor_data[8];
int right_sensor_data[8];

byte counter = 8;

Adafruit_MotorShield motor_shield = Adafruit_MotorShield();

Adafruit_DCMotor *left_motor = motor_shield.getMotor(LEFT_WHEEL);
byte left_dir = FORWARD; // default
int left_sensor_read = 0;
int left_threshold = 1024;
// bool left_tag = false;
// bool prev_left_tag = false;

Adafruit_DCMotor *right_motor = motor_shield.getMotor(RIGHT_WHEEL);
byte right_dir = FORWARD; // default
int right_sensor_read = 0;
int right_threshold = 1024;
// bool right_tag = false;
// bool prev_right_tag = false;

DIRECTION last_direction;

int added_speed = 0;

void auto_calibrate() {
  String outputRead;
  Serial.print("Is the Left Sensor on the Line?: ");
  while (Serial.available() <= 0)
    ;
  outputRead = Serial.readString();
  Serial.print(outputRead);
  if (outputRead == ("done\n")) {
    Serial.println("Reading...");
    int left_sensor_read_sum = 0;
    for (int i = 0; i < 100; i++) {
      int left_sensor = analogRead(LEFT_SENSOR);
      left_sensor_read_sum += left_sensor;
    }
    left_threshold = left_sensor_read_sum / 100;
    Serial.println("Left Sensor Done!");
  }
  Serial.flush();
  Serial.print("Is the Right Sensor on the Line?: ");
  while (Serial.available() <= 0)
    ;
  outputRead = Serial.readString();
  if (outputRead == ("done\n")) {
    Serial.println("Reading...");
    int right_sensor_read_sum = 0;
    for (int i = 0; i < 100; i++) {
      int right_sensor = analogRead(RIGHT_SENSOR);
      right_sensor_read_sum += right_sensor;
    }
    right_threshold = right_sensor_read_sum / 100;
    Serial.println("Right Sensor Done!");
  }
  Serial.println("Done!");
}

void setup() {
  motor_shield.begin();
  Serial.begin(9600);
  delay(1000);

  WiFi.begin(SSID, PWD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.print("Connecting...");
  }
  Serial.print("IP = ");
  Serial.println(WiFi.localIP());

  udp.begin(8182);

  pinMode(LEFT_SENSOR, INPUT);
  pinMode(RIGHT_SENSOR, INPUT);
  auto_calibrate();
}

void loop() {
  left_sensor_read = analogRead(LEFT_SENSOR);
  right_sensor_read = analogRead(RIGHT_SENSOR);

  if (abs(left_sensor_read - left_threshold) < 100 &&
      abs(right_sensor_read - right_threshold) < 100) { // both off line
    left_dir = FORWARD;
    right_dir = FORWARD;
    left_motor->run(left_dir);
    right_motor->run(right_dir);
    last_direction = DRIVE;
    delay(10);
  } else if (abs(left_sensor_read - left_threshold) > 450 &&
             abs(right_sensor_read - right_threshold) <
                 100) { // left on line, turn left
    left_dir = BACKWARD;
    right_dir = FORWARD;
    left_motor->run(left_dir);
    right_motor->run(right_dir);
    last_direction = LEFT;
    delay(10);
  } else if (abs(left_sensor_read - left_threshold) < 100 &&
             abs(right_sensor_read - right_threshold) <
                 450) { // right on line, turn right
    left_dir = FORWARD;
    right_dir = BACKWARD;
    left_motor->run(left_dir);
    right_motor->run(right_dir);
    last_direction = RIGHT;
    delay(10);
  } else { // both on line, default to turning right
    switch (last_direction) {
    case DRIVE:
      left_dir = FORWARD;
      right_dir = BACKWARD;
      left_motor->run(left_dir);
      right_motor->run(right_dir);
      delay(10);
      break;
    case LEFT:
      left_dir = FORWARD;
      right_dir = BACKWARD;
      left_motor->run(left_dir);
      right_motor->run(right_dir);
      delay(10);
      break;
    case RIGHT:
      left_dir = BACKWARD;
      right_dir = FORWARD;
      left_motor->run(left_dir);
      right_motor->run(right_dir);
      delay(10);
      break;
    }
  }

  if (udp.parsePacket()) {
    int data = udp.available();
    udp.read(packet, 255);
    Serial.println(packet);

    if (String(packet).substring(0, String(packet).length() - 1) == ("up")) {
      added_speed += 10;
    } else if (String(packet).substring(0, String(packet).length() - 1) ==
               ("down")) {
      added_speed -= 10;
    }

    udp.beginPacket(udp.remoteIP(), udp.remotePort());
    udp.print("");
    if (counter == 7) {
      udp.print("left_speed ");
      udp.print("<");
      for (int i = 0; i < 8; i++) {
        udp.print(left_speed_data[i]);
        if (i != 7)
          udp.print(",");
      }
      udp.print(">");
      udp.println();
      udp.print("right_speed ");
      udp.print("<");
      for (int i = 0; i < 8; i++) {
        udp.print(right_speed_data[i]);
        if (i != 7)
          udp.print(",");
      }
      udp.print(">");
      udp.println();
      udp.print("left_sensor ");
      udp.print("<");
      for (int i = 0; i < 8; i++) {
        udp.print(left_sensor_data[i]);
        if (i != 7)
          udp.print(",");
      }
      udp.print(">");
      udp.println();
      udp.print("right_sensor ");
      udp.print("<");
      for (int i = 0; i < 8; i++) {
        udp.print(right_sensor_data[i]);
        if (i != 7)
          udp.print(",");
      }
      udp.print(">");
      udp.println();
    }
    udp.endPacket();

    for (int i = 0; i < 16; i++) {
      packet[i] = 0;
    }
  }

  Serial.println(" --------------- CURRENT READ --------------- ");
  Serial.print("Added Speed: ");
  Serial.println(added_speed);
  Serial.print("Left Sensor Read: ");
  Serial.println(left_sensor_read);
  Serial.print("Right Sensor Read: ");
  Serial.println(right_sensor_read);

  left_speed_data[counter] =
      ((left_dir == BACKWARD) ? -1 : 1) * (BASE_LEFT_SPEED + added_speed);
  right_speed_data[counter] =
      ((right_dir == BACKWARD) ? -1 : 1) * (BASE_RIGHT_SPEED + added_speed);
  left_sensor_data[counter] = left_sensor_read;
  left_sensor_data[counter] = right_sensor_read;

  left_motor->setSpeed(BASE_LEFT_SPEED + added_speed);
  right_motor->setSpeed(BASE_RIGHT_SPEED + added_speed);
}
