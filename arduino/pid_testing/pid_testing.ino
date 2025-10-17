#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <WiFiS3.h>
#include <WiFiUdp.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include "arduino_secrets.h"
#include "direction_functions.h"

WiFiUDP udp;
int port = 8182;
char packet[16];
int dataLength;

Adafruit_MotorShield motor_shield = Adafruit_MotorShield();

Adafruit_DCMotor *left_motor = motor_shield.getMotor(LEFT_WHEEL);
byte left_dir = FORWARD;   // default
double left_sensor_read = 0;
double left_threshold = 1024;
bool left_tag = false;
bool prev_left_tag = false;

Adafruit_DCMotor *right_motor = motor_shield.getMotor(RIGHT_WHEEL);
byte right_dir = FORWARD;  // default
double right_sensor_read = 0;
double right_threshold = 1024;
bool right_tag = false;
bool prev_right_tag = false;

bool left_on = false;

DIRECTION last_direction;

int added_speed = 0;


void auto_calibrate() {
  String outputRead;
  Serial.print("Is the Left Sensor on the Line?: ");
  while (Serial.available() <= 0);
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
  while (Serial.available() <= 0);
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
  left_tag = abs(left_sensor_read - left_threshold) > 450 && left_sensor_read < 850;
  right_tag = abs(right_sensor_read - right_threshold) < 450 && right_sensor_read < 850;
  if (abs(left_sensor_read - left_threshold) < 100 && abs(right_sensor_read - right_threshold) < 100) { // both off line
    left_on = false;
    Serial.println("DRIVE!");
    left_dir = FORWARD;
    right_dir = FORWARD;
    left_motor->run(left_dir);
    right_motor->run(right_dir);
    last_direction = DRIVE;
    delay(10);
  } else if ((abs(left_sensor_read - left_threshold) > 450 && left_sensor_read < 850) && abs(right_sensor_read - right_threshold) < 100) { // left on line, turn left
    left_on = false;
    Serial.println("LEFT!");
    left_dir = BACKWARD;
    right_dir = FORWARD;
    left_motor->run(left_dir);
    right_motor->run(right_dir);
    last_direction = LEFT;
    delay(10);
  } else if (abs(left_sensor_read - left_threshold) < 100 && (abs(right_sensor_read - right_threshold) < 450 && right_sensor_read < 850)) { // right on line, turn right
    left_on = false;
    Serial.println("RIGHT!");
    left_dir = FORWARD;
    right_dir = BACKWARD;
    left_motor->run(left_dir);
    right_motor->run(right_dir);
    last_direction = RIGHT;
    delay(10);
  } else { // both on line, default to turning right
    left_on = true;
    Serial.print("Both on....");
    if (prev_left_tag && !prev_right_tag) {
      Serial.println("LEFT");
      left_dir = BACKWARD;
      right_dir = FORWARD;
      left_motor->run(left_dir);
      right_motor->run(right_dir);
      last_direction = LEFT;
      delay(10);
    } else if (prev_right_tag && !prev_left_tag) {
      Serial.println("RIGHT!");
      left_dir = FORWARD;
      right_dir = BACKWARD;
      left_motor->run(left_dir);
      right_motor->run(right_dir);
      last_direction = RIGHT;
      delay(10);
    }
    // switch (last_direction) {
    //   case DRIVE:
    //     left_dir = FORWARD;
    //     right_dir = BACKWARD;
    //     left_motor->run(left_dir);
    //     right_motor->run(right_dir);
    //     delay(10);
    //     break;
      // case LEFT:
        // left_dir = FORWARD;
        // right_dir = BACKWARD;
        // left_motor->run(left_dir);
        // right_motor->run(right_dir);
        // delay(10);
        // break;
    //   case RIGHT:
    //     left_dir = BACKWARD;
    //     right_dir = FORWARD;
    //     left_motor->run(left_dir);
    //     right_motor->run(right_dir);
    //     delay(10);
    //     break;
    // }
  }

  if (udp.parsePacket()) {
    int data = udp.available();
    udp.read(packet, 255);
    Serial.println(packet);

    if (String(packet).substring(0, String(packet).length() - 1) == ("up")) {
      added_speed += 10;
    } else if (String(packet).substring(0, String(packet).length() - 1) == ("down")) {
      added_speed -= 10;
    }
    udp.beginPacket(udp.remoteIP(), udp.remotePort());
      udp.println("send");
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

  left_motor->setSpeed(BASE_LEFT_SPEED + added_speed);
  right_motor->setSpeed(BASE_RIGHT_SPEED + added_speed);
  prev_left_tag = (left_on) ? prev_left_tag : left_tag;
  prev_right_tag = (left_on) ? prev_right_tag : right_tag;
}
