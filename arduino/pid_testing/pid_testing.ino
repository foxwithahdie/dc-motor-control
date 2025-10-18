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

// Motor Shield class
Adafruit_MotorShield motor_shield = Adafruit_MotorShield();

// Left important parts.
Adafruit_DCMotor *left_motor = motor_shield.getMotor(LEFT_WHEEL); // Left wheel from motor shield
byte left_dir = FORWARD; // Direction of left motor
double left_sensor_read = 0; // Left sensor's read
double left_threshold = 1024; // Left sensor's threshold. It is the calibrated value considered as "outside of the line"
bool left_tag = false; // Tag of whether the left sensor is on the line
bool prev_left_tag = false; // Tag of whether the left sensor was on the line previously

// All same but right
Adafruit_DCMotor *right_motor = motor_shield.getMotor(RIGHT_WHEEL);
byte right_dir = FORWARD;  // default
double right_sensor_read = 0;
double right_threshold = 1024;
bool right_tag = false;
bool prev_right_tag = false;

// Whether the "both on the line" was on before. It should commit to one turn
bool left_on = false;

// The previous direction that the robot has moved
DIRECTION last_direction;

// The added speed, controlled from the computer.
int added_speed = 0;

// A timer to make sure that the "both on" period doesn't happen within a certain time period
int timer = 0;

// The accumulator for fixing deadlock
int accumulator = 0;

/*
Auto-calibrates at the beginning of a session.
*/
void auto_calibrate() {
  String outputRead;
  // Prompts user to check if both sensors are ready for calibrating
  Serial.print("Is the Left Sensor off the line?: ");
  while (Serial.available() <= 0);
  outputRead = Serial.readString();
  Serial.print(outputRead);
  if (outputRead == ("done\n")) {
      Serial.println("Reading...");
      // Loops 100 times and averages the readings it receives
      int left_sensor_read_sum = 0;
      for (int i = 0; i < 100; i++) {
        int left_sensor = analogRead(LEFT_SENSOR);
        left_sensor_read_sum += left_sensor;
      }
      left_threshold = left_sensor_read_sum / 100;
      Serial.println("Left Sensor Done!");
  }
  Serial.flush();
  Serial.print("Is the Right Sensor off the line?: ");
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
  
  // Initializes WiFi
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
  // Reads data from sensors
  left_sensor_read = analogRead(LEFT_SENSOR);
  right_sensor_read = analogRead(RIGHT_SENSOR);

  // Sets tags at the very beginning
  left_tag = abs(left_sensor_read - left_threshold) > 450;
  right_tag = abs(right_sensor_read - right_threshold) < 450;

  if (abs(left_sensor_read - left_threshold) < 200 && abs(right_sensor_read - right_threshold) < 200) { // both off line
    left_on = false;
    left_dir = FORWARD;
    right_dir = FORWARD;
    left_motor->run(left_dir);
    right_motor->run(right_dir);
    last_direction = DRIVE;
    if (accumulator > 0) accumulator -= 2;
    delay(10);
  } else if ((abs(left_sensor_read - left_threshold) > 450) && abs(right_sensor_read - right_threshold) < 200) { // left on line, turn left
    left_on = false;
    left_dir = BACKWARD;
    right_dir = FORWARD;
    left_motor->run(left_dir);
    right_motor->run(right_dir);
    last_direction = LEFT;
    if (accumulator > 0) accumulator -= 2;
    delay(10);
  } else if (abs(left_sensor_read - left_threshold) < 200 && (abs(right_sensor_read - right_threshold) < 450)) { // right on line, turn right
    left_on = false;
    left_dir = FORWARD;
    right_dir = BACKWARD;
    left_motor->run(left_dir);
    right_motor->run(right_dir);
    last_direction = RIGHT;
    if (accumulator > 0) accumulator -= 2;
    delay(10);
  } else { // both on line, default to turning right
    accumulator++; // Adding accumulator
    if (!left_on) {
      if ((abs(millis() - timer) > DEFAULT_BOTH_ON_TIME)) { // If the time elasped that the last "both on" scenario has happened is greater than 200ms
        timer = millis();
        left_on = true;

        if (prev_left_tag && !prev_right_tag) { // If the left tag was on the line first
          left_dir = BACKWARD;
          right_dir = FORWARD;
          left_motor->run(left_dir);
          right_motor->run(right_dir);
          last_direction = LEFT;
          delay(10);
        } else if (prev_right_tag && !prev_left_tag) { // If the right tag was on the line first
          left_dir = FORWARD;
          right_dir = BACKWARD;
          left_motor->run(left_dir);
          right_motor->run(right_dir);
          last_direction = RIGHT;
          delay(10);
        }
      } else { // Back up a little to readjust the robot if it is bobbing too much
        left_dir = BACKWARD;
        right_dir = BACKWARD;
        left_motor->run(left_dir);
        left_motor->run(right_dir);
        delay(10);
      }
    }
  }

  if (udp.parsePacket()) {
    int data = udp.available();
    udp.read(packet, 255);

    if (String(packet).substring(0, String(packet).length() - 1) == ("up")) { // Increase speed
      added_speed += 10;
    } else if (String(packet).substring(0, String(packet).length() - 1) == ("down")) { // Decrease speed
      added_speed -= 10;
    }
    udp.beginPacket(udp.remoteIP(), udp.remotePort());
      udp.println("send");
    udp.endPacket();

    for (int i = 0; i < 16; i++) {
      packet[i] = 0;
    }
  }

  int left_speed, right_speed;
  // Speed determined by base speed, added speed, accumulator and whether it is an inside wheel (if it is going backwards)
  left_speed = BASE_LEFT_SPEED + added_speed + ((left_dir == BACKWARD) ? 12 + ((accumulator > 5 && left_dir == BACKWARD && right_dir == BACKWARD) ? accumulator + 2 : accumulator) : 0);
  right_speed = BASE_RIGHT_SPEED + added_speed + ((right_dir == BACKWARD) ? 12 + ((accumulator > 5 && left_dir == BACKWARD && right_dir == BACKWARD) ? accumulator + 2 : accumulator) : 0);
  left_motor->setSpeed(left_speed);
  right_motor->setSpeed(right_speed);

  // Sets previous tags
  prev_left_tag = (left_on) ? prev_left_tag : left_tag;
  prev_right_tag = (left_on) ? prev_right_tag : right_tag;
}
