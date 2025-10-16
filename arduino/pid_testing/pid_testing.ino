#include <Wire.h>
#include <Adafruit_MotorShield.h>
// #include <WiFiS3.h>
// #include <WiFiUdp.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include "arduino_secrets.h"
#include "direction_functions.h"

#include <PID_v1.h>

// WiFiUDP udp;
// int port = 8182;
// char packet[16];
// int dataLength;

Adafruit_MotorShield motor_shield = Adafruit_MotorShield();

Adafruit_DCMotor *left_motor = motor_shield.getMotor(LEFT_WHEEL);
byte left_dir = FORWARD;   // default
double left_speed = 0;
double left_sensor_read = 0;
double left_threshold = 0;
double left_prop = 10,
       left_int = 0,
       left_deriv = 0;

PID left_pid_handler = PID( 
                            &left_sensor_read, &left_speed, &left_threshold,
                            left_prop, left_int, left_deriv,
                            P_ON_E, DIRECT
                          );

Adafruit_DCMotor *right_motor = motor_shield.getMotor(RIGHT_WHEEL);
byte right_dir = FORWARD;  // default
double right_speed = 0;
double right_sensor_read = 0;
double right_threshold = 0;
double right_prop = 10,
       right_int = 0,
       right_deriv = 0;

PID right_pid_handler = PID(
                            &right_sensor_read, &right_speed, &right_threshold,
                            right_prop, right_int, right_deriv,
                            P_ON_E, DIRECT
                          );


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
  Serial.println();
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
  pinMode(LEFT_SENSOR, INPUT);
  pinMode(RIGHT_SENSOR, INPUT);
  auto_calibrate();

  left_pid_handler.SetMode(AUTOMATIC);
  right_pid_handler.SetMode(AUTOMATIC);
}

void loop() {
  left_sensor_read = analogRead(LEFT_SENSOR);
  right_sensor_read = analogRead(RIGHT_SENSOR);
  left_pid_handler.Compute();
  right_pid_handler.Compute();


  Serial.println(" --------------- CURRENT READ --------------- ");
  Serial.print("Left Speed: ");
  Serial.println(left_speed);
  Serial.print("Right Speed: ");
  Serial.println(right_speed);
  Serial.print("Left Sensor Read: ");
  Serial.println(left_sensor_read);
  Serial.print("Right Sensor Read: ");
  Serial.println(right_sensor_read);
  
  drive(left_motor, right_motor,
        left_speed, right_speed,
        left_dir, right_dir
  ); 
}
