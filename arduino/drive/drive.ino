#include <Wire.h>
#include <Adafruit_MotorShield.h>
// #include <WiFiS3.h>
// #include <WiFiUdp.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include "direction_functions.h"
// #include "arduino_secrets.h"

// wifi/udp values for sending data via wifi
// WiFiUDP udp;
// int port = 8182;
// char packet[16];
// int dataLength;

Adafruit_MotorShield motor_shield = Adafruit_MotorShield();

Adafruit_DCMotor *left_motor = motor_shield.getMotor(LEFT_WHEEL);
Adafruit_DCMotor *right_motor = motor_shield.getMotor(RIGHT_WHEEL);

int whole_speed = 25;
byte left_dir = FORWARD;         // default
byte right_dir = FORWARD;  // default


void setup() {
  motor_shield.begin();
  Serial.begin(9600);

  // Serial.setTimeout(0.000001);

  // prints ip once connected to wifi
  // WiFi.begin(SSID, PWD);
  // while (WiFi.status() != WL_CONNECTED) {
  //   delay(100);
  //   Serial.print("Connecting...");
  // }
  // Serial.print("IP = ");
  // Serial.println(WiFi.localIP());

  // udp.begin(8182);
}

// void auto_calibrate() {
//   Serial.print("Left Sensor On Line: ");
//   if (Serial.readString() == ("done\n")) {
    
//   }
// }


void loop() {
  left_motor->setSpeed(BASE_LEFT_SPEED);
  right_motor->setSpeed(BASE_RIGHT_SPEED);

  left_motor->run(left_dir);
  right_motor->run(right_dir);
  // drive(left_motor, right_motor, whole_speed, whole_speed, left_dir, right_dir);

  // if (Serial.readString() == ("switch\n")) {
  //   left_dir = swap_dir(left_dir);
  //   right_dir = swap_dir(right_dir);
  // }

  // if (udp.parsePacket()) {
  //   int data = udp.available();
  //   udp.read(packet, 255);
  //   Serial.println(packet);

  //   if (String(packet).substring(0, String(packet).length() - 1) == ("switch")) {
  //     left_dir = swap_dir(left_dir);
  //     right_dir = swap_dir(right_dir);
  //   }
  //   udp.beginPacket(udp.remoteIP(), udp.remotePort());
  //   udp.println("send");
  //   udp.endPacket();
  //   for (int i = 0; i < 16; i++) {
  //     packet[i] = 0;
  //   }
  // }
}
