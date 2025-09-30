#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

#define byte uint8_t

#define LEFT_WHEEL 1
#define RIGHT_WHEEL 2

Adafruit_MotorShield motor_shield = Adafruit_MotorShield(); 

Adafruit_DCMotor *left_motor = motor_shield.getMotor(LEFT_WHEEL);
Adafruit_DCMotor *right_motor = motor_shield.getMotor(RIGHT_WHEEL);

int speed = 150;


void setup() {
  motor_shield.begin();
}

void loop() {
  left_motor->setSpeed(speed);
  right_motor->setSpeed(speed);

  left_motor->run(FORWARD);
  right_motor->run(FORWARD);
}
