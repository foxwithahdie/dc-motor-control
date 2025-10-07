#define byte uint8_t

#define LEFT_WHEEL 1
#define RIGHT_WHEEL 2

#define RIGHT_FORWARD BACKWARD;

#define BASE_LEFT_SPEED 26
#define BASE_RIGHT_SPEED 25

byte swap_dir(byte dir) {
  switch (dir) {
    case FORWARD:
      return BACKWARD;
    case BACKWARD:
      return FORWARD;
  }
}

void drive(Adafruit_DCMotor *left_motor, Adafruit_DCMotor *right_motor, int left_speed, int right_speed, byte left_direction, byte right_direction) {
  left_motor->setSpeed(BASE_LEFT_SPEED + left_speed);
  right_motor->setSpeed(BASE_RIGHT_SPEED + right_speed);

  left_motor->run(left_direction);
  right_motor->run(right_direction);
}