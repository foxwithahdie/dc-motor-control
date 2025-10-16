#define byte uint8_t

#define LEFT_WHEEL 1
#define RIGHT_WHEEL 2

#define LEFT_SENSOR A0
#define RIGHT_SENSOR A1

#define RIGHT_FORWARD BACKWARD;

#define BASE_LEFT_SPEED 25
#define BASE_RIGHT_SPEED 25

byte swap_dir(byte dir) {
  switch (dir) {
    case FORWARD:
      return BACKWARD;
    case BACKWARD:
      return FORWARD;
  }
}

void drive(Adafruit_DCMotor *left_motor_, Adafruit_DCMotor *right_motor_, int left_speed, int right_speed, byte left_direction, byte right_direction) {
  left_motor_->setSpeed(BASE_LEFT_SPEED + left_speed);
  right_motor_->setSpeed(BASE_RIGHT_SPEED + right_speed);

  left_motor_->run(left_direction);
  right_motor_->run(right_direction);
