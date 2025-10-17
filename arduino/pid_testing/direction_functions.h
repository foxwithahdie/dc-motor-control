#define byte uint8_t

#define LEFT_WHEEL 1
#define RIGHT_WHEEL 2

#define LEFT_SENSOR A0
#define RIGHT_SENSOR A1

#define BASE_LEFT_SPEED 30
#define BASE_RIGHT_SPEED 32

#define TURN_SPEED 60

typedef enum {
  LEFT,
  RIGHT,
  DRIVE
} DIRECTION;
