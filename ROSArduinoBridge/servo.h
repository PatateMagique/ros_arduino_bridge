#ifdef USE_SERVOS

#define EOC_SWITCH A3

#define DirectionPin (10u)
#define BaudRate     (1000000ul)
#define SERVO_ID     (1u)

void setupServo();
void stopServo();
void ledServo(int val);
void turnServo(int val);
void moveServo(int val);

#endif
