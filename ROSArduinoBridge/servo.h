#ifdef USE_SERVOS

#define N_SERVOS 1
#define EOC_SWITCH A2

#define DirectionPin  (10u)
#define BaudRate    (1000000ul)
#define SERVO_ID    (1u)

#define initialPosition  (512u)

void setupServo();
void stopServo();
void ledServo(int val);
void turnServo(int val);
void moveServo(int val);

#endif
