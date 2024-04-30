#ifdef USE_SERVOS

#include "servo.h"
#include "arduino.h"
#include "AX12A.h"

void moveServo(int val)          //Move servo to given position
{
    ax12a.setEndless(SERVO_ID, 0);     //0 = Joint mode, 1 = Wheel mode
    ax12a.move(SERVO_ID, val);
    Serial.println(val);
}

void turnServo(bool direction, int val)          //Turn servo in one direction
{   
    ax12a.setEndless(SERVO_ID, 1);     //0 = Joint mode, 1 = Wheel mode
    ax12a.turn(SERVO_ID, direction, val);
}

void ledServo(int val)           //Turn ON/OFF servo LED
{   
    if (val == 0) {
      ax12a.ledStatus(SERVO_ID, OFF);
    }
    else { 
      ax12a.ledStatus(SERVO_ID, ON);
    }
}

void stopServo(void)              //Stop servo
{
  ax12a.turn(SERVO_ID, LEFT, 0);
}

void setupServo(void)
{ 
  ax12a.begin(BaudRate, DirectionPin, &Serial1); //&Serial if TX, &Serial 1 if TX1
}
#endif