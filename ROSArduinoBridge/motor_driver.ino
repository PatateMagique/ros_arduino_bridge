/***************************************************************
   Motor driver definitions
   
   Add a "#elif defined" block to this file to include support
   for a particular motor driver.  Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   *************************************************************/
   
#ifdef USE_MAXON_MOTOR

  void initMotorController() {
    digitalWrite(RIGHT_MOTOR_ENABLE, HIGH);
    digitalWrite(LEFT_MOTOR_ENABLE, HIGH);
  }
  
  void setMotorSpeed(int i, int spd) {
    unsigned char reverse = 0;

    if (spd > 0)
    { 
      spd = map(spd, 0, MAX_SPEED, MIN_PWM, MAX_PWM);
    }
    else if (spd < 0)
    { 
      spd = -spd;
      reverse = 1;
      spd = map(spd, 0, MAX_SPEED, MIN_PWM, MAX_PWM);
    }
    if (spd <= MIN_PWM && i == LEFT) {
      digitalWrite(LEFT_MOTOR_ENABLE, LOW); 
      return; 
    }
    else if (spd <= MIN_PWM && i == RIGHT) {
      digitalWrite(RIGHT_MOTOR_ENABLE, LOW);
      return;
    }
    if (i == LEFT) { 
      digitalWrite(LEFT_MOTOR_ENABLE, HIGH);
      if      (reverse == 0) { digitalWrite(LEFT_MOTOR_DIRECTION, HIGH);}
      else if (reverse == 1) { digitalWrite(LEFT_MOTOR_DIRECTION, LOW);}
        analogWrite(LEFT_MOTOR_MOVE, spd);
    }
    else /*if (i == RIGHT) //no need for condition*/ {
      digitalWrite(RIGHT_MOTOR_ENABLE, HIGH);
      if      (reverse == 0) { digitalWrite(RIGHT_MOTOR_DIRECTION, LOW);}
      else if (reverse == 1) { digitalWrite(RIGHT_MOTOR_DIRECTION, HIGH);}
      analogWrite(RIGHT_MOTOR_MOVE, spd);
    }
  }

  void setMotorSpeeds(int leftSpeed, int rightSpeed, bool sweeper_blocked) {
    #ifdef USE_SWEEPERS
      if ((leftSpeed > 0 && rightSpeed > 0) && !sweeper_blocked) {
        activateSweeper();
      } else if ((leftSpeed < 0 || rightSpeed < 0) && !sweeper_blocked) {
        stopSweeper();
      } else if ((abs(leftSpeed) - abs(rightSpeed) <= 100) && (leftSpeed * rightSpeed < 0) && !sweeper_blocked) {
        // If the speeds of the wheels are almost equal (within a margin of 100) but with opposite signs, stop the sweeper
        stopSweeper();
      }
    #endif
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
}
#endif

#ifdef USE_SWEEPERS

  void activateSweeper() {
    analogWrite (RIGHT_SWEEPER_MOVE,RIGHT_SWEEPER_SPEED);
    digitalWrite(RIGHT_SWEEPER_DIRECTION,HIGH);
    analogWrite (LEFT_SWEEPER_MOVE,LEFT_SWEEPER_SPEED);
    digitalWrite(LEFT_SWEEPER_DIRECTION,LOW);
  }

  void stopSweeper() {
    analogWrite (RIGHT_SWEEPER_MOVE,0);
    analogWrite (LEFT_SWEEPER_MOVE,0);
  }

  void reverseSweeper() {
    analogWrite (RIGHT_SWEEPER_MOVE,RIGHT_SWEEPER_REVERSE_SPEED);
    digitalWrite(RIGHT_SWEEPER_DIRECTION,LOW);
    analogWrite (LEFT_SWEEPER_MOVE,LEFT_SWEEPER_REVERSE_SPEED);
    digitalWrite(LEFT_SWEEPER_DIRECTION,HIGH);
  }

#endif
