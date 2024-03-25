/***************************************************************
   Motor driver definitions
   
   Add a "#elif defined" block to this file to include support
   for a particular motor driver.  Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   *************************************************************/

#ifdef USE_BASE
   
#ifdef MAXON_MOTOR_DRIVER

  void initMotorController() {
    digitalWrite(RIGHT_MOTOR_ENABLE, HIGH);
    digitalWrite(LEFT_MOTOR_ENABLE, HIGH);
  }
  
  void setMotorSpeed(int i, int spd) {
    unsigned char reverse = 0;

    spd = map(spd, 0, 4500, 25.5, 229.5);
  
    if (spd < 0)
    { 
      spd = -spd;
      reverse = 1;
    }
    if (spd > 230)
      spd = 253; // 253 MAX

    else if (spd < 26 && i == LEFT) {
      digitalWrite(LEFT_MOTOR_ENABLE, LOW); 
      return; 
    }
    else if (spd < 26 && i == RIGHT) {
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
  
  void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    setMotorSpeed(LEFT, leftSpeed);
    setMotorSpeed(RIGHT, rightSpeed);
  }
#else
  #error A motor driver must be selected!
#endif

#endif
