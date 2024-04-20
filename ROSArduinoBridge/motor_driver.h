/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/

#ifdef USE_MAXON_MOTOR

  #define RIGHT_MOTOR_MOVE 5
  #define RIGHT_MOTOR_ENABLE 6
  #define RIGHT_MOTOR_DIRECTION 7
  
  #define LEFT_MOTOR_MOVE   8
  #define LEFT_MOTOR_ENABLE 9
  #define LEFT_MOTOR_DIRECTION 10

  void initMotorController();
  void setMotorSpeed(int i, int spd);
  void setMotorSpeeds(int leftSpeed, int rightSpeed);

#endif

#ifdef USE_SWEEPERS

  #define RIGHT_SWEEPER_MOVE 3 // M2 motor (RIGHT)
  #define RIGHT_SWEEPER_DIRECTION 11 
  #define RIGHT_SWEEPER_IS A1 
  
  #define LEFT_SWEEPER_MOVE 4 // M1 motor (LEFT)
  #define LEFT_SWEEPER_DIRECTION 2
  #define LEFT_SWEEPER_IS A0

  bool sweeper_blocked;

  void activateSweeper();
  void stopSweeper();
  void reverseSweeper();

#endif

