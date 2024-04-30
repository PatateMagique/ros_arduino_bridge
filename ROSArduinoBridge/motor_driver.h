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

  #define MAX_PWM 229.5
  #define MIN_PWM 25.5
  #define MAX_SPEED 4500

  void initMotorController();
  void setMotorSpeed(int i, int spd);
  void setMotorSpeeds(int leftSpeed, int rightSpeed);

#endif

#ifdef USE_SWEEPERS

  #define RIGHT_SWEEPER_MOVE 3 // M2 motor (RIGHT)
  #define RIGHT_SWEEPER_DIRECTION 11 
  #define RIGHT_SWEEPER_IS A1 
  #define RIGHT_SWEEPER_SPEED 145
  #define RIGHT_SWEEPER_REVERSE_SPEED 100
  
  #define LEFT_SWEEPER_MOVE 4 // M1 motor (LEFT)
  #define LEFT_SWEEPER_DIRECTION 2
  #define LEFT_SWEEPER_IS A0
  #define LEFT_SWEEPER_SPEED 150
  #define LEFT_SWEEPER_REVERSE_SPEED 105

  bool sweeper_blocked;

  void activateSweeper();
  void stopSweeper();
  void reverseSweeper();

#endif

