/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/

#ifdef USE_MAXON_MOTOR

  #define RIGHT_MOTOR_MOVE 4
  #define RIGHT_MOTOR_ENABLE 3
  #define RIGHT_MOTOR_DIRECTION 2
  
  #define LEFT_MOTOR_MOVE 7
  #define LEFT_MOTOR_ENABLE 6
  #define LEFT_MOTOR_DIRECTION 5

  #define MAX_PWM 229.5
  #define MIN_PWM 25.5
  #define MAX_SPEED 7500

  void initMotorController();
  void setMotorSpeed(int i, int spd);
  void setMotorSpeeds(int leftSpeed, int rightSpeed, bool sweeper_blocked);

#endif

#ifdef USE_SWEEPERS

  #define RIGHT_SWEEPER_MOVE 9 // M2 motor (RIGHT)
  #define RIGHT_SWEEPER_DIRECTION 8
  #define RIGHT_SWEEPER_IS A1 
  #define RIGHT_SWEEPER_REVERSE_SPEED 80
  
  #define LEFT_SWEEPER_MOVE 11 // M1 motor (LEFT)
  #define LEFT_SWEEPER_DIRECTION 10
  #define LEFT_SWEEPER_IS A0
  #define LEFT_SWEEPER_REVERSE_SPEED 85
  int RIGHT_SWEEPER_SPEED = 120; //115
  int LEFT_SWEEPER_SPEED = 135; //120

  void activateSweeper();
  void stopSweeper();
  void reverseSweeper();
  void invertSweeperSpeeds();

#endif

