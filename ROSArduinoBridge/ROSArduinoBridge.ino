/*********************************************************************
 *  ROSArduinoBridge
 
    A set of simple serial commands to control a differential drive
    robot and receive back sensor and odometry data. Default 
    configuration assumes use of an Arduino Mega + Pololu motor
    controller shield + Robogaia Mega Encoder shield.  Edit the
    readEncoder() and setMotorSpeed() wrapper functions if using 
    different motor controller or encoder method.

    Created for the Pi Robot Project: http://www.pirobot.org
    and the Home Brew Robotics Club (HBRC): http://hbrobotics.org
    
    Authors: Patrick Goebel, James Nugen

    Inspired and modeled after the ArbotiX driver by Michael Ferguson
    
    Software License Agreement (BSD License)

    Copyright (c) 2012, Patrick Goebel.
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

     * Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above
       copyright notice, this list of conditions and the following
       disclaimer in the documentation and/or other materials provided
       with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#define USE_MAXON_MOTOR
//#undef USE_MAXON_MOTOR // Comment this line out if you want to use the Maxon motors
#define USE_SERVOS
//#undef USE_SERVOS // Comment this line out if you want to use the servos
#define USE_SWEEPERS
//#undef USE_SWEEPERS // Comment this line out if you want to use the sweepers

/* Serial port baud rate */
#define BAUDRATE     57600

/* Include the Arduino standard libraries */
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

/* Include definition of serial commands */
#include "commands.h"

#ifdef USE_MAXON_MOTOR

  /* Motor driver function definitions */
  #include "motor_driver.h" 

  /* Stop the robot if it hasn't received a movement command
  in this number of milliseconds */
  #define AUTO_STOP_INTERVAL 1000

  long lastMotorCommand = AUTO_STOP_INTERVAL;

#endif

#ifdef USE_SERVOS

  /* Servo function definitions */
  #include "servo.h"

#endif

#ifdef USE_SWEEPERS

  /* Sweeper check parameters*/
  #define LEFT_SWEEPER_ISSUE 45
  #define RIGHT_SWEEPER_ISSUE 40
  #define SWEEPER_CHECK_INTERVAL 200

  /* Stop the sweepers if the robot hasn't received a movement command
    in this number of milliseconds */
  #define AUTO_STOP_SWEEPERS_INTERVAL 2000

  /* Reset the sweepers counter if they haven't had a problem
    in this number of milliseconds */
  #define SWEEPER_MEMORY 2000

  /* Stop the sweepers after this number of milliseconds
    if they had to run in reverse */
  #define SWEEPER_REVERSE_TIME 2000

  long lastSweeperProblem = SWEEPER_MEMORY;
  long lastSweeperReverse = SWEEPER_REVERSE_TIME;

#endif

/* Variable initialization */

// A pair of varibles to help parse serial commands (thanks Fergs)
int arg = 0;
int index = 0;
// Variable to hold an input character
char chr;
// Variable to hold the current single-character command
char cmd;
// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];
// The arguments converted to integers
long arg1;
long arg2;

// The counter used for current sense and diagnosis
int counter = 0;
// Variables used fo the open-loop feedback
float current_speed_l = 0;
float current_speed_r = 0;

enum ServoState {ROTATING_R, ROTATING_L, IDLE}; // servo state: rotating/idle
ServoState servo_state = IDLE;
bool ServoUp = false;
bool ServoDown = true;
unsigned long timerStart = 0;
bool timerActive = false;

/* Clear the current command parameters */
void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index = 0;
}

/* Run a command.  Commands are defined in commands.h */
int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);
  
  switch(cmd) {
  case GET_BAUDRATE:
    Serial.println(BAUDRATE);
    break;
  case ANALOG_READ:
    Serial.print(current_speed_l);
    Serial.print(" ");
    Serial.println(current_speed_r);
    break;
  case DIGITAL_READ:
    Serial.println(digitalRead(arg1));
    break;
  case ANALOG_WRITE:
    analogWrite(arg1, arg2);
    Serial.println("OK"); 
    break;
  case DIGITAL_WRITE:
    if (arg2 == 0) digitalWrite(arg1, LOW);
    else if (arg2 == 1) digitalWrite(arg1, HIGH);
    Serial.println("OK"); 
    break;
  case PIN_MODE:
    if (arg2 == 0) pinMode(arg1, INPUT);
    else if (arg2 == 1) pinMode(arg1, OUTPUT);
    Serial.println("OK");
    break;
  
#ifdef USE_MAXON_MOTOR
  case MOTOR_RAW_PWM:
    /* Reset the auto stop timer */
    lastMotorCommand = millis();
    setMotorSpeeds(arg1, arg2);
    current_speed_l = arg1;
    current_speed_r = arg2; 
    Serial.println("OK"); 
    break;
#endif

#ifdef USE_SERVOS
  case SERVO_WRITE:
    if (arg1 == 1) {
      servo_state = ROTATING_L;
      Serial.println("OK");
    } else if (arg1 == 0) {
      servo_state = ROTATING_R;
      Serial.println("OK");
    } else {
      servo_state = IDLE;
      Serial.println("OK");
    }    
    break;
#endif

  default:
    Serial.println("Invalid Command");
    break;
  }
}

/* Setup function--runs once at startup. */
void setup() {

  #ifdef USE_SERVOS
    setupServo();
  #endif  

  Serial.begin(BAUDRATE);

  // Set up the MAXON and SWEEPERS pins
  pinMode(LEFT_SWEEPER_DIRECTION, OUTPUT);  // Digital 2
  pinMode(RIGHT_SWEEPER_MOVE, OUTPUT);      // Digital 3
  pinMode(LEFT_SWEEPER_MOVE, OUTPUT);       // Digital 4
  pinMode(RIGHT_MOTOR_MOVE, OUTPUT);        // Digital 5
  pinMode(RIGHT_MOTOR_ENABLE, OUTPUT);      // Digital 6
  pinMode(RIGHT_MOTOR_DIRECTION, OUTPUT);   // Digital 7
  pinMode(LEFT_MOTOR_MOVE, OUTPUT);         // Digital 8
  pinMode(LEFT_MOTOR_ENABLE, OUTPUT);       // Digital 9
  pinMode(LEFT_MOTOR_DIRECTION, OUTPUT);    // Digital 10
  pinMode(RIGHT_SWEEPER_DIRECTION, OUTPUT); // Digital 11
  pinMode(LEFT_SWEEPER_IS,INPUT);           // Analog 0
  pinMode(RIGHT_SWEEPER_IS,INPUT);          // Analog 1
  pinMode(EOC_SWITCH,INPUT);                // Analog 2


  initMotorController();
}

void current_sense()                  // current sense and diagnosis
{
  int val_L=analogRead(LEFT_SWEEPER_IS);
  int val_R=analogRead(RIGHT_SWEEPER_IS);

  if(val_L > LEFT_SWEEPER_ISSUE || val_R > RIGHT_SWEEPER_ISSUE){
    counter++;
    lastSweeperProblem = millis();
    if(counter==3){
      reverseSweeper();
      lastSweeperReverse = millis();
      sweeper_blocked = true;
      counter=0;
    }
  }
}

/* Enter the main loop.  Read and parse input from the serial port,
 run any valid commands and check for auto-stop conditions. */
void loop() {

  #ifdef USE_SWEEPERS
    static unsigned long timePoint = 0;    // current sense and diagnosis,if you want to use this
    if(millis() - timePoint > SWEEPER_CHECK_INTERVAL){ 
      current_sense();
      timePoint = millis();
    }
  #endif
  
  while (Serial.available() > 0) {
    
    // Read the next character
    chr = Serial.read();

    // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1) argv1[index] = NULL;
      else if (arg == 2) argv2[index] = NULL;
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[index] = NULL;
        arg = 2;
        index = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[index] = chr;
        index++;
      }
      else if (arg == 2) {
        argv2[index] = chr;
        index++;
      }
    }
  }
  
  #ifdef USE_MAXON_MOTOR
    // Check to see if we have exceeded the auto-stop interval
    if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {;
      setMotorSpeeds(0, 0);
      current_speed_l = 0;
      current_speed_r = 0;
    } 
  #endif
  #ifdef USE_SWEEPERS
    // Stop the sweepers if the robot hasn't received a movement command 
    if ((millis() - lastMotorCommand) > AUTO_STOP_SWEEPERS_INTERVAL) {;
      stopSweeper();
    } // Reset the sweepers counter if they haven't had a problem
    if ((millis() - lastSweeperProblem) > SWEEPER_MEMORY) {;
      sweeper_blocked= false;
      counter = 0;
      lastSweeperProblem = millis();
    } // Stop the reverse process after a moment
    if (sweeper_blocked && (millis() - lastSweeperReverse) > SWEEPER_REVERSE_TIME) {  
      sweeper_blocked= false;
      stopSweeper();
    }
  #endif


  if (servo_state == ROTATING_L){
    if (!ServoUp){
      turnServo(LEFT, 500);
    } // Check if the end-of-course switch is triggered
    if (digitalRead(EOC_SWITCH) == 0){
      stopServo();
      ServoUp = true; // Prevent the servo from breaking the EOC switch
      ServoDown = false;
      servo_state = IDLE;
    }
  }
  if (servo_state == ROTATING_R) {
    if (!timerActive && !ServoDown){
      ServoUp = false;
      timerStart = millis();      // Start the timer
      timerActive = true;         // Activate the timer
    }
    if (!ServoDown){
      turnServo(RIGHT, 600);
    }
    // Check if 4.4 seconds have passed since the timer was started
    if (millis() - timerStart >= 4400) {
      ServoDown = true;
      stopServo(); 
      servo_state = IDLE;         // Change state to IDLE
      timerActive = false;        // Deactivate the timer
    }
  }
  if (servo_state == IDLE) {
    stopServo(); 
  }
}



// Code to put in loop() if we want to use the Maxon motors in closed loop

// int LEFT_VOLTAGE = ((analogRead(A3) * 5.0)/1023.0);
  // int RIGHT_VOLTAGE = ((analogRead(A1) * 5.0)/1023.0);

  // int RPM_LEFT_READING = 0;
  // int RPM_RIGHT_READING = 0;

  // if (analogRead(A3) > 610) {
  //   RPM_LEFT_READING   = ((((analogRead(A3) * 5.0)/1023.0)+(0.09*((analogRead(A3) * 5.0)/1023.0)/3.91))*4500.0)/4; // LEFT MOTOR
  // } 
  // else if (analogRead(A3) > 410) {
  //   RPM_LEFT_READING   = ((((analogRead(A3) * 5.0)/1023.0)+(0.112*((analogRead(A3) * 5.0)/1023.0)/3.91))*4500.0)/4; // LEFT MOTOR
  // } 
  // else if (analogRead(A3) > 110) {
  //   RPM_LEFT_READING   = ((((analogRead(A3) * 5.0)/1023.0)+(0.12*((analogRead(A3) * 5.0)/1023.0)/3.91))*4500.0)/4; // LEFT MOTOR
  // }
  // else {
  //   RPM_LEFT_READING   = ((((analogRead(A3) * 5.0)/1023.0)+(0.75*((analogRead(A3) * 5.0)/1023.0)/3.91))*4500.0)/4; // LEFT MOTOR
  // }

  // if (analogRead(A1) > 610) {
  //   RPM_RIGHT_READING  = ((((analogRead(A1) * 5.0)/1023.0)+(0.09*((analogRead(A1) * 5.0)/1023.0)/3.91))*4500.0)/4; // RIGHT MOTOR
  // } 
  // else if (analogRead(A1) > 410) {
  //   RPM_RIGHT_READING  = ((((analogRead(A1) * 5.0)/1023.0)+(0.112*((analogRead(A1) * 5.0)/1023.0)/3.91))*4500.0)/4; // RIGHT MOTOR
  // } 
  // else if (analogRead(A1) > 110) {
  //   RPM_RIGHT_READING  = ((((analogRead(A1) * 5.0)/1023.0)+(0.12*((analogRead(A1) * 5.0)/1023.0)/3.91))*4500.0)/4; // RIGHT MOTOR
  // }
  // else {
  //   RPM_RIGHT_READING  = ((((analogRead(A1) * 5.0)/1023.0)+(0.75*((analogRead(A1) * 5.0)/1023.0)/3.91))*4500.0)/4; // RIGHT MOTOR
  // }

  // // Calculate EMA
  // ema_l = alpha * RPM_LEFT_READING + (1 - alpha) * ema_l;
  // ema_r = alpha * RPM_RIGHT_READING + (1 - alpha) * ema_r;

  

  // And this at the top

  // Variables used for the RPM reading
// const int nb_samples = 20;
// const float alpha = 0.01; // Exponential filter factor (adjust as needed)

// float ema_l = 0.0; // Exponential moving average
// float ema_r = 0.0; // Exponential moving average