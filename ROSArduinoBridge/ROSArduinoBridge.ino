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
#define BAUDRATE 9600

/* Include the Arduino standard libraries */
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <SPI.h>
#include <Wire.h>
#include <FastLED.h>
#include <TimeLib.h>
#include <VL53L0X.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

/* Include definition of serial commands */
#include "commands.h"

#ifdef USE_MAXON_MOTOR

  /* Motor driver function definitions */
  #include "motor_driver.h" 

  /* Stop the robot if it hasn't received a movement command in this number of milliseconds */
  #define AUTO_STOP_INTERVAL 1000

  long lastMotorCommand = AUTO_STOP_INTERVAL;

#endif

#ifdef USE_SERVOS

  /* Servo function definitions */
  #include "servo.h"
  #define SERVO_SAFETY 500
  enum ServoState {ROTATING_R, ROTATING_L, IDLE}; // servo state: rotating/idle
  ServoState servo_state = IDLE;
  bool ServoUp = false;
  bool ServoDown = true;
  unsigned long timerStart = 0;
  bool timerActive = false;
  unsigned long previousMillis_SERVO = 0;

#endif

#ifdef USE_SWEEPERS

  #define LEFT_SWEEPER_ISSUE 45
  #define RIGHT_SWEEPER_ISSUE 40
  #define SWEEPER_CHECK_INTERVAL 200

  /* Stop the sweepers if the robot hasn't received a movement command in this number of milliseconds */
  #define AUTO_STOP_SWEEPERS_INTERVAL 1000

  /* Reset the sweepers counter if they haven't had a problem in this number of milliseconds */
  #define SWEEPER_MEMORY 2000

  /* Stop the sweepers after this number of milliseconds if they had to run in reverse */
  #define SWEEPER_REVERSE_TIME 2000

  long lastSweeperProblem = SWEEPER_MEMORY;
  long lastSweeperReverse = SWEEPER_REVERSE_TIME;

  bool sweeper_blocked = false;
  int counter = 0; // The counter used for current sense and diagnosis

#endif

/* Global Variable initialization */

// Serial command variables
int arg = 0;    // A pair of varibles to help parse serial commands (thanks Fergs)
int index = 0;
char chr;       // Variable to hold an input character
char cmd;       // Variable to hold the current single-character command
char argv1[16]; // Character arrays to hold the first and second arguments
char argv2[16];
long arg1;
long arg2;

// Variables used fo the open-loop feedback
float current_speed_l = 0;
float current_speed_r = 0;

// LED Variables
tmElements_t tm;
#define PIN_LEDS     A2
#define NUM_LEDS     22
CRGB leds[NUM_LEDS];
unsigned long previousMillis_LED = 0;
unsigned long previousMillis_LED_2 = 0;
unsigned long interval_LED = 50;
unsigned long interval_LED_2 = 50;
bool ledsOn = false;
int fadeCounter = 255;
int current_color = 0;
bool fadeDirection = true;  // true for increasing brightness, false for decreasing
bool colorDirection = true; // true for increasing color, false for decreasing

// Voltage Variables
#define BATTERY_VOLTAGE A4
#define NUM_READINGS 5                // Number of readings for the median filter
unsigned long previousMillis_V = 0;
float voltage_battery = 0;
int battery_pourcentage = 0;
const float alpha = 0.1;              // Smoothing factor for EMA
bool first_reading = true;            // Flag to indicate the first reading
float voltageReadings[NUM_READINGS];  // Array to store voltage readings
int currentIndex = 0;                 // Current index in the array

// Screen Variables
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET    -1
#define SCREEN_ADDRESS 0x3C
#define OLED_INTERVAL 500
#define SENSOR_INTERVAL 20
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
unsigned long startTime;
unsigned long previousMillis_OLED = 0;
unsigned long previousMillis_SENSOR = 0;

// Sensor Duplo variables
VL53L0X sensor;
int duplo_eaten = 0;
int duplo_storage = 0;
bool sensorBelowThreshold = false;
unsigned long debounceDelay = 2000; // Debounce delay in milliseconds
unsigned long lastDuploTime = 0;    // Timestamp of the last Duplo detection
bool newDuplo = false;

// Buzzer Variables
#define BUZZER A7
#define BUZZER_INTERVAL 900
#define BUZZER_ON_DURATION 100
unsigned long previousMillis_BUZZER = 0;
unsigned long previousMillis_BUZZER_ON = 0;
bool buzzerOn = false;

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
    case GET_SYSTEM_DATA:
      Serial.print(current_speed_l);
      Serial.print(" ");
      Serial.print(current_speed_r);
      Serial.print(" ");
      Serial.print(battery_pourcentage);
      Serial.print(" ");
      Serial.println(duplo_eaten);
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
      if (arg1 != 0 || arg2 != 0) lastMotorCommand = millis(); 
      setMotorSpeeds(arg1, arg2, sweeper_blocked);
      current_speed_l = arg1;
      current_speed_r = arg2;
      Serial.println("OK");
      break;
  #endif 

  #ifdef USE_SERVOS
    case SERVO_WRITE:
      if (arg1 == 2) {
        servo_state = ROTATING_L;
        Serial.println("OK");
      } else if (arg1 == 1) {
        if (servo_state == IDLE) {
          servo_state = ROTATING_R; }
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

  startTime = millis();

  Serial.begin(BAUDRATE);
  Wire.begin();

  // Set up the MAXON and SWEEPERS pins
  pinMode(RIGHT_MOTOR_DIRECTION, OUTPUT);   // Digital 2
  pinMode(RIGHT_MOTOR_ENABLE, OUTPUT);      // Digital 3
  pinMode(RIGHT_MOTOR_MOVE, OUTPUT);        // Digital 4

  pinMode(LEFT_MOTOR_DIRECTION, OUTPUT);    // Digital 5
  pinMode(LEFT_MOTOR_ENABLE, OUTPUT);       // Digital 6
  pinMode(LEFT_MOTOR_MOVE, OUTPUT);         // Digital 7

  pinMode(LEFT_SWEEPER_MOVE, OUTPUT);       // Digital 8
  pinMode(LEFT_SWEEPER_DIRECTION, OUTPUT);  // Digital 9
  pinMode(RIGHT_SWEEPER_MOVE, OUTPUT);      // Digital 10
  pinMode(RIGHT_SWEEPER_DIRECTION, OUTPUT); // Digital 11

  pinMode(LEFT_SWEEPER_IS,INPUT);           // Analog 0
  pinMode(RIGHT_SWEEPER_IS,INPUT);          // Analog 1
  pinMode(EOC_SWITCH,INPUT);                // Analog 2
  pinMode(RIGHT_MOTOR_DIRECTION, OUTPUT);   // Analog 3
  pinMode(BATTERY_VOLTAGE, INPUT);          // Analog 4
  pinMode(BUZZER, OUTPUT);                  // Analog 7

  initMotorController(); 

  battery_voltage();   // Initialize the battery voltage

  // Initialize the LEDs
  FastLED.addLeds<WS2812, PIN_LEDS, GRB>(leds, NUM_LEDS);
  for(size_t i=0; i<NUM_LEDS; i++){
      leds[i] = CRGB(150, 0, 0);
  }
  FastLED.show();

  // Initialize the VL53L0X sensor
  sensor.setTimeout(800);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
  }
  sensor.startContinuous();

  // Initialize the OLED display
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  display.display();
  delay(500);
  display.clearDisplay();
}

void current_sense() {  // current sense and diagnosis
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

void battery_voltage() {
  int val = analogRead(BATTERY_VOLTAGE);
  float voltage_received = (val * 5.0) / 1023.0;
  float voltage_real = voltage_received * 2.5; // Multiply by 2.5 to get the real voltage

  // If this is the first reading, fill the array with the first reading value
  if (first_reading) {
    for (int i = 0; i < NUM_READINGS; i++) {
      voltageReadings[i] = voltage_real;
    }
    voltage_battery = voltage_real;
    first_reading = false;
    currentIndex = 0;
  } else {
    // Store the reading in the array
    voltageReadings[currentIndex] = voltage_real;
    currentIndex++;
    // Check if the array is filled
    if (currentIndex >= NUM_READINGS) {
      currentIndex = 0;  // Reset the index
    }
  }

  // Copy readings into a temporary array for sorting 
  float sortedReadings[NUM_READINGS];
  for (int i = 0; i < NUM_READINGS; i++) {
    sortedReadings[i] = voltageReadings[i];
  }

  // Sort the array to find the median
  for (int i = 0; i < NUM_READINGS - 1; i++) {
    for (int j = i + 1; j < NUM_READINGS; j++) {
      if (sortedReadings[i] > sortedReadings[j]) {
        float temp = sortedReadings[i];
        sortedReadings[i] = sortedReadings[j];
        sortedReadings[j] = temp;
      }
    }
  }
  // Find the median value
  float medianVoltage;
  if (NUM_READINGS % 2 == 0) {
    medianVoltage = (sortedReadings[NUM_READINGS / 2 - 1] + sortedReadings[NUM_READINGS / 2]) / 2.0;
  } else {
    medianVoltage = sortedReadings[NUM_READINGS / 2];
  }
  // Use the median value to update the battery voltage
  voltage_battery = alpha * medianVoltage + (1 - alpha) * voltage_battery;

  // Compute battery percentage (0% = 10.3V, 100% = 12.55V)
  if (voltage_battery < 10.3) {
    battery_pourcentage = 0;
  } else if (voltage_battery > 12.52) {
    battery_pourcentage = 100;
  } else {
    battery_pourcentage = round(((voltage_battery - 10.3) / 2.22) * 100);
  }
}

void control_LEDs() {

  unsigned long currentMillis_LED = millis();

  if (newDuplo) {
    fill_solid(leds, NUM_LEDS, CRGB(255, 255, 255));
    FastLED.show();
    tone(BUZZER, 340);
    if (currentMillis_LED - previousMillis_LED >= 300) {
      previousMillis_LED = currentMillis_LED;
      newDuplo = false;
    }
  } else if (current_speed_l == 0 && current_speed_r == 0 && servo_state != ROTATING_L && servo_state != ROTATING_R) {
    if (currentMillis_LED - previousMillis_LED >= interval_LED) {
      previousMillis_LED = currentMillis_LED;

      // Update LED brightness
      if (fadeDirection) {
        fadeCounter += 3;
        if (fadeCounter >= 255) {
          fadeDirection = false;
          if (colorDirection) {
            current_color = current_color + 3;
            if (current_color >= 30) {
              colorDirection = false;
            }
          } else {
            current_color = current_color - 3;
            if (current_color <= 0) {
              colorDirection = true;
            }
          }
        }
      } else {
        fadeCounter -= 3;
        if (fadeCounter <= 120) {
          fadeDirection = true;
          if (colorDirection) {
            current_color = current_color + 3;
            if (current_color >= 30) {
              colorDirection = false;
            }
          } else {
            current_color = current_color - 3;
            if (current_color <= 0) {
              colorDirection = true;
            }
          }
        }
      }
    }
    fill_solid(leds, NUM_LEDS, CHSV(current_color, 255, fadeCounter));
    FastLED.show();
  } else if (servo_state == ROTATING_L or servo_state == ROTATING_R) {
    if (currentMillis_LED - previousMillis_LED >= interval_LED) {
      previousMillis_LED = currentMillis_LED;

      // Update LED brightness
      if (fadeDirection) {
        fadeCounter += 3;
        if (fadeCounter >= 255) {
          fadeDirection = false;
        }
      } else {
        fadeCounter -= 3;
        if (fadeCounter <= 0) {
          fadeDirection = true;
        }
      }
      if (servo_state == ROTATING_L) {
        fill_solid(leds, NUM_LEDS, CHSV(190, 255, fadeCounter));
      } else {
        fill_solid(leds, NUM_LEDS, CHSV(120, 255, fadeCounter));
      }
      FastLED.show();
    }
  } else {
    if (currentMillis_LED - previousMillis_LED_2 >= interval_LED_2) {
      previousMillis_LED_2 = currentMillis_LED;

      interval_LED_2 = random(100, 501); // Set a new random interval

      // Toggle the LEDs on and off
      if (ledsOn) {
        fill_solid(leds, NUM_LEDS, CRGB(0, 0, 0));
      } else {
        fill_solid(leds, NUM_LEDS, CRGB(120, 0, 0));
      }
      ledsOn = !ledsOn;
      FastLED.show();
    }
  }
}

void drawBattery() {
  display.clearDisplay();
  
  // Calculate the width of the text to center it
  int16_t x1, y1;
  uint16_t w, h;
  char buffer[5];
  sprintf(buffer, "%d%%", battery_pourcentage);
  display.setTextSize(2);
  display.getTextBounds(buffer, 0, 0, &x1, &y1, &w, &h);
  
  // Draw percentage text above the battery
  int16_t textX = (50 - w) / 2;
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(textX, 0);
  display.print(buffer);
  
  // Draw battery outline at the bottom left
  display.drawRect(0, 16, 50, 14, SSD1306_WHITE);  // Longer battery body
  display.fillRect(50, 19, 2, 8, SSD1306_WHITE);   // Battery head
  
  // Draw battery level
  int width = map(battery_pourcentage, 0, 100, 0, 46);
  display.fillRect(2, 18, width, 10, SSD1306_WHITE);
}

void drawCounter() {
  // Calculate elapsed time in seconds
  unsigned long elapsedTime = (millis() - startTime) / 1000;
  
  // Calculate minutes and seconds from elapsed time
  unsigned int minutes = elapsedTime / 60;
  unsigned int seconds = elapsedTime % 60;

  // Set text size and color for elapsed time
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(60, 0);
  
  // Print elapsed time in MM:SS format
  display.print("Time: ");
  if (minutes < 10) display.print('0');
  display.print(minutes);
  display.print(':');
  if (seconds < 10) display.print('0');
  display.print(seconds);

  // Set text size and color for Duplos count
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(60, 10);
  display.print("Total:   ");
  display.print(duplo_eaten);

  display.setCursor(60, 20);
  display.print("Current: ");
  display.print(duplo_storage);

  // Display the buffer on the screen
  display.display();
}

void detect_duplo() {
  unsigned long currentMillis = millis();

  int sensorValue = sensor.readRangeContinuousMillimeters();
  if (sensor.timeoutOccurred()) { 
    Serial.print("SENSOR TIMEOUT "); 
    return;  // Skip the rest of the loop if a timeout occurred
  }

  // Check if sensor value is below the threshold
  if (sensorValue < 230) {
    if (!sensorBelowThreshold && (currentMillis - lastDuploTime >= debounceDelay)) {
      newDuplo = true;
      playBuzzer();
      duplo_eaten++;
      duplo_storage++;
      sensorBelowThreshold = true;
      lastDuploTime = currentMillis; // Update the timestamp
    }
  } else if (sensorValue > 230){
    sensorBelowThreshold = false;
  }
}

void playBuzzer() {
  unsigned long currentMillis = millis();

  if (!buzzerOn && currentMillis - previousMillis_BUZZER >= BUZZER_INTERVAL) {
    previousMillis_BUZZER = currentMillis;
    previousMillis_BUZZER_ON = currentMillis;
    tone(BUZZER, 340);
    buzzerOn = true;
  } else if (buzzerOn && currentMillis - previousMillis_BUZZER_ON >= BUZZER_ON_DURATION) {
    noTone(BUZZER);
    buzzerOn = false;
  }
}

/* Enter the main loop.  Read and parse input from the serial port,
 run any valid commands and check for auto-stop conditions. */
void loop() {

  if (millis() - previousMillis_SENSOR >= SENSOR_INTERVAL) {
    detect_duplo();
    previousMillis_SENSOR = millis();
  }
  if (millis() - previousMillis_OLED >= OLED_INTERVAL) {
    battery_voltage();
    previousMillis_OLED = millis();
    drawBattery();
    drawCounter();
  }
  // actiavte buzzer if the robot goes backward
  if (current_speed_l < 0 && current_speed_r < 0) playBuzzer();
  else noTone(BUZZER);

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
    //Serial.print(chr);

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
      setMotorSpeeds(0, 0, sweeper_blocked);
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
      counter = 0;
      lastSweeperProblem = millis();
    } // Stop the reverse process after a moment
    if (sweeper_blocked && (millis() - lastSweeperReverse) > SWEEPER_REVERSE_TIME) {  
      sweeper_blocked = false;
      stopSweeper();
    }
  #endif

  #ifdef USE_SERVOS
    if (servo_state == ROTATING_L){
      if (digitalRead(EOC_SWITCH) != 0 && ServoDown){
        turnServo(LEFT, 500);
      } else {
        stopServo();
        ServoDown = false;
        duplo_storage = 0;
        servo_state = IDLE;
      }
    } else if (servo_state == ROTATING_R) {
      if (!timerActive && !ServoDown){
        timerStart = millis();      // Start the timer
        timerActive = true;         // Activate the timer
      } 
      if (timerActive) {
        if (!ServoDown && (millis() - timerStart <= 2750)){ //4600 ms for 600 speed
          turnServo(RIGHT, 1000);
        } else {
          ServoDown = true;
          stopServo(); 
          timerActive = false;        // Deactivate the timer
          servo_state = IDLE;
        }
      } else {
        servo_state = IDLE;
      }
    } else if (servo_state == IDLE && (millis() - previousMillis_SERVO >= SERVO_SAFETY)){
      stopServo();
      previousMillis_SERVO = millis();
    } 
  #endif

  control_LEDs();
}