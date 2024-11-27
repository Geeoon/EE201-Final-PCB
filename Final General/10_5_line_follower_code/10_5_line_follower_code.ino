/* ************************************************************************************************* */
// * ECE 201: Line Following Robot with PID * //
/* ************************************************************************************************* */
// This is code for your PID controlled line following robot. 
//
// Code Table of Contents
  // 1) Declare Variables - declares many variables as global variables so each variable can be accessed from every function
  // 2) Setup (Main) - runs once at beginning when you press button on arduino or motor drive or when you open serial monitor
  // 3) Loop (Main) - loops forever calling on a series of functions 
  // 4) Blink - blinks and LED
  // 5) Calibration - makes white = 0 and black = 100 (a few seconds to prep, a few seconds on white, a few seconds to move to black, a few seconds of black)
  // 6) Read Potentiometers - reads each potentiometer
  // 7) Run Motors - runs motors
  // 8) Read Photoresistors - reads each photoresistor
  // 9) Calculate Error - calculate error from photoresistor readings
  // 10) PID Turn - takes the error and implements PID control
  // 11) Print - used for debugging but can comment out when not debugging because it slows down program

// ************************************************************************************************* //
// Declare Variables

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Variables and Libaries for Motor
#include <Wire.h>
#include <Adafruit_MotorShield.h> // Must add libary - see MotorShield Manual
//https://cdn-learn.adafruit.com/downloads/pdf/adafruit-motor-shield-v2-for-arduino.pdf

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Motors can be switched here (1) <--> (2)
Adafruit_DCMotor *Motor1 = AFMS.getMotor(1); // left motor
Adafruit_DCMotor *Motor2 = AFMS.getMotor(2); // right motor
  
// Set Initial Speed of Motors (CAN BE EDITED BY USER)
// TODO: Replace "___", and set the default motor speeds to an integer between 0 and 255.
//  the defualt motor speed is how fast the cart will move with no inputs. this can be
//  thought of as how fast should the cart be going when travelling in a straight line.
//  theoretical max. is 255, but the motors will likely overdraw power and cause the Arduino to shut off. 
//  motors likely need a minimum speed of at least 30 to move the cart.
//  a good starting speed is roughly 50-60. motor speeds are separated incase one motor turns faster than the other.  
const int M1Sp = 60; 
const int M2Sp = 60;

// Variables for Potentiometer
// TODO: Replace "___", and assign the pin number connected to the Arduino
const int S_pin = 0; //proportional control
const int P_pin = 1; //proportional control
const int I_pin = 2; //integral control
const int D_pin = 3; //derivative control

// User set SPID values
// TODO: Replace "___", and assign an integer value for the MAX value of S, P, I, and D.
//  The on-cart potentiometers are used to determine the actual values of S, P, I, and D. 
//  Note: These values will likely need to be experimentally determined
const float speed       = ___;
const float proportion  = ___;
const float integral    = ___;
const float derivative  = ___;

const int numMeas = ___; // TODO: Replace "___", and assign number of calibration measurements
                         //   setting numMeas to 40 as in code 10.4 is recommended

// Initialize SPID values to zero
int SpRead = 0; //Speed Increase 
int kPRead = 0; //proportional gain
int kIRead = 0; //integral gain
int kDRead = 0; //derivative gain
  
// Variables for Light Sensors (arrays are used to simplify the code):
//  these are variables that have multiple elements to each variable name.
//  LDR_Pin holds 7 values of the LDR pin connection to the Arduino
// TODO: Replace "___", and assign the pin number for each LDR connected to the Arduino.
//  reference 10.2 potentiometer code if necessarry
int LDR_Pin[7] = {___,
                  ___,
                  ___,
                  ___,
                  ___,
                  ___,
                  ___}; 
int LDR[7]; // this is an empty array that will hold the readings from our potentiometers

// TODO: Replace "___", and assign the pin number connected to the Arduino.
//  it is recommended to use the built-in LED connected to pin 13, but can change 
//  to another digital pin and connect external LED to be more easily seen
int led_Pin = 13; 

// Calibration Variables
//  Complete understanding of the calibration process is not required 
float Mn[7]; // array containing minimum read values of every potentiometer
float Mx[7]; // array containing maximum read values of every potentiometer
float LDRf[7] = {0.,0.,0.,0.,0.,0.,0.}; // initialize the LDR read values to float zero

int MxRead; // maximum read value by any potentiometer
int MxIndex; // index (location in array) of max read value
float AveRead; // average read value of potentiometer
int CriteriaForMax; // value to determine whether a read value is a max
float WeightedAve; // weighted average after calibration
  
// Error calculation variables
int im0, im1, im2; 
  
// Motor control variables
int M1SpeedtoMotor, M2SpeedtoMotor; // actual speeds motor will be rotating at
int Turn; // which direction and how sharply should the cart turn
int M1P = 0, M2P = 0; // proportion control for motors 1 and 2 respectively
float error; // current error
int lasterror = 0; // previous error
int sumerror = 0; // sum of total errors
float kP, kI, kD; // final values of P, I, and D used in control
 
// ************************************************************************************************* //
// setup - runs once
void setup() {
  Serial.begin(___); // TODO: Replace "___", and input the baud rate for serial communication
  AFMS.begin(); // initialize the motor

  pinMode(___, ___); // TODO: Replace "___", and set the led_pin to be an output
  
  Calibrate(); // Calibrate black and white sensing
  
  ReadPotentiometers(); // Read potentiometer values (Sp, P, I, & D)
  
  delay(2000); // Delay to ensure cart is properly placed before moving
  
  RunMotors(); // Starts motors straight forward depending on Sp (Speed from potentiometer) and M1Sp/M2Sp (Nominal values)
 
} // end setup()

// ************************************************************************************************* //
// loop - runs/loops forever
void loop() {
  // Note: some functions are only helpful for debugging and can be commented out once functionality is ensured
 
  ReadPotentiometers(); // Only if you want to see Potentiometers working in set up as you run the line following
  
  ReadPhotoResistors(); // Read photoresistors and map to 0-100 based on calibration
  
  CalcError();
  
  PID_Turn(); // PID Control and Output to motors to turn
  RunMotors(); // Uses info from
  
  Print(); // Print values to serial monitor //currently commented out but could be good for debugging =)
 
} // end loop()

// ************************************************************************************************* //
// function to quickly blink an LED
void fastBlink() {
  digitalWrite(___, ___); // TODO: Replace "___", and turn on the LED
  delay(100); // wait for 100 milliseconds

  digitalWrite(___, ___); // TODO: Replace "___", and turn off the LED
  delay(100); // wait for 100 milliseconds
} // end fastBlink()

// ************************************************************************************************* //
// function to slowly blink an LED
void slowBlink() {
  digitalWrite(___, ___); // TODO: Replace "___", and turn on the LED
  delay(100); // wait for 100 milliseconds

  digitalWrite(___, ___); // TODO: Replace "___", and turn off the LED
  delay(900); // wait for 900 milliseconds
} // end slowBlink()

// ************************************************************************************************* //
// function to calibrate
void Calibrate() { 
  // wait to make sure cart is in position with sensors over a white (light) surface
  for (int calii = 0; calii < 4; calii++) { // wait four blinks for the cart to be positioned
    slowBlink(); // blink the indicator LED
  }
  
  // Calibration
  // White Calibration
  for (int calii = ___; calii < ___; calii = ___) { // TODO: Replace "___", and repeat the section of code
                                                    //  for the number of calibration measurements desired

    fastBlink(); // blink the indicator LED

    for (int ci = ___; ci < ___; ci = ____) { // TODO: Replace "___", and loop over all of the LDRs
      LDRf[ci] = LDRf[ci] + (float) analogRead(LDR_Pin[ci]); // keep track of the sum of each LDR's readings
      delay(2); 
    } 
  } 
    
  // Find average of each LDR's read values
  for (int cm = ___; cm < ___; cm = ___) { // TODO: Replace "___", and loop over all of the LDRs
    Mn[cm] = round(LDRf[cm] / (float)numMeas); // take average of each LDR's readings
    LDRf[cm] = 0.; // reset the array holding the sum of the LDR's readings
  }
 
 // Wait to move from White to Black Surface
  for (int calii = 0; calii < 10; calii++) { // wait ten blinks for the cart to be repositioned
    slowBlink();
  }
 
  // Black Calibration
  for (int calii = ___; calii < ___; calii = ___) { // TODO: Replace "___", and repeat the section of code
                                                    //  for the number of calibration measurements desired

    fastBlink(); // blink the indicator LED

    for (int ci = ___; ci < ___; ci = ____) { // TODO: Replace "___", and loop over all of the LDRs
      LDRf[ci] = LDRf[ci] + (float) analogRead(LDR_Pin[ci]); // keep track of the sum of each LDR's readings
      delay(2); 
    } 
  }  
  
  // Find average of each LDR's read values
  for (int cm = ___; cm < ___; cm = ___) { // TODO: Replace "___", and loop over all of the LDRs
    Mx[cm] = round(LDRf[cm] / (float)numMeas); // take average of each LDR's readings
    LDRf[cm] = 0.; // reset the array holding the sum of the LDR's readings
  }
} // end Calibrate()

// ************************************************************************************************* //
// function to read and map values from potentiometers
void ReadPotentiometers() {
  SpRead = map(analogRead(S_pin), 0, 1023, 0, speed);
  kPRead = map(analogRead(P_pin), 0, 1023, 0, proportion);
  kIRead = map(analogRead(I_pin), 0, 1023, 0, integral);
  kDRead = map(analogRead(D_pin), 0, 1023, 0, derivative);
} // end ReadPotentiometers()
 
// ************************************************************************************************* //
// function to start motors using nominal speed + speed addition from potentiometer
void RunMotors() { 
  M1SpeedtoMotor = min(M1Sp + SpRead + M1P, 255); // limits speed to 255 
  M2SpeedtoMotor = min(M2Sp + SpRead + M2P, 255); // remember M1Sp & M2Sp is defined at beginning of code (default 60)
  
  Motor1->setSpeed(abs(M1SpeedtoMotor)); 
  Motor2->setSpeed(abs(M2SpeedtoMotor));
  
  // Motor 1 control
  if (M1SpeedtoMotor > 0) {
    Motor1->run(FORWARD);
  } else { // < 0
    Motor1->run(BACKWARD);
  }

  // Motor 2 control
  if (M2SpeedtoMotor > 0) {
    Motor2->run(FORWARD);
  } else { // < 0 
    Motor2->run(BACKWARD);
  }
} // end RunMotors()

// ************************************************************************************************* //
// function to read photo resistors, map from 0 to 100, and find darkest photo resitor (MxIndex)
void ReadPhotoResistors() {
  for (int Li = ___; Li < ___; Li = ___) { // TODO: Replace "___", and loop over all of the LDRs
    LDR[Li] = map(analogRead(LDR_Pin[Li]), Mn[Li], Mx[Li], 0, 100);
    delay(2); 
  }
} // end ReadPhotoResistors()

// ************************************************************************************************* //
// Calculate error from photoresistor readings
//  Do not worry about understanding or completing this function
void CalcError() {
  MxRead = -99; // initialize max read to an impossible value to ensure initialization does not impact functionality
  AveRead = 0.0; // initialize the average read

  for (int i = 0; i < 7; i = i + 1) { // loop over all of the LDRs
    if (MxRead < LDR[i]) { // if LDR value is greater than current max
      MxRead = LDR[i]; // set max equal to LDR value
      MxIndex = -1 * (i-3);
      im1 = (float)i;
    }
    AveRead = AveRead + (float)LDR[i]/7.;
  } 
  CriteriaForMax = 2; // max should be at least twice as big as the other values 
  if (MxRead > CriteriaForMax*AveRead) {
    if (im1 != 0 && im1 != 6) {
      im0 = im1 - 1;
      im2 = im1 + 1;
      WeightedAve = ((float)(LDR[im0] * im0 + LDR[im1]*im1 + LDR[im2] * im2))/((float)(LDR[im0] + LDR[im1] + LDR[im2]));
      error = -1 * (WeightedAve - 3);
    }
    else if (im1 == 0) {
      im2 = im1 + 1;
      WeightedAve = ((float)(LDR[im1]*im1 + LDR[im2]*im2))/((float)(LDR[im1]+LDR[im2]));
      error = -1 * (WeightedAve - 3);
    }
    else if (im1 == 6) {
      im0 = im1 - 1;
      WeightedAve = ((float)(LDR[im0]*im0 + LDR[im1]*im1))/((float)(LDR[im0]+LDR[im1]));
      error = -1 * (WeightedAve - 3);
    } 
  } 

  if (isnan(error)) {
    error = lasterror;
  } else {
    error = error;
  }
} // end CalcError()

// ************************************************************************************************* //
// function to make a turn (a basic P controller)
void PID_Turn() {
  // Read values are scaled PID constants from potentiometers
  kP = (float)kPRead; 
  kI = (float)kIRead; 
  kD = (float)kDRead;

  // error holds values from -3 to 3
  Turn = error * kP + sumerror * kI + (error - lasterror) * kD; //PID!!!!!
  
  sumerror = sumerror + error;

  // prevents integrator wind-up
  if (sumerror > 5) {
    sumerror = 5; 
  } else if (sumerror < -5) {
    sumerror = -5;
  }
  
  lasterror = error;
  
  // One motor becomes slower and the other faster, to "turn"
  if (Turn < 0) { // turn in direction of M2
    M1P = Turn; 
    M2P = -Turn;
  } else if (Turn > 0) { // turn in direction of M1
    M1P = Turn; 
    M2P = -Turn;
  } else { // continue in straight line
    M1P = 0; 
    M2P = 0;
  } 
} // end PID_Turn()

// ************************************************************************************************* //
// function to print values of interest
void Print() {
  Serial.print(Turn); Serial.print(" ");
  Serial.print(error); Serial.print(" ");
  Serial.print(sumerror); Serial.print(" ");
  Serial.print(lasterror); Serial.print(" ");
  
  Serial.print("   |   "); // create a visual divider
  
  Serial.print(SpRead); Serial.print(" "); // Initial Speed addition from potentiometer
  Serial.print(kP); Serial.print(" "); // PID values from potentiometers after scaling 
  Serial.print(kI); Serial.print(" ");
  Serial.print(kD); Serial.print(" ");

  Serial.print("   |   "); // create a visual divider
  
  Serial.print(LDR[0]); Serial.print(" "); // Each photo resistor value is shown
  Serial.print(LDR[1]); Serial.print(" ");
  Serial.print(LDR[2]); Serial.print(" ");
  Serial.print(LDR[3]); Serial.print(" ");
  Serial.print(LDR[4]); Serial.print(" ");
  Serial.print(LDR[5]); Serial.print(" ");
  Serial.print(LDR[6]); Serial.print(" "); 

  Serial.print("   |   "); // create a visual divider
  
  Serial.print(MxRead); Serial.print(" "); // the maximum value from the photo resistors is shown again
  Serial.print(MxIndex);Serial.print(" "); // this is the index of that maximum (0 through 6) (aka which element in LDR)
  Serial.print(error); Serial.print(" "); // this will show the calculated error (-3 through 3) 

  Serial.print("   |   "); // create a visual divider
  
  Serial.print(M1SpeedtoMotor); Serial.print(" "); // This prints the arduino output to each motor so you can see what the values (0-255)
  Serial.println(M2SpeedtoMotor); // that are sent to the motors would be without actually needing to power/run the motors
  
  delay(200); //just here to slow down the output for easier reading if wanted 
                 // ensure delay is commented when actually running your robot or this will slow down sampling too much
                 // and prevent the cart from functioning well
} // end Print()
