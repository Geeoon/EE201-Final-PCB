#include <Wire.h>
#include <Adafruit_MotorShield.h> // Must add libary - see MotorShield Manual

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Motors can be switched here (1) <--> (2)
Adafruit_DCMotor *Motor1 = AFMS.getMotor(1); // left motor
Adafruit_DCMotor *Motor2 = AFMS.getMotor(2); // right motor

// Photoresistor related globals
int LDR_Pin[7] = {8,
                  9,
                  10,
                  11,
                  12,
                  13,
                  14}; 
int LDR[7];
float Mn[7] = {
  388.00,
  312.00,
  313.00,
  311.00,
  285.00,
  321.00,
  327.00
}; // array containing minimum read values of every potentiometer
float Mx[7] {
  684.00,
  597.00,
  597.00,
  644.00,
  580.00,
  606.00,
  589.00
}; // array containing maximum read values of every potentiometer
float LDRf[7] = {0.,0.,0.,0.,0.,0.,0.}; // initialize the LDR read values to float zero
float WeightedAve; // weighted average after calibration
float error; // final error

// Photoresistor related methods
// function to read photo resistors, map from 0 to 100
void ReadPhotoResistors();
// Calculate error from photoresistor readings
float CalcError();

// PID and pot related globals
float max_speed = 255.0;
float max_proportion = 200.0;
float max_integral = 20.0;
float max_derivative = 200.0;
float M1SpeedtoMotor, M2SpeedtoMotor; // actual speeds motor will be rotating at
float Turn = 0; // which direction and how sharply should the cart turn
int minimum_speed_1 = 35;
int minimum_speed_2 = 35;
float SpRead = 0.0; //Speed Increase 
float kPRead = 0.0; //proportional gain
float kIRead = 0.0; //integral gain
float kDRead = 0.0; //derivative gain
float M1P = 0.0, M2P = 0.0; // proportion control for motors 1 and 2 respectively
float lasterror = 0.0; // previous error  D
float sumerror = 0.0; // sum of total errors  I
float max_error = 5;
float kP, kI, kD; // final values of P, I, and D used in control
int S_PIN = 0, P_PIN = 1, I_PIN = 2, D_PIN = 3;
// PID and Pot related methods
// function to make a turn (a basic P controller)
void PID_Turn();
// function to start motors using nominal speed + speed addition from potentiometer
void RunMotors();
// function to read and map values from potentiometers
void ReadPotentiometers();

void setup() {
  Serial.begin(9600);
  AFMS.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  ReadPotentiometers();

  ReadPhotoResistors();
  error = CalcError();
  // Serial.println();
  // Serial.print(SpRead); Serial.print(" ");
  // Serial.print(kPRead); Serial.print(" ");
  // Serial.print(kIRead); Serial.print(" ");
  // Serial.print(kDRead); Serial.print(" ");
  // delay(500);
  PID_Turn();
  SpRead /= ((abs(Turn) * .35) + 1);  // slow down when turning
  Serial.print(Turn); Serial.print(" | ");
  Serial.println(SpRead);
  // actual speed cap
  SpRead = min(SpRead, 150);
  RunMotors();
}

// function to read photo resistors, map from 0 to 100
void ReadPhotoResistors() {
  for (int Li = 0; Li < 7; Li++) {
    LDR[Li] = map(analogRead(LDR_Pin[Li]), Mn[Li], Mx[Li], 0, 100);
    delay(2); 
  }
}

float CalcError() {
  int MxRead = -99; // initialize max read to an impossible value to ensure initialization does not impact functionality
  float AveRead = 0.0; // initialize the average read
  int im0, im1, im2; // error calculation variables

  for (int ii = 0; ii < 7; ii = ii + 1) { // loop over all of the LDRs
    if (MxRead < LDR[ii]) { // if LDR value is greater than current max
      MxRead = LDR[ii]; // set max equal to LDR value
      im1 = (float)ii;
    }

    AveRead = AveRead + (float)LDR[ii] / 7.; // update the average
  }
  
  int CriteriaForMax = 2; // max should be at least twice as big as the other values 
  float WeightedAve = 0.0;
  float error = 0.0;
  if (MxRead > CriteriaForMax * AveRead) {
    if (im1!=0 && im1!=6) {
      im0 = im1 - 1;
      im2 = im1 + 1;
      WeightedAve = ((float)(LDR[im0]*im0 + LDR[im1]*im1 + LDR[im2]*im2))/((float)(LDR[im0]+LDR[im1]+LDR[im2]));
      error = -1.0 * (WeightedAve - 3.0);

    } else if (im1 == 0) {
      im2 = im1 + 1;
      WeightedAve = ((float)(LDR[im1]*im1 + LDR[im2]*im2))/((float)(LDR[im1]+LDR[im2]));
      error = -1.0 * (WeightedAve - 3.0);

    } else if (im1 == 6) {
      im0 = im1-1;
      WeightedAve = ((float)(LDR[im0]*im0 + LDR[im1]*im1))/((float)(LDR[im0]+LDR[im1]));
      error = -1.0 * (WeightedAve - 3.0);
    }
  }
  if (isnan(error)) {
    error = lasterror;
  }
  if (abs(error - lasterror) > 4) {  // sudden changes shouldn't happen, it's probably noise
    error = lasterror;
  }
  return error;
}

void PID_Turn() {
  // Read values are scaled PID constants from potentiometers
  kP = kPRead; 
  kI = kIRead; 
  kD = kDRead;

  // error holds values from -3 to 3
  Turn = error * kP + sumerror * kI + (error - lasterror) * kD; //PID!!!!!
  
  sumerror = sumerror + error;

  // prevents integrator wind-up
  if (sumerror > max_error) {
    sumerror = max_error;
  } else if (sumerror < -max_error) {
    sumerror = -max_error;
  }
  
  lasterror = error;  
  M1P = -Turn; 
  M2P = Turn;
}

// function to start motors using nominal speed + speed addition from potentiometer
void RunMotors() { 
  M1SpeedtoMotor = min((minimum_speed_1 + SpRead + M1P) * 1.0, 255); // limits speed to 255 
  M2SpeedtoMotor = min(minimum_speed_2 + SpRead + M2P, 255); // remember M1Sp & M2Sp is defined at beginning of code (default 60)
  
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
}

void ReadPotentiometers() {
  SpRead = mapf((float)analogRead(S_PIN), 0.0, 1023, 0.0, max_speed);
  kPRead = mapf((float)analogRead(P_PIN), 0.0, 1023, 0.0, max_proportion);
  kIRead = mapf((float)analogRead(I_PIN), 0.0, 1023, 0.0, max_integral);
  kDRead = mapf((float)analogRead(D_PIN), 0.0, 1023, 0.0, max_derivative);
}
