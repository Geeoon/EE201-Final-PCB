/* Calibration Code 
*
* * TODO: Locate the commented lines with "TODO:" and 
* complete the task in the descriptions
*/

// Variables for Light Sensors (arrays are used to simplify the code):
//  these are variables that have multiple elements to each variable name.
//  LDR_Pin holds 7 values of the LDR pin connection to the Arduino

// TODO: Replace "___", and assign the pin number for each LDR connected to the Arduino
//  reference 10.2 potentiometer code
int LDR_Pin[7] = {8,
                  9,
                  10,
                  11,
                  12,
                  13,
                  14}; 
int LDR[7]; // this is an empty array that will hold the readings from our potentiometers

// TODO: Replace "___", and assign the pin number connected to the Arduino.
//  it is recommended to use pin 13, but can change to another digital pin 
//  and connect extra LED to me more easily seen
int led_Pin = 13; 

// Calibration Variables
//  Complete understanding of the calibration process is not required 
float Mn[7]; // array containing minimum read values of every potentiometer
float Mx[7]; // array containing maximum read values of every potentiometer
float LDRf[7] = {0.,0.,0.,0.,0.,0.,0.}; // initialize the LDR read values to float zero

int numMeas = 40; // number of measurements used in calibration (CAN BE EDITED BY USER)

int MxRead; // maximum read value by any potentiometer
int MxIndex; // index (location in array) of max read value
float AveRead; // average read value of potentiometer
int CriteriaForMax; // value to determine whether a read value is a max
float WeightedAve; // weighted average after calibration

int im0,im1,im2; // error calculation variables
float error; // final error
 
// ************************************************************************************************* //
// setup - runs once
void setup() {
  Serial.begin(9600); // TODO: Replace "___", and input the baud rate for serial communication
  pinMode(led_Pin, OUTPUT); // TODO: Replace "___", and set the led_pin to be an output
  
  Calibrate(); // Calibrate black and white sensing
 
} // end setup()

// ************************************************************************************************* //
// loop - runs/loops forever
void loop() { 
  ReadPhotoResistors(); // Read photoresistors and map to 0-100 based on calibration

  CalcError();
  
  Print(); // Print values to serial monitor. currently commented out but could be good for debugging :)
} // end loop()

// ************************************************************************************************* //
// function to quickly blink an LED
void fastBlink() {
  digitalWrite(led_Pin, HIGH); // TODO: Replace "___", and turn on the LED
  delay(100); // wait for 100 milliseconds

  digitalWrite(led_Pin, LOW); // TODO: Replace "___", and turn off the LED
  delay(100); // wait for 100 milliseconds
} // end fastBlink()

// ************************************************************************************************* //
// function to slowly blink an LED
void slowBlink() {
  digitalWrite(led_Pin, HIGH); // TODO: Replace "___", and turn on the LED
  delay(100); // wait for 100 milliseconds

  digitalWrite(led_Pin, LOW); // TODO: Replace "___", and turn off the LED
  delay(900); // wait for 900 milliseconds
} // end slowBlink()

// ************************************************************************************************* //
// function to calibrate
void Calibrate() {
  Serial.println("Move to White"); 
  // wait to make sure cart is in position with sensors over a white (light) surface
  for (int calii = 0; calii < 4; calii++) { // wait four blinks for the cart to be positioned
    slowBlink(); // blink the indicator LED
  }
  Serial.println("Starting calibration");
  // Calibration
  // White Calibration
  for (int calii = 0; calii < numMeas; calii++) { // TODO: Replace "___", and repeat the section of code
                                                    //  for the number of calibration measurements desired

    fastBlink(); // blink the indicator LED

    for (int ci = 0; ci < 7; ci++) { // TODO: Replace "___", and loop over all of the LDRs
      LDRf[ci] = LDRf[ci] + (float) analogRead(LDR_Pin[ci]); // keep track of the sum of each LDR's readings
      delay(2); 
    } 
  } 
    
  // Find average of each LDR's read values
  for (int cm = 0; cm < 7; cm++) { // TODO: Replace "___", and loop over all of the LDRs
    Mn[cm] = round(LDRf[cm] / (float)numMeas); // take average of each LDR's readings
    LDRf[cm] = 0.; // reset the array holding the sum of the LDR's readings
  }
 
  Serial.println("Move to black surface");
 // Wait to move from White to Black Surface
  for (int calii = 0; calii < 10; calii++) { // wait ten blinks for the cart to be repositioned
    slowBlink();
  }
  Serial.println("Starting calibration");
 
  // Black Calibration
  for (int calii = 0; calii < numMeas; calii++) { // TODO: Replace "___", and repeat the section of code
                                                    //  for the number of calibration measurements desired
    fastBlink(); // blink the indicator LED

    for (int ci = 0; ci < 7; ci++) { // TODO: Replace "___", and loop over all of the LDRs
      LDRf[ci] = LDRf[ci] + (float) analogRead(LDR_Pin[ci]); // keep track of the sum of each LDR's readings
      delay(2); 
    } 
  }  
  
  // Find average of each LDR's read values
  for (int cm = 0; cm < 7; cm++) { // TODO: Replace "___", and loop over all of the LDRs
    Mx[cm] = round(LDRf[cm] / (float)numMeas); // take average of each LDR's readings
    LDRf[cm] = 0.; // reset the array holding the sum of the LDR's readings
  }
  Serial.println("End of calibration");
} // end Calibrate()

// ************************************************************************************************* //
// function to read photo resistors, map from 0 to 100, and find darkest photo resitor (MxIndex)
void ReadPhotoResistors() {
  for (int Li = 0; Li < 7; Li++) { // TODO: Replace "___", and loop over all of the LDRs
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

  for (int ii = 0; ii < 7; ii = ii + 1) { // loop over all of the LDRs
    if (MxRead < LDR[ii]) { // if LDR value is greater than current max
      MxRead = LDR[ii]; // set max equal to LDR value
      MxIndex = -1 * (ii - 3); 
      im1 = (float)ii;
    }

    AveRead = AveRead + (float)LDR[ii] / 7.; // update the average
  }
  
  CriteriaForMax = 2; // max should be at least twice as big as the other values 
  if (MxRead > CriteriaForMax * AveRead) {
    if (im1!=0 && im1!=6) {
      im0 = im1 - 1;
      im2 = im1 + 1;
      WeightedAve = ((float)(LDR[im0]*im0 + LDR[im1]*im1 + LDR[im2]*im2))/((float)(LDR[im0]+LDR[im1]+LDR[im2]));
      error = -1 * (WeightedAve - 3);

    } else if (im1 == 0) {
      im2 = im1 + 1;
      WeightedAve = ((float)(LDR[im1]*im1 + LDR[im2]*im2))/((float)(LDR[im1]+LDR[im2]));
      error = -1 * (WeightedAve - 3);

    } else if (im1 == 6) {
      im0 = im1-1;
      WeightedAve = ((float)(LDR[im0]*im0 + LDR[im1]*im1))/((float)(LDR[im0]+LDR[im1]));
      error = -1 * (WeightedAve - 3);
    } 
  } 
} // end CalcError()

// ************************************************************************************************* //
// function to print values of interest
void Print() {
 // Each photo resistor value is shown
  Serial.print(LDR[0]); Serial.print(" "); 
  Serial.print(LDR[1]); Serial.print(" ");
  Serial.print(LDR[2]); Serial.print(" ");
  Serial.print(LDR[3]); Serial.print(" ");
  Serial.print(LDR[4]); Serial.print(" ");
  Serial.print(LDR[5]); Serial.print(" ");
  Serial.print(LDR[6]); Serial.print(" "); 
  
  Serial.print("   |   "); // create a visual divider between the LDR readings and other information
  Serial.print(MxRead); Serial.print(" "); // the maximum value from the photo resistors is shown again
  Serial.print(MxIndex);Serial.print(" "); // this is the index of that maximum (0 through 6) (aka which element in LDR)
  Serial.println(error); // this will show the calculated error (-3 through 3) 
  
  Serial.println("Min Max (0-7)");
  for (int Li = 0; Li < 7; Li++) { // TODO: Replace "___", and loop over all of the LDRs
    Serial.print(Mn[Li]); Serial.print(" ");
    Serial.print(Mx[Li]); Serial.print(" ");
    delay(2);
    Serial.print(" | ");
  }
  delay(200); //just here to slow down the output for easier reading if wanted 
 
} // end Print()