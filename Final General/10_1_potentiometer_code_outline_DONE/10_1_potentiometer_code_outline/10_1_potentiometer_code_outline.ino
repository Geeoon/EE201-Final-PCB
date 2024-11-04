/* 
* Potentiometer Code Outline 
*
* TODO: Locate the commented lines with "TODO:" and 
* complete the task in the descriptions
*/

// Declare Variables for Potentiometer 
// TODO: Replace "___", and assign the pin number connected to the Arduino
  const int S_pin = A0; // proportional - analog pin 
  const int P_pin = A1; // proportional - analog pin 
  const int I_pin = A2; // integral - analog pin 
  const int D_pin = A3; // derivative - analog pin 

// Initialize coefficients as zero
  int Sp = 0; // speed gain coefficient
  int kP = 0; // proportional gain coefficient
  int kI = 0; // integral gain coefficient
  int kD = 0; // derivative gain coefficient
  
void setup() { /* Setup - runs once (when power is supplied or after reset) */
 Serial.begin(9600); // TODO: Replace "___", and input the baud rate for serial communication
}

void loop() { /* Loop - loops forever (until unpowered or reset) */
 ReadPotentiometers(); // Call on user-defined function to read Potentiometer values
 Print(); // Call on user-defined function to print values from potentiometers
}

// ************************************************************************************************* //
// function to read and map values from potentiometers
//  A map changes the potentiometer values from 0-1023 (analogRead values) to 0-100 while keeping
//    the same number of steps in between.
void ReadPotentiometers() 
{
  Sp = map(analogRead(S_pin),0,1023,0,100);
  kP = map(analogRead(P_pin),0,1023,0,100);
  kI = map(analogRead(I_pin),0,1023,0,100);
  kD = map(analogRead(D_pin),0,1023,0,100);
} 

// ************************************************************************************************* //
// function to print values of interest
void Print()
{
  Serial.print(Sp); Serial.print(" "); 
  Serial.print(kP); Serial.print(" ");
  Serial.print(kI); Serial.print(" ");
  Serial.println(kD); 
  
  delay(200); //just here to slow down the output for easier reading if desired
}