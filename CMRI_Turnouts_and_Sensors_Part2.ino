// This sketch combines servo turnout control, sensor inputs and other outputs

// Pins 0, 1 and 2 are protected since these are used for communication
// Pin 13 is the Arduino LED pin and should not be used as an input, but can be used as an output for some applications
// Pins 20 and 21 are reserved for PWM servo control

// We will set the Arduino up to behave like a piece of CMRI hardware called a SUSIC with up to 64 slots
// Each slot has either a 24 or 32 bit input/output card
// 64 x 32 bit cards gives up to 2048 input/outputs!
// However, it's best to only set up the SUSIC with the required inputs/outputs to make the process more efficient.
// We will set cards 0 and 1 to be the sensor inputs (up to 64 inputs) and cards 2 - 5  to support 128 outputs

// Include libraries
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <CMRI.h>
#include <Auto485.h>

// CMRI Settings
#define CMRI_ADDR 1 //CMRI node address in JMRI
#define DE_PIN 2

// The number of servos connected
#define numServos 1

// Define the PCA9685 board addresses
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); //setup the board address - defaults to 0x40 if not specified

// Setup serial communication
Auto485 bus(DE_PIN); // Arduino pin 2 -> MAX485 DE and RE pins

// Define CMRI connection with 64 inputs and 128 outputs
CMRI cmri(CMRI_ADDR, 64, 128, bus);

// Create tables to hold data about the servo positions
int Status[numServos]; //Create a table to hold the status of each turnout, signal, etc.
int Throw[numServos]; //Create a table to hold the throw value for each servo
int Close[numServos]; //Create a table to hold the close value for each servo

void setup() {

  // SET PINS TO INPUT OR OUTPUT

    for (int i=3; i<20; i++)  {
           pinMode(i, INPUT_PULLUP);       // define sensor shield pins 3 to 19 as inputs
        }
        
    for (int i=22; i<46; i++)  {
           pinMode(i, INPUT_PULLUP);       // define sensor shield pins 22 to 45 as inputs
        }
    
    for (int i=46; i<70; i++)  {
           pinMode(i, OUTPUT);             // define sensor shield pins 46 to 69 as outputs
        }

  //INITIALISE PCA9685 BOARDS
  
  Serial.begin(19200); //Baud rate of 19200, ensure this matches the baud rate in JMRI, using a faster rate can make processing faster but can also result in incomplete data
  bus.begin(19200);
  pwm.begin();
  pwm.setPWMFreq(50);  // This is the maximum PWM frequency

  //SET THE THROW AND CLOSE VALUES FOR EACH SERVO BASED ON THE CALIBRATION PROCESS
    //Servo connection 0 - point motor
    Throw[0] = 1015;
    Close[0] = 1800;
}

void loop(){
   cmri.process();

   // PROCESS SERVOS
   // Assume servos start on bit 0 which corresponds to output address 1001
     for (int i = 0; i < numServos; i++) {
         Status[i] = (cmri.get_bit(i));
         if (Status[i] == 1){
             pwm.writeMicroseconds(i, Throw[i]);
         }
         else {
             pwm.writeMicroseconds(i, Close[i]);
         }
     }

   // PROCESS SENSORS
   // Only include lines that are required. This reduces processing time - delete or comment out lines that are not required
   // Ensure bit address matches pin, i.e. a sensor attached to pin 17 corresponds to bit 13 (because we've skipped pins 0, 1, 2 and 13) which is address 1014 for this CMRI node

      // Do not read 0, 1 or 2
      cmri.set_bit(0, !digitalRead(3));  //Bit 0 = address 1001 in JMRI
      //cmri.set_bit(1, !digitalRead(4));  //Bit 1 = address 1002 in JMRI
      //cmri.set_bit(2, !digitalRead(5));  //Bit 2 = address 1003 in JMRI
      //etc.
      //Do not read 13
      //Do not read 20 or 21  

    // PROCESS OUTPUTS
    // Non servo outputs will start on bit 100, this will be address 1101 in CMRI, bit 101 will be 1102, etc.
    // Only include lines that are required. This reduces processing time - delete or comment out lines that are not required
    
        digitalWrite(46, cmri.get_bit(100));
      //  digitalWrite(47, cmri.get_bit(101));
      //  digitalWrite(48, cmri.get_bit(102));
      //  etc...
      //  digitalWrite(67, cmri.get_bit(121));
      //  digitalWrite(68, cmri.get_bit(122));
      //  digitalWrite(69, cmri.get_bit(123));
}
