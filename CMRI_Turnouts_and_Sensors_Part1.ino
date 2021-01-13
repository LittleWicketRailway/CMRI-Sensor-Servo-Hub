// This sketch combines servo turnout control and sensor inputs

// Pins 0, 1 and 2 are protected since these are used for communication
// Pin 13 is the Arduino LED pin and should not be used as an input, but can be used as an output for some applications
// Pins 20 and 21 are reserved for PWM servo control
// 64 pins will be used for sensor inputs

// We will set the Arduino up to behave like a piece of CMRI hardware called a SUSIC with up to 64 slots
// Each slot has either a 24 or 32 bit input/output card
// 64 x 32 bit cards gives up to 2048 input/outputs!
// However, it's best to only set up the SUSIC with the required inputs/outputs to make the process more efficient.
// We will set cards 0 and 1 to be the sensor inputs (up to 64 inputs) and card 2 to support 32 servo outputs

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

// Define CMRI connection with 64 inputs and 32 outputs
CMRI cmri(CMRI_ADDR, 64, 32, bus);

// Create tables to hold data about the servo positions
int Status[numServos]; //Create a table to hold the status of each turnout, signal, etc.
int Throw[numServos]; //Create a table to hold the throw value for each servo
int Close[numServos]; //Create a table to hold the close value for each servo

void setup() {

  // Set pins 3-19 and 22-69 as input pins for sensors

    for (int i=3; i<=19; i++)  {
           pinMode(i, INPUT_PULLUP);       // set sensor shield pins 3-19 as inputs
        }
        
    for (int i=22; i<=69; i++)  {
           pinMode(i, INPUT_PULLUP);       // set sensor shield pins 22-69 as inputs
        }
        
  // Start the serial connection
  Serial.begin(19200);  //Baud rate of 19200, ensure this matches the baud rate in JMRI, using a faster rate can make processing faster but can also result in incomplete data
  bus.begin(19200);
  
  // Initialise PCA9685 board
  pwm.begin();
  pwm.setPWMFreq(50);  // This is the maximum PWM frequency

  // SET THE THROW AND CLOSE VALUES FOR EACH SERVO BASED ON THE CALIBRATION PROCESS
  
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
}
