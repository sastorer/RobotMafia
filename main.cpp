#include "meArm.h"
#include "Servo.h"
#include "MMA8452.h"
#include "mbed.h"

meArm arm;
Serial pc(USBTX,USBRX);
 
MMA8452 mma(p28, p27, 100000);
int x, y, z;
 
//Change these values if accelerometer reading are different:
//How far the accerometer is tilted before starting to move the arm:
const int MovementThreshold = 18;
 
//The average zero acceleration values read from the accelerometer for each axis:
const int ZeroXValue = 0;
const int ZeroYValue = 0;
const int ZeroZValue = 0;
 
//The maximum (positive) acceleration values read from the accelerometer for each axis:
const int MaxXValue = 4096;
const int MaxYValue = 4096;
const int MaxZValue = 4096;
 
//The minimum (negative) acceleration values read from the accelerometer for each axis:
const int MinXValue = -4096;
const int MinYValue = -4096;
const int MinZValue = -4096;
 
//The sign of the arm movement relative to the acceleration.
//If arm is going in the opposite direction you think it should go, change the sign for the appropriate axis.
const int XSign = 1;
const int YSign = 1;
const int ZSign = 1;
 
//The maximum speed in each axis (x and y)
//that the arm should move. Set this to a higher or lower number if the arm does not move fast enough or is too fast.
const int MaxArmMovement = 50;  
 
//This reduces the 'twitchiness' of the cursor by calling a delay function at the end of the main loop.
//There are better way to do this without delaying the whole microcontroller, but that is left for another tutorial or project.
const int ArmDelay = .001;

//Function to process the acclerometer data
//and send mouse movement information via USB
void processAccelerometer(int16_t XReading, int16_t YReading, int16_t ZReading)
{
  //Initialize values for the mouse cursor movement.
  int16_t ArmXMovement = 0;
  int16_t ArmYMovement = 0;
  int16_t ArmZMovement = 0;
  
  //Calculate mouse movement
  //If the analog X reading is ouside of the zero threshold...
  if( MovementThreshold < abs( XReading - ZeroXValue ) ){
    //...calculate X mouse movement based on how far the X acceleration is from its zero value.
    ArmXMovement = XSign * ( ( ( (float)( 2 * MaxArmMovement ) / ( MaxXValue - MinXValue ) ) * ( XReading - MinXValue ) ) - MaxArmMovement );
    //it could use some improvement, like making it trigonometric.
  }  else  {
    //Within the zero threshold, the cursor does not move in the X.
    ArmXMovement = 0;
  }
 
  //If the analog Y reading is ouside of the zero threshold... 
  if( MovementThreshold < abs( YReading - ZeroYValue ) ){
    //...calculate Y mouse movement based on how far the Y acceleration is from its zero value.
    ArmYMovement = YSign * ( ( ( (float)( 2 * MaxArmMovement ) / ( MaxYValue - MinYValue ) ) * ( YReading - MinYValue ) ) - MaxArmMovement );
    //it could use some improvement, like making it trigonometric.
  }  else  {
    //Within the zero threshold, the cursor does not move in the Y.
    ArmYMovement = 0;
  }

    //Calculate mouse movement
  //If the analog Z reading is ouside of the zero threshold...
  if( MovementThreshold < abs( ZReading - ZeroZValue ) )
  {
    //...calculate Z mouse movement based on how far the Z acceleration is from its zero value.
    ArmZMovement = ZSign * ( ( ( (float)( 2 * MaxArmMovement ) / ( MaxZValue - MinZValue ) ) * ( ZReading - MinZValue ) ) - MaxArmMovement );
    //it could use some improvement, like making it trigonometric.
  }
  else
  {
    //Within the zero threshold, the cursor does not move in the X.
    ArmZMovement = 0;
  }
  arm.gotoPoint(ArmXMovement, ArmYMovement, ArmZMovement);  // otherwise just move mouse
}
  
int main() { 
  mma.readXYZCounts(&x, &y, &z);    // get an initial read   
  arm.begin();               // Initialize arm

  while(1) {
    mma.readXYZCounts(&x, &y, &z);  //   // Read the 'raw' data in 14-bit counts   
    processAccelerometer(x,y,z);  // Work with the read data    
    wait(ArmDelay);  // wait until next reading - was 500 in Adafruit example
  }
}