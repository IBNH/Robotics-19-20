/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\alexa                                            */
/*    Created:      Sat Sep 07 2019                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include <cstdlib>

using namespace vex;

// A global instance of vex::brain used for printing to the V5 brain screen
vex::brain       Brain;
// A global instance of vex::competition
vex::competition Competition;

// define your global instances of motors and other devices here

vex::motor DRIVE_RF = vex::motor(PORT1, true);
vex::motor DRIVE_RB = vex::motor(PORT4, true);
vex::motor DRIVE_LF = vex::motor(PORT2, false);
vex::motor DRIVE_LB = vex::motor(PORT3, false);

vex::motor RAMP = vex::motor(PORT19, false);

vex::pwm_out Led1 = vex::pwm_out(Brain.ThreeWirePort.A);


vex::motor INTAKE_R = vex::motor(PORT6);
vex::motor INTAKE_L = vex::motor(PORT7, true);

vex::motor ARM = vex::motor(PORT5);




vex::controller controller1 = vex::controller();

//vex::accelerometer accelerometer1 = vex::accelerometer(triport::port );

int slowDrive = 1, slowInt;


const int WHEEL_CIRCUMFERENCE = 33;
//33 cm

void sleep(int i){
  vex::task::sleep(i);
}

//run a motor for a set speed
void run(motor motor1, int speed){
motor1.setVelocity(speed, rpm);
motor1.spin(fwd);
}

//hold one motor
void motorBrake(motor motor1){
motor1.stop(hold);
}

//hold two motos
void motorBrake(motor motor1, motor motor2){
  motor1.stop(hold);
  motor2.stop(hold);
}

//run a pair of motors
void run(motor motor1, motor motor2, int speed)
{
  motor1.setVelocity(speed, rpm);
  motor1.spin(fwd);
  motor2.setVelocity(speed, rpm);
  motor2.spin(fwd);
}
//drive with speed of either wheel
void drive(int left, int right){
  if(left == 0 && right == 0)
  {
    motorBrake(DRIVE_LB, DRIVE_LF);
    motorBrake(DRIVE_RB, DRIVE_RF);
  }else{
  run(DRIVE_RF, right);
  run(DRIVE_LF, left);
  run(DRIVE_LB, left);
  run(DRIVE_RB, right);
}

}


void drive(int left, int right, int time){
  drive(left, right);
  sleep(time);
  drive(0, 0);
}

void sideDrive(int speed){
  run(DRIVE_RF, speed);
  run(DRIVE_LF, -speed);

  run(DRIVE_LB, speed);
  run(DRIVE_RB, -speed);
}



void pidDrive(double d, float speed1, float speed2){
DRIVE_LB.resetRotation();
double distance = -((d / 15) * 23);

static float PID_INTEGRAL_LIMIT = 50;
static int PID_DRIVE_MAX = 127;
static int PID_DRIVE_MIN = -127;

float  encoderValueL = 0;
float  encoderValueR = 0;
float  pid_Kp = 2.0;
float  pid_Ki = 0.04;
float  pid_Kd = 0.0;

float  pidErrorL = 1000;
float  pidErrorR = 1000;
float  pidLastErrorL = 0;
float  pidLastErrorR = 0;
float  pidIntegralL = 0;
float  pidIntegralR = 0;
float  pidDerivativeL = 0;
float  pidDerivativeR = 0;
float  pidDriveL;
float  pidDriveR;

pidLastErrorL  = 0;
pidLastErrorR  = 0;
pidIntegralL   = 0;
pidIntegralR   = 0;


while(fabs(pidErrorL) > 0.03 || fabs(pidErrorR) > 0.03 ){

encoderValueL = -DRIVE_LB.rotation(vex::rotationUnits (rev));
encoderValueR = -DRIVE_RB.rotation(vex::rotationUnits (rev));


pidErrorL = encoderValueL - distance;
pidErrorR = encoderValueR - distance;


if( pid_Ki != 0 )
                {
                // If we are inside controlable window then integrate the error
                if( fabs(pidErrorL) < PID_INTEGRAL_LIMIT || fabs(pidErrorR) < PID_INTEGRAL_LIMIT){
                    pidIntegralL = pidIntegralL + pidErrorL;
                    pidIntegralR = pidIntegralR + pidErrorR;
                     } else{
                    pidIntegralL = 0;
                    pidIntegralR = 0;
                     }
                }
            else{
                pidIntegralL = 0;
                pidIntegralR = 0;
            }
            // calculate the derivative
            pidDerivativeL = pidErrorL - pidLastErrorL;
            pidLastErrorL  = pidErrorL;
            pidDerivativeR = pidErrorR - pidLastErrorR;
            pidLastErrorR  = pidErrorR;


            // calculate drive
            pidDriveL = (pid_Kp * pidErrorL) + (pid_Ki * pidIntegralL) + (pid_Kd * pidDerivativeL);
            pidDriveR = (pid_Kp * pidErrorR) + (pid_Ki * pidIntegralR) + (pid_Kd * pidDerivativeR);

            // limit drive
            if( pidDriveL > PID_DRIVE_MAX )
                pidDriveL = PID_DRIVE_MAX;
            if( pidDriveL < PID_DRIVE_MIN )
                pidDriveL = PID_DRIVE_MIN;

            if( pidDriveR > PID_DRIVE_MAX )
                pidDriveR = PID_DRIVE_MAX;
            if( pidDriveR < PID_DRIVE_MIN )
                pidDriveR = PID_DRIVE_MIN;
            


            // send to motor
            drive(pidDriveL * speed1, pidDriveR * speed2);
            }
}


void pidRamp(){
RAMP.resetRotation();
double distance = -3.0;

static float PID_INTEGRAL_LIMIT = 50;
static int PID_DRIVE_MAX = 127;
static int PID_DRIVE_MIN = -127;

float  encoderValue = 0;
float  pid_Kp = 2.0;
float  pid_Ki = 0.04;
float  pid_Kd = 0.0;

float  pidError = 1000;
float  pidLastError = 0;
float  pidIntegral = 0;
float  pidDerivative = 0;
float  pidDrive;

pidLastError  = 0;
pidIntegral   = 0;

while(fabs(pidError) > 0.03){

encoderValue = -RAMP.rotation(vex::rotationUnits (rev));


pidError = encoderValue - distance;


if( pid_Ki != 0 )
                {
                // If we are inside controlable window then integrate the error
                if( fabs(pidError) < PID_INTEGRAL_LIMIT )
                    pidIntegral = pidIntegral + pidError;
                else
                    pidIntegral = 0;
                }
            else
                pidIntegral = 0;

            // calculate the derivative
            pidDerivative = pidError - pidLastError;
            pidLastError  = pidError;

            // calculate drive
            pidDrive = (pid_Kp * pidError) + (pid_Ki * pidIntegral) + (pid_Kd * pidDerivative);

            // limit drive
            if( pidDrive > PID_DRIVE_MAX )
                pidDrive = PID_DRIVE_MAX;
            if( pidDrive < PID_DRIVE_MIN )
                pidDrive = PID_DRIVE_MIN;

            // send to motor
            run(RAMP, pidDrive );
            }
        sleep(750);
}

vex::thread calcL;

void test(void *func);

int* encoderL(int i){
  while(true){

  }
  
  return 0;
}

void calcR()
{
  while(true){
    
  }
}



void test(void *func) {

}



void* threadR(*ptr){
while(true){

  }
(*ptr) ();
}


/*
double getVelocity(){

}
*/

//easier sleep


/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the cortex has been powered on and    */ 
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton( void ) {
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
  
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous( void ) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................

std::string side = "blue";


run(RAMP, 100);
sleep(500);

run(RAMP, 0);



run(INTAKE_L, INTAKE_R, 300);
sleep(1000);
run(INTAKE_L, INTAKE_R, 0);
run(ARM, 200);
sleep(500);
run(ARM, -200);
sleep(500);

run(RAMP, -100);
sleep(500);
run(RAMP, 0);




drive(0, 0);

run(INTAKE_L, INTAKE_R, -300);
pidDrive(2.5, .6, .6);

drive(0,0);

sleep(100);

pidDrive(-2, 1.5, 1.5);


run(INTAKE_L, INTAKE_R, 0);

drive(0,0);

//THE TURNING PART
pidDrive(-1.3, 1, -1);


drive(0,0);

pidDrive(.5, 1, 1);

drive(0,0);

run(INTAKE_L, INTAKE_R, 100);

run(RAMP, 100);
sleep(500);
//run(INTAKE_L, INTAKE_R, 100);

run(INTAKE_L, INTAKE_R, 0);


sleep(2000);
run(RAMP, 0);

drive(-50, -50);

run(INTAKE_L, INTAKE_R, 0);
sleep(1000);

run(DRIVE_RF, 0);
  run(DRIVE_LF, 0);
  run(DRIVE_LB, 0);
  run(DRIVE_RB, 0);



}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol( void ) {
  // User control code here, inside the loop
   int count = 0;
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo 
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to 
    // update your motors, etc.
    // ........................................................................
Led1.state(100,vex::percentUnits::pct);



count++;
//std::string s = "Spiral1.png";
 
//std::string picString;
//picString = pic;


 
 //s.replace(6, 1, picString); 
// s.replace(7, 4, ".png");
//const char* picture = s.c_str();
if(count == 1)
  Brain.Screen.drawImageFromFile( "Spiral1.png" , 0, 0);
if(count == 2)
  Brain.Screen.drawImageFromFile( "Spiral2.png" , 0, 0);
if(count == 3)
  Brain.Screen.drawImageFromFile( "Spiral3.png" , 0, 0);
if(count == 4)
  Brain.Screen.drawImageFromFile( "Spiral4.png" , 0, 0);
if(count == 5)
  Brain.Screen.drawImageFromFile( "Spiral5.png" , 0, 0);
if(count == 6)
  Brain.Screen.drawImageFromFile( "Spiral6.png" , 0, 0);
if(count == 7)
  Brain.Screen.drawImageFromFile( "Spiral7.png" , 0, 0);
if(count == 8)
  Brain.Screen.drawImageFromFile( "Spiral8.png" , 0, 0);
if(count == 9)
  Brain.Screen.drawImageFromFile( "Spiral9.png" , 0, 0);
if(count == 10)
  Brain.Screen.drawImageFromFile( "Spiral10.png" , 0, 0);
if(count == 11)
  Brain.Screen.drawImageFromFile( "Spiral11.png" , 0, 0);
if(count == 12)
  Brain.Screen.drawImageFromFile( "Spiral12.png" , 0, 0);




  //Brain.Screen.print(picture);
  //Brain.Screen.print(s[6] + s[7] + s[8]);
 // Brain.Screen.newLine();
if(count == 13)
count = 0;

 
  run(DRIVE_RF, (controller1.Axis3.value() - controller1.Axis4.value() - controller1.Axis1.value()) * 1.5 / slowDrive);
  run(DRIVE_LB, (controller1.Axis3.value() - controller1.Axis4.value() + controller1.Axis1.value()) * 1.5 / slowDrive);

  run(DRIVE_LF, (controller1.Axis3.value() + controller1.Axis4.value() + controller1.Axis1.value()) * 1.5 / slowDrive);
  run(DRIVE_RB, (controller1.Axis3.value() + controller1.Axis4.value() - controller1.Axis1.value()) * 1.5 / slowDrive);


if(controller1.ButtonA.pressing())
{
  if(slowDrive == 1){
  slowDrive = 2;
  slowInt = 0;
  vex::task::sleep(500);
}else{
  slowDrive = 1;
  slowInt = 0;
  vex::task::sleep(500);
}
}

if(controller1.ButtonR1.pressing()){
  run(ARM, 200);
}else if(controller1.ButtonR2.pressing()){
  run(ARM, -200);
}else{
  ARM.stop(hold);
}


if(controller1.ButtonX.pressing()){
  pidRamp();
}else if(controller1.ButtonB.pressing()){
  run(RAMP, -200);
}else{
  RAMP.stop(hold);
}

if(controller1.ButtonL1.pressing()){
    run(INTAKE_L, INTAKE_R, 300);
} else if(controller1.ButtonL2.pressing()){
    run(INTAKE_L, INTAKE_R, -300 );
} else{
    INTAKE_L.stop(hold);
    INTAKE_R.stop(hold);
}




    vex::task::sleep(20); //Sleep the task for a short amount of time to prevent wasted resources. 
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
    //Set up callbacks for autonomous and driver control periods.
   //calcTask =  vex::task(encoderL());
   vex::thread thread( test(calcR()) );
  
    Competition.autonomous( autonomous );
    Competition.drivercontrol( usercontrol );
    
    //Run the pre-autonomous function. 
    pre_auton();
       
    //Prevent main from exiting with an infinite loop.                        
    while(1) {
      vex::task::sleep(100);//Sleep the task for a short amount of time to prevent wasted resources.
    }    
       
}
