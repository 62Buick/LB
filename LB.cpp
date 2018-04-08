/* Revised 180604.001
To compile use:
g++ -Wall -lpthread -o LB LB.cpp -lpigpio -lrt -std=c++14

This program is designed to operate an automated trailer loader and unloader.  The goal is to cue pallets for loading
between trucks and mow the yard with no measley human interaction.

60' extendable arm to reach into the 53' trailer.
Need to adjust for trailer slope orientation and movement.
Lift head will need to pivot left and right 90 degrees.  Tilt forward and back 10 degrees.  Extend 10', shift 3' left and right
Cameras for left, right, top, bottom on head.
Camera on power unit.
Rechargable power.
4000 lb lifting capacity.

Error History

If the files do not close correctly it may leave a pigpio.pid file open that will prevent the pigpio from initializing the next time it runs
Fix it by issuing a "sudo killall pigpiod" command.

*/

#include <iostream>
#include <pigpio.h>
#include <ctime>
#include <ratio>
#include <chrono> // For time controls.

using namespace std;
using namespace std::chrono;

// Initialize Control and set to nominal position.
    int mower_on = 0;  //GPIO17
    int move_on = 0;  //GPIO18
    int forward_reverse = 1; //GPIO27
    int direction = 0;// GPIO23 Uses azmuith relative to the orientation of the mower 0 is straight ahead, 90 is right, 270 is left Range is 0 - 360. 
    int safety = 0;
    int left_detect = 0;  //GPIO16
    int right_detect = 0;  //GPIO20
    int front_detect = 0; //GPIO12
    int rear_detect = 0;  //GPIO21
    int azimuth;
    int currentposition = 1500;
    int kbnumber;
    char kbchar;
    steady_clock::time_point time_boot;
    steady_clock::time_point time_current;
    steady_clock::time_point time_last_locate;
    steady_clock::time_point time_last_picture;
    steady_clock::time_point time_last_mowing;

// Initialize PIGPIO.
  gpioInitialise ();
  gpioSetMode (17,PI_OUTPUT); // Mower control - GPIO17
  gpioSetMode (18,PI_OUTPUT); // Drive Motor Power control - GPIO18 ** Currently On - Off may want to use a Variable Drive
  gpioSetMode (27,PI_OUTPUT); // Drive Motor Forward/Reverse - GPIO27  
  gpioSetMode (12,PI_INPUT); // Front Detect Pin Number -  Normally Open switch to 3V on GPIO12 
  gpioSetMode (16,PI_INPUT); // Left Detect Pin Number - Normally Open switch to 3V on GPIO16
  gpioSetMode (20,PI_INPUT); // Right Detect Pin Number - Normally Open switch to 3V on GPIO20
  gpioSetMode (21,PI_INPUT); // Rear Detect Pin Number - Normally Open switch to 3V on GPIO21
  gpioSetPullUpDown (12,PI_PUD_DOWN);
  gpioSetPullUpDown (16,PI_PUD_DOWN);
  gpioSetPullUpDown (20,PI_PUD_DOWN);
  gpioSetPullUpDown (21,PI_PUD_DOWN);
  gpioPWM (23,128); //Activate and set steering servo plugged into pin 23 to 0
  gpioServo (23, 1500); 

// Takes a picture and texts it to the number in the .sh file.
int pic01 (void)
{
	system ("bash ./Picture_with_email.sh");
	return (0);
}

// Takes a picture and saves it.
int pic02 (void)
{
	system ("bash ./Picture.sh");
	return (0);
}

int selftest (void)
{
  gpioServo (23, 1500);//Centers the steering
  gpioSleep(PI_TIME_RELATIVE, 0, 200000); // sleep for 0.2 seconds
  gpioWrite (27, 1); //Set the drive to forward.
  gpioSleep(PI_TIME_RELATIVE, 0, 200000); // sleep for 0.2 seconds
  gpioWrite (18, 1); //Turn move on
  gpioSleep(PI_TIME_RELATIVE, 0, 200000); // sleep for 0.2 seconds
  gpioWrite (18, 0); //Shut the move off.
  gpioSleep(PI_TIME_RELATIVE, 0, 200000); // sleep for 0.2 seconds
  gpioWrite (27, 0); //Set the drive to reverse.
  gpioSleep(PI_TIME_RELATIVE, 0, 200000); // sleep for 0.2 seconds  
  gpioWrite (18, 1); //Turn move on
  gpioSleep(PI_TIME_RELATIVE, 0, 200000); // sleep for 0.2 seconds
  gpioWrite (18, 0); //Turn move off.
  gpioSleep(PI_TIME_RELATIVE, 0, 200000); // sleep for 0.2 seconds
  gpioServo (23, 1600);//Steers Right
  gpioSleep(PI_TIME_RELATIVE, 0, 200000); // sleep for 0.2 seconds
  gpioWrite (27, 1); //Set the drive to forward.
  gpioSleep(PI_TIME_RELATIVE, 0, 200000); // sleep for 0.2 seconds
  gpioWrite (18, 1); //Turn move on
  gpioSleep(PI_TIME_RELATIVE, 0, 200000); // sleep for 0.2 seconds
  gpioWrite (18, 0); //Shut the move off.
  gpioSleep(PI_TIME_RELATIVE, 0, 200000); // sleep for 0.2 seconds
  gpioWrite (27, 0); //Set the drive to reverse.
  gpioSleep(PI_TIME_RELATIVE, 0, 200000); // sleep for 0.2 seconds  
  gpioWrite (18, 1); //Turn move on
  gpioSleep(PI_TIME_RELATIVE, 0, 200000); // sleep for 0.2 seconds
  gpioWrite (18, 0); //Turn move off.
  gpioSleep(PI_TIME_RELATIVE, 0, 200000); // sleep for 0.2 seconds
  gpioServo (23, 1400);//Steers Left
  gpioSleep(PI_TIME_RELATIVE, 0, 200000); // sleep for 0.2 seconds
  gpioWrite (27, 1); //Set the drive to forward.
  gpioSleep(PI_TIME_RELATIVE, 0, 200000); // sleep for 0.2 seconds
  gpioWrite (18, 1); //Turn move on
  gpioSleep(PI_TIME_RELATIVE, 0, 200000); // sleep for 0.2 seconds
  gpioWrite (18, 0); //Shut the move off.
  gpioSleep(PI_TIME_RELATIVE, 0, 200000); // sleep for 0.2 seconds
  gpioWrite (27, 0); //Set the drive to reverse.
  gpioSleep(PI_TIME_RELATIVE, 0, 200000); // sleep for 0.2 seconds  
  gpioWrite (18, 1); //Turn move on
  gpioSleep(PI_TIME_RELATIVE, 0, 200000); // sleep for 0.2 seconds
  gpioWrite (18, 0); //Turn move off.
  gpioSleep(PI_TIME_RELATIVE, 0, 200000); // sleep for 0.2 seconds
  gpioWrite (17, 1); //Turn the mower on.
  gpioSleep(PI_TIME_RELATIVE, 0, 200000); // sleep for 0.2 seconds
  gpioWrite (17, 0); //Turn the mower off.
  gpioSleep(PI_TIME_RELATIVE, 0, 200000); // sleep for 0.2 seconds
  gpioServo (23, 1500);//Centers the steering
  gpioSleep(PI_TIME_RELATIVE, 0, 200000); // sleep for 0.2 seconds
  pic01 ();
  return (0);
}

int readkey (void)
{
	cin >> kbchar;
	kbnumber = kbchar;
	return (0);
}


// Reads Sensors and Updates Controls
int controlupdate (void)
{
        mower_on = 1;
        move_on = 1;
	forward_reverse = 1;
        left_detect = gpioRead(16);
        right_detect = gpioRead(20);  
        rear_detect = gpioRead(21);  
        front_detect = gpioRead(12);
	cout << "Front Left Right Rear" << endl;
	cout << front_detect << "     " << left_detect << "    " << right_detect << "     " << rear_detect << endl;
        if(left_detect == 1) {mower_on = 0; direction = (direction + 10);}
        if(right_detect == 1) {mower_on = 0; direction = (direction - 10);}
        if(left_detect == 0 && right_detect == 0) {direction = 0;}
        if(front_detect == 1) {mower_on = 0;forward_reverse = 0;direction = (direction - 10);} 
        if(rear_detect == 1) {mower_on = 0;safety = 1;pic01 ();}
        if(direction > 360) {direction = (direction - 360);}
        if(direction < 0) {direction = (360 + direction);}
        if(mower_on == 1) {gpioWrite (17, 1);}
             else {gpioWrite (17, 0);}
        if(move_on == 1) {gpioWrite (18, 1);}
             else {gpioWrite (18, 0);}
        if(forward_reverse == 1) {gpioWrite (27, 1);}
             else {gpioWrite (27, 0);}
	return 0;
}

// Sets Steering Servo with the variable direction as the input.
// Steering limits of the mechanics need to be expanded. Fixes needed.

int steering (int azimuth)
{
    int c;
    cout << "Steering Module " << azimuth << endl;
    if (azimuth == 360) {c = 1500;}
//  Switch the + and - after 1500 on the next 2 lines if the servo is mounted above the drive.
    if (azimuth >= 0 && azimuth <= 60){c = 1500-azimuth*11;}
    if (azimuth >=300 && azimuth <= 359) {c = 1500 + (360-azimuth)*11;}
    if (azimuth > 60 && azimuth < 300) {c = 1500;direction = 0;}
    return c;
}

int main (void)
{
  time_boot = steady_clock::now();
  time_current = steady_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(time_current - time_boot);
  duration<double> time_picture = duration_cast<duration<double>>(time_current - time_last_picture);
  duration<double> time_mow = duration_cast<duration<double>>(time_current - time_last_mowing);
  duration<double> time_locate = duration_cast<duration<double>>(time_current - time_last_locate);
  selftest ();
  while (time_span.count() < 20)
  {
    controlupdate ();
    currentposition = steering (direction);
    cout << "I have been running for " << time_span.count() << " seconds." << endl;
    gpioServo (23, currentposition);
    cout << "Direction : " << direction << endl;
    cout << "Servo Position : " << currentposition << endl;
    cout << "Mower Power : " << mower_on << endl;
    cout << "Move Power : " << move_on << endl;
    cout << "Safety : " << safety << endl;
    cout << "Front Obstruction Detect : " <<  front_detect << endl; 
    cout << "Left Obstruction Detect : " << left_detect << endl;
    cout << "Right Obstruction Detect : " << right_detect << endl;
    cout << "Rear Obstruction Detect : " << rear_detect << endl;
    gpioSleep(PI_TIME_RELATIVE, 0, 200000); // sleep for 0.2 seconds
    time_current = steady_clock::now();
    time_span = duration_cast<duration<double>>(time_current - time_boot);
    time_picture = duration_cast<duration<double>>(time_current - time_last_picture);
    time_mow = duration_cast<duration<double>>(time_current - time_last_mowing);
    time_locate = duration_cast<duration<double>>(time_current - time_last_locate);
    if (time_picture.count > 10) 
      {pic02 (); 
      time_last_picture = duration_cast<duration<double>>(time_current);
      } //Saves a picture
  }
  // Needs a reset plan if the safety goes off.
  gpioWrite (17, 0); //Shut the mower off.
  gpioWrite (18, 0); //Shut the move off.
  gpioWrite (27, 1); //Set the drive to forward.
  gpioServo (23, 1500);// Centers the Servo
  gpioSleep(PI_TIME_RELATIVE, 0, 200000); // sleep for 0.2 seconds 
  gpioSleep(PI_TIME_RELATIVE, 0, 200000); // sleep for 0.2 seconds
  gpioSleep(PI_TIME_RELATIVE, 0, 200000); // sleep for 0.2 seconds
  void gpioTerminate (void);
  return 0 ;
}
