#include "VisualTracking.hpp"
#include <webots/Servo.hpp>
#include <webots/LED.hpp>
#include <webots/Camera.hpp>
#include <managers/DARwInOPVisionManager.hpp>

#include <cstdlib>
#include <cmath>
#include <iostream>
#include <fstream>

using namespace webots;
using namespace managers;
using namespace std;

static const char *servoNames[NSERVOS] = {
  "ShoulderR" /*ID1 */, "ShoulderL" /*ID2 */, "ArmUpperR" /*ID3 */, "ArmUpperL" /*ID4 */,
  "ArmLowerR" /*ID5 */, "ArmLowerL" /*ID6 */, "PelvYR"    /*ID7 */, "PelvYL"    /*ID8 */,
  "PelvR"     /*ID9 */, "PelvL"     /*ID10*/, "LegUpperR" /*ID11*/, "LegUpperL" /*ID12*/,
  "LegLowerR" /*ID13*/, "LegLowerL" /*ID14*/, "AnkleR"    /*ID15*/, "AnkleL"    /*ID16*/,
  "FootR"     /*ID17*/, "FootL"     /*ID18*/, "Neck"      /*ID19*/, "Head"      /*ID20*/
};

VisualTracking::VisualTracking():
    Robot()
{
  //m****** variable are defined in VisualTracking.hpp
  mTimeStep = getBasicTimeStep();//int
  mEyeLED = getLED("EyeLed");//LED class; pointer
  mHeadLED = getLED("HeadLed");//LED class; pointer
  mCamera = getCamera("Camera");//Camera class; pointer
  mCamera->enable(mTimeStep);
  
  for (int i=0; i<NSERVOS; i++)
    mServos[i] = getServo(servoNames[i]);
  mVisionManager = new DARwInOPVisionManager(mCamera->getWidth(), mCamera->getHeight(), 355, 15, 60, 15, 0.1, 30);
  //width =360, height = 240
  //from DARwInOPVisionManager.cpp
  //DARwInOPVisionManager::DARwInOPVisionManager(int width, int height, 
  //int hue, int hueTolerance, int minSaturation, int minValue, int minPercent, int maxPercent
}

VisualTracking::~VisualTracking() {
}

void VisualTracking::myStep() {
  int ret = step(mTimeStep);
  //////////where did step came from
  if (ret == -1)
    exit(EXIT_SUCCESS);
}

// function containing the main feedback loop
void VisualTracking::run() {
  double horizontal = 0.0;//head at centre default position
  double vertical = 0.0;
	
  cout << "---------------Visual Tracking---------------" << endl;
  cout << "This example illustrate the possibilities of the vision manager." << endl;
  cout << "Move the red ball by pressing ctrl + shift and selecting it (mouse left pressed)." << endl;
	
  // First step to update sensors values
  myStep();//function found in this cpp
  ////////////y this function updates sensor value?

  double x, y,xold=0,yold=0;

  while (true) {
    
    bool ballInFieldOfView = mVisionManager->getBallCenter(x, y, mCamera->getImage());
	//3rd argument is const unsigned char array
	//converts image to HSV format and save as Point2D class
	//looks for ball in HSV format
            //true if present and give value to X n Y
	//false if absent and x n y == 0.0
	//save value to ballInFieldOfView
	//use ColorFinder Class in getBallCenter() to find ball
    	
    if(yold != y || xold !=x){
      cout << "x= "<< x <<" y= "<< y <<" Detected: "<< ballInFieldOfView<< endl;
      cout << "horizontal= "<< horizontal <<" vertical= "<< vertical << endl;
    }
    // Eye led indicate if ball has been found
    if(ballInFieldOfView)
      mEyeLED->set(0x00FF00);
    else
      mEyeLED->set(0xFF0000);
    
    // Move the head in direction of the ball if it has been found
    if(ballInFieldOfView) {
      // Horizontal
      if(x > 0.6 * mCamera->getWidth())
        horizontal -= 0.05;
      //if ball is located >216, ie too right, horizontal goes into -ve to turn neck left
      else if(x < 0.4 * mCamera->getWidth())
        horizontal += 0.05;
      //if ball is located >216, ie too left, horizontal goes into +ve to turn neck right
	  // Vertical
      if(y > 0.6 * mCamera->getHeight())
        vertical -= 0.02;
      else if(y < 0.4 * mCamera->getHeight())
        vertical += 0.02;
		//making coordinate within 144 to 216 in X axis
		//and 96 to 144 in the Y axis of camera	frame
    }
    
    //mServos[18]->setPosition(horizontal);//neck servo in charge of panning
    //mServos[19]->setPosition(vertical);//head servo in charge of tilting
    //changes in horizontal and vertical values stored
    //no changes in values when ball detected, so head remain last position
    
    xold=x;
    yold=y;
    //store old values for comparison  
    
    // step
    myStep();
  }
}