/*
 * main.cpp
 *
 *  Created on: 2011. 1. 4.
 *      Author: robotis
 */

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <libgen.h>
#include "DARwIn.h"//y cant this work?
#include "Camera.h"
#include "mjpg_streamer.h"
#include "LinuxDARwIn.h"
//#include "myfunc.h"

#define INI_FILE_PATH       "config.ini"
#define U2D_DEV_NAME        "/dev/ttyUSB0"

using namespace Robot;
using namespace std;

void showvalue()
{
	printf("Walking::GetInstance()->A_MOVE_AMPLITUDE= %f\n", Walking::GetInstance()->A_MOVE_AMPLITUDE);
	printf("Walking::GetInstance()->X_MOVE_AMPLITUDE= %f\n", Walking::GetInstance()->X_MOVE_AMPLITUDE);
}

void inc()
{
	//Walking::GetInstance()->A_MOVE_AMPLITUDE+=10;
	Walking::GetInstance()->X_MOVE_AMPLITUDE+=10;
}
void change_current_dir()
{
    char exepath[1024] = {0};
    if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
        chdir(dirname(exepath));
}

int main(void)
{
	int count=0;
    	
	printf( "\n===== Ball following Tutorial for DARwIn =====\n\n");

    	change_current_dir();

	Image* rgb_ball = new Image(Camera::WIDTH, Camera::HEIGHT, Image::RGB_PIXEL_SIZE);

	minIni* ini = new minIni(INI_FILE_PATH);

	LinuxCamera::GetInstance()->Initialize(0);
	LinuxCamera::GetInstance()->LoadINISettings(ini);

	mjpg_streamer* streamer = new mjpg_streamer(Camera::WIDTH, Camera::HEIGHT);

	ColorFinder* ball_finder = new ColorFinder();
	ball_finder->LoadINISettings(ini);
	httpd::ball_finder = ball_finder;

	BallTracker tracker = BallTracker();
	BallFollower follower = BallFollower();
	follower.DEBUG_PRINT = true;

	//////////////////// Framework Initialize ////////////////////////////
	LinuxCM730 linux_cm730(U2D_DEV_NAME);
	CM730 cm730(&linux_cm730);
	if(MotionManager::GetInstance()->Initialize(&cm730) == false)
	{
		printf("Fail to initialize Motion Manager!\n");
			return 0;
	}
	MotionManager::GetInstance()->AddModule((MotionModule*)Head::GetInstance());
	MotionManager::GetInstance()->AddModule((MotionModule*)Walking::GetInstance());
    	LinuxMotionTimer *motion_timer = new LinuxMotionTimer(MotionManager::GetInstance());
    	motion_timer->Start();
	/////////////////////////////////////////////////////////////////////

	int n = 0;
	int param[JointData::NUMBER_OF_JOINTS * 5];
	int wGoalPosition, wStartPosition, wDistance;

	for(int id=JointData::ID_R_SHOULDER_PITCH; id<JointData::NUMBER_OF_JOINTS; id++)
	{
		wStartPosition = MotionStatus::m_CurrentJoints.GetValue(id);
		wGoalPosition = Walking::GetInstance()->m_Joint.GetValue(id);
		if( wStartPosition > wGoalPosition )
			wDistance = wStartPosition - wGoalPosition;
		else
			wDistance = wGoalPosition - wStartPosition;

		wDistance >>= 2;
		if( wDistance < 8 )
			wDistance = 8;

		param[n++] = id;
		param[n++] = CM730::GetLowByte(wGoalPosition);
		param[n++] = CM730::GetHighByte(wGoalPosition);
		param[n++] = CM730::GetLowByte(wDistance);
		param[n++] = CM730::GetHighByte(wDistance);
	}
	cm730.SyncWrite(MX28::P_GOAL_POSITION_L, 5, JointData::NUMBER_OF_JOINTS - 1, param);	
	//////////////////////////////////////////////////////////////////////


	//////////////////////////////////////////////////////////////////////
	Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
	Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
	MotionManager::GetInstance()->SetEnable(true);
	//////////////////////////////////////////////////////////////////////

	Walking::GetInstance()->X_MOVE_AMPLITUDE = 10.0;//initialize value of variables
 	Walking::GetInstance()->A_MOVE_AMPLITUDE = 10.0;

	printf("Press the ENTER key to begin!\n");
	getchar();

while(count<8)//limit max value
{
 	Walking::GetInstance()->Start();//start walking
	showvalue();
	printf("Press the ENTER key to increase value! \ncount=%d \n", count);	
	getchar();
	inc();
	count++;
} 

	Walking::GetInstance()->Stop(); //walking stops when variables reahes max value


    return 0;
}


