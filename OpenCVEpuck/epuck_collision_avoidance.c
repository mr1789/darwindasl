#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <cv.h>
#include <cxcore.h>
#include <cvcompat.h>
#include <cxtypes.h>
#include <cvaux.h>
#include <highgui.h>

#include <stdio.h>
#include <stdlib.h>

#define TIME_STEP 64

int main(int argc, char **argv)
{
  wb_robot_init();

  int i;
  WbDeviceTag ps[8];
  char ps_names[8][4] = {
    "ps0", "ps1", "ps2", "ps3",
    "ps4", "ps5", "ps6", "ps7"
  };
  
  for (i=0; i<8 ; i++) {
    ps[i] = wb_robot_get_device(ps_names[i]);
    wb_distance_sensor_enable(ps[i], TIME_STEP);
  }
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera,TIME_STEP);
  
////////////////////
 int width =  wb_camera_get_width(camera);
  int height = wb_camera_get_height(camera);
  IplImage* imgOpencv = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
  unsigned char* data = (unsigned char *)imgOpencv->imageData;
  //cvNamedWindow( "Example1", CV_WINDOW_AUTOSIZE);
  const unsigned char* image;
  int max = height * width * 3;
/////////////////////  
  
  while (1) { 
    int delay = wb_robot_step(TIME_STEP);
    if (delay == -1) // exit event from webots
      break;
//////////////
    wb_camera_save_image(camera,"myimage.jpg",1);
    imgOpencv = cvLoadImage( "myimage.jpg",CV_LOAD_IMAGE_COLOR );
    //cvShowImage("Example1", imgOpencv);
    cvReleaseImage( &imgOpencv );

    double ps_values[8];
    for (i=0; i<8 ; i++)
      ps_values[i] = wb_distance_sensor_get_value(ps[i]);
    
    bool left_obstacle =
      ps_values[0] > 100.0 ||
      ps_values[1] > 100.0 ||
      ps_values[2] > 100.0;
    bool right_obstacle =
      ps_values[5] > 100.0 ||
      ps_values[6] > 100.0 ||
      ps_values[7] > 100.0;

    double left_speed  = 500;
    double right_speed = 500;
    
    if (left_obstacle) {
      left_speed  -= 500;
      right_speed += 500;
    }
    else if (right_obstacle) {
      left_speed  += 500;
      right_speed -= 500;
    }
    
    wb_differential_wheels_set_speed(left_speed, right_speed);
  }
  
  wb_robot_cleanup();
  return 0;
}