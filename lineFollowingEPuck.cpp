/*****************************************
  
  intermediate_lfm
  Author: Rohrer Fabien
  Inspired of te epuck_line_demo controller of
  Jean-Christophe Zufferey
  
******************************************/

// Included libraries
#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/camera.h>
#include <webots/led.h>
#include <stdlib.h>

// Global defines
#define THRESHOLD_DIST 50
#define TIME_STEP 32 // [ms] // time step of the simulation
#define TIME_STEP_CAM  32
#define SIMULATION 0 // for robot_get_mode() function
#define REALITY 2    // for robot_get_mode() function
#define LEFT 0        // Left side
#define RIGHT 1       // right side

//leds
#define NB_LEDS   8
#define ON 1
#define OFF 0
WbDeviceTag led[NB_LEDS];

// camera
WbDeviceTag cam;
unsigned short width, height;

/*****************************
*
*  Functions
*
******************************/

// This function returns the position
// of the peak contained in the array given
// in argument
int find_middle(int tab[], int sizeTab){
  
  int i,j;
  int *copy = (int *)malloc(sizeof(int)*sizeTab);
  int mean=0;
  int nb_best = sizeTab/10;
  ////nb_best will be working on a tenth of the camera width, thus sizeTab, 
  ////which is equal to width of the camera view, is deived by 10
  int *index_bests = (int *)malloc(sizeof(int)*nb_best);
  
  // copy the tab, calculate the mean and
  // test if all the values are identical
  int identical=1;
  for (i=0; i<sizeTab;i++){
    copy[i]=tab[i];
    mean+=tab[i];
    if (tab[i]!=tab[0]) identical=0;
	////if last pixel of the 1st line of the camera does not equal the 1st pixel of the camera, identical==0
  }
  if (identical) {
    free(copy);
	free(index_bests);
    return sizeTab/2;
	////if 1st and last pixel of the topmost row of pixels of the camera width have the same value,
	////value of sizeTab/2 is return to delta in lfm(), which eventually equates lfm_speed[LEFT] and lfm_speed[RIGHT] to 0
	////This means that if 
	////the whole body of Epuck is within the line and it does not see the edge of the line in the topmost row of its camera,
	////or if
	////the whole body is outside of the line and it does not see any part of the black line,
	////Epuck will continue to move straight forward at the default speed
  }
  mean/=sizeTab;
  ////mean will be assigned the average value of all the pixels in the topmost row of pixel
  
  // take the best values of the tab
  for (i=0; i<nb_best; i++){
    index_bests[i]=-1;
    int index=-1;
    int max=0;
    for (j=0; j<sizeTab; j++){
      if (max<copy[j] && copy[j]>mean){
        max=copy[j];
        index=j;
		////The index of the blackest pixel will be saved in max
		////Only the 1st of the blackest pixel will be stored and would not be overwritten
      }
    }
    index_bests[i]=index;
    if (index >=0 && index < sizeTab)
	//if the blackest pixel not the 1st or last pixel in the topmost row of pixel of the camera
      copy[index]=0;
  }
  free(copy);
  // calculate the position mean of the best values
  int firstMean=0;
  int count=0;
  for (i=0; i<nb_best; i++){
    if(index_bests[i]!=-1) {
      firstMean+=index_bests[i];
      count++;
    }
  }
  if (count==0) {
    free(index_bests);
	return sizeTab/2;
  }
  firstMean/=count;

  // eliminate extreme values
  int secondMean=0;
  count=0;
  for (i=0; i<nb_best; i++){
    if (index_bests[i]<firstMean+sizeTab/10 && index_bests[i]>firstMean-sizeTab/10) {
      count++;
      secondMean+=index_bests[i];
    }
  }
  free(index_bests);
  if (count==0) return sizeTab/2;
  
  return secondMean/count;
}

// return the mean of the values of an array
int mean(int array[], int size){
  if (size==0) {return 0;}
  int sum=0,i;
  for (i=0;i<size;i++){
    sum+=array[i];
  }
  return sum/size;
}

/*****************************
*
*  Modules
*
******************************/

//Line following module
#define MAX_DELTA 300.0f
int lfm_speed[2] = {0,0};
int lfm_active = 1;
void lfm(int array[], int size){
  if (lfm_active){
    int delta = find_middle(array,size)-width/2;
    lfm_speed[LEFT]=MAX_DELTA*delta/size;
    lfm_speed[RIGHT]=-lfm_speed[LEFT];
	////give adjustments to default speed, speed[LEFT] and  speed[right]
  } else {
    lfm_speed[RIGHT]=lfm_speed[LEFT]=0;
  }
}

// line entering module
#define SENSIBILITY 10
int previous_mean[]={0,0,0};
int current_mean[]={0,0,0};
int is_in[]={0,0,0};
void lem(int array[], int size){

  int *left   = (int *)malloc(sizeof(int)*size/10);
  int *right  = (int *)malloc(sizeof(int)*size/10);
  int *middle = (int *)malloc(sizeof(int)*size/10);
  int i;
  
  for (i=0;i<size/10;i++){
    left[i]=array[i];////array of values of the leftmost tenth of the topmost row of the camera
    right[i]=array[size-1-i];////array of values of the rightmost tenth of the topmost row of the camera
    middle[i]=array[size/2-size/20+i];////value of middle tenth of pixels of camera
  }
  
  current_mean[0] = mean(left, size/10);
  current_mean[1] = mean(middle, size/10);
  current_mean[2] = mean(right, size/10);

  for (i=0;i<3;i++){
    if (current_mean[i]>previous_mean[i]+SENSIBILITY){
	////if each section is "black" enough, is_in[]=1
      is_in[i]=1;
    }
  }
  
  if (is_in[0]||is_in[1]||is_in[2]){
    lfm_active=1;//as long as one of the sections are black enough, considered following line
    
    // TODO : leds on
    //...
    
  }
  free(left);
  free(right);
  free(middle);
}
// line leaving module
void llm(int array[], int size){

  int i;
  for (i=0;i<3;i++){
    if (current_mean[i]<previous_mean[i]-SENSIBILITY){
      is_in[i]=0;
    }
    previous_mean[i]=current_mean[i];
  }
  
  if (!is_in[0]&&!is_in[1]&&!is_in[2]){
    lfm_active=0;

    // TODO : leds off
    //...

  }
}

// U Turn module
int utm_speed[]={0,0};
void utm(void){
  
  // TODO : implement this module
  // - put your results in the speed_utm array
  // - look at the main function: add a function
  //   call to this function and add your results
  //   in the differential_wheels_set_speed(...)
  //   function
  
  //...
  
}

/*****************************
*
*  Standard functions
*
******************************/

static void reset(void)
{ 
  int i;
  char text[5]="led0";
  for(i=0;i<NB_LEDS;i++) {
    led[i]=wb_robot_get_device(text);
    text[3]++;
    wb_led_set(led[i],0); 
  }

  // enable the camera
  cam = wb_robot_get_device("camera");
  wb_camera_enable(cam,TIME_STEP_CAM);
  width = wb_camera_get_width(cam);
  height = wb_camera_get_height(cam);
}

static int run(void) {

  int i;
  int *grey = (int *)malloc(sizeof(int)*width);
  int speed[2]={150,150};
  ////default fixed speed, whther Epuck is on line or not
  const unsigned char *image;
    
  // 1. Get the sensors values
  image = wb_camera_get_image(cam);
  for (i = 0; i < width; i++) {
    grey[i] = 255-wb_camera_image_get_grey(image, width, i, 0);
	////0 = black, 255 = white, so 255-0=255 in grey[i] = black (value representation is now reversed
	////capture just topmost row for the width of the camera vision
  }
  
  // 2. Behavior-based robotic:
  // call the modules in the right order
  lem(grey,width);
  lfm(grey, width);
  llm(grey, width);
  //utm();
  
  // 3. Send the values to actuators
  wb_differential_wheels_set_speed(
    speed[LEFT]+lfm_speed[LEFT],//+utm_speed[LEFT],
    speed[RIGHT]+lfm_speed[RIGHT]//+utm_speed[RIGHT]
  );
  free(grey);
  return TIME_STEP;
}

int main() {
  wb_robot_init();
  
  reset();
  
  while(wb_robot_step(TIME_STEP) != -1) {
    run();
  }
  
  wb_robot_cleanup();
  
  return 0;
}
