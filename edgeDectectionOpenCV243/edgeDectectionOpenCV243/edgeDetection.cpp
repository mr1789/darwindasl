#include "cv.h"
#include "highgui.h"

using namespace cv;

int main(int, char**)
{
    VideoCapture cap(0); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return -1;

    Mat original;
	Mat processed;
	
    namedWindow("original",1);
	namedWindow("processed",1);

    for(;;)//same as while(1)
    {
        cap >> original; // get a new frame from camera
        cvtColor(original, processed, CV_BGR2GRAY);//original is colored, processed is grey
        GaussianBlur(processed, processed, Size(7,7), 1.5, 1.5);//original is not blur, processed is blur
        Canny(processed, processed, 0, 60, 3);//produce the lines, aperture size must be 3, 5 or 7

/*		delaration C++: void Canny(InputArray image, OutputArray edges, double threshold1, double threshold2, int apertureSize=3, bool L2gradient=false )
		Parameters:	
			image – single-channel 8-bit input image.
			edges – output edge map; it has the same size and type as image .
			threshold1 – first threshold for the hysteresis procedure.
			threshold2 – second threshold for the hysteresis procedure.
			apertureSize – aperture size for the Sobel() operator.
			L2gradient – a flag, indicating whether a more accurate   norm   
						 should be used to calculate the image gradient magnitude ( L2gradient=true ), 
						 or whether the default   norm   is enough ( L2gradient=false ).
*/

		imshow("original", original);
		imshow("processed", processed);
		original.release();
		processed.release();
        if(waitKey(30) >= 0) break;
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}