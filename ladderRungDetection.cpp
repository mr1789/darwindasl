#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>

using namespace cv;
using namespace std;

Mat original;//hold e image
Mat inrange;//filter out only white color to grayscale single channel array
Mat pyr, timg, gray;

vector<vector<Point> > contours;
//declaration of a vector of a vector of X and Y points
//the nested vector of points is the list of coordinates of point that make up ONE contour
//the outer vector of vectors of points is the list of ALL the contours detected

vector<vector<Point> > squares;
//a vector storing a vector storing a list of X and Y points which refer to the vertices of each detected rectangle
vector<Point> approx;
//a vector to store a list of vertices of ONE detected rectangle

static double angle( Point pt1, Point pt2, Point pt0 )
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

static void findSquares( const Mat& image, vector<vector<Point> >& squares )
{

    squares.clear();
	//empty the previous stored value

	int squareCount=0;
	  
	Canny(image, gray, 50, 200, 5);
	//Detect edges on the input "image" passed by reference from the main program into this findSquare function
	//save the newly processed to "gray"
	//with lower threshold of 50 and higher threshold to 200
	//with the aperture size of the kernel at 5

	dilate(gray, gray, Mat(), Point(-1,-1));
	//take input image, that is the 1st "gray"
	//"blur" the image (I am not too sure of the term that should be used)
	//save processed image to 2nd "gray"
	//using the kernel of default aperture size of 3 as defined by "Mat()"
	//at default start point specified by "Point(-1,-1)"

    findContours(gray, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
	//input image is "gray"
	//processed image is saved to contours
	//in the designated mode

    for( size_t i = 0; i < contours.size(); i++ )
    {
        approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);
		//for each detected contour, if they approximate to a polygon according to Ramer–Douglas–Peucker algorithm
		//the vertices of X and Y coordinates are stored as a list in the vector "approx"

		//if the polygons are squares or rectangles they must be
        if( approx.size() == 4							//4sided
			&& fabs(contourArea(Mat(approx))) > 1000	//big enough (eliminate noise)
			&& isContourConvex(Mat(approx)) )			//convex
        {
            squareCount++;
			//taking count of the number of rectangles detected

			double maxCosine = 0;			
            for( int j = 2; j < 5; j++ )
            {
                double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
				//store cosine of the angle between 3 adjacent vertices into the variable "cosine"
                maxCosine = MAX(maxCosine, cosine);
            }

            if( maxCosine < 0.3 )
				squares.push_back(approx);
				//if the angles are above cos^-1(0.3) = 72 degrees,
				//the 4 sided polygon characterized by the current contour is considered a square
				//the "push_back" function stores vector of 4 2D points into the "squares" vector

			double sum_x=0, sum_y=0;
			for(int k=0; k<approx.size(); k++)
			{
				sum_x+=approx[k].x;
				sum_y+=approx[k].y;
			}
			cout << "Square #"<< squareCount << "\nX: "<< sum_x/4 << "\t\tY: " << sum_y/4 <<endl;
			//print the coordinates of the center of each detected rectangles
        }
    }
}

static void drawSquares( Mat& image, const vector<vector<Point> >& squares )
{
    for( size_t i = 0; i < squares.size(); i++ )
    {
        const Point* p = &squares[i][0];
        int n = (int)squares[i].size();
        polylines(image, &p, &n, 1, true, Scalar(0,255,0), 3, CV_AA);
		//outline each detected rectangles
    }
}

int main(int argc, char** argv)
{
	VideoCapture cap(0); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return -1;

    while(1)
    {
		cap >> original; // get a new frame from camera
		
		inRange(timg,// input timg as the image to be processed
				Scalar(0,  0,  0),// min filtering value (if color is greater than or equal to this)
				Scalar(90,90,90),// max filtering value (if color is less than this)
				inrange);//save filtered image to inrange

		// down-scale and upscale the image to filter out the noise
		pyrDown(inrange, pyr, Size(original.cols/2, original.rows/2));
		pyrUp(pyr, timg, original.size());

		findSquares(inrange,squares);
		drawSquares(original,squares);
		
		imshow("original", original);
		imshow("inrange", inrange);
		imshow("gray", gray);

        if(waitKey(30) >= 0) break;
	}
	original.release();
	inrange.release();
	gray.release();
	pyr.release();
	timg.release();
	return 0;
}