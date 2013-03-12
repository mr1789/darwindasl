#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>

using namespace cv;
using namespace std;

Mat original;//hold e image
Mat inrange;//filter out only white color to grayscale single channel array
Mat pyr, timg, gray;

vector<vector<Point> > squares;
vector<vector<Point> > contours;
vector<Point> approx;
//vector<vector<Point> > coordinates;

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
    squares.clear();//empty the previous stored value

	int squareCount=0;
	  
	Canny(image, gray, 50, 200, 5);
	dilate(gray, gray, Mat(), Point(-1,-1));

    findContours(gray, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

    for( size_t i = 0; i < contours.size(); i++ )
    {
        approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);
	
        if( approx.size() == 4 && fabs(contourArea(Mat(approx))) > 1000 && isContourConvex(Mat(approx)) )// and be convex.
        {
            squareCount++;
			double maxCosine = 0;
			//printing the coordinates of each square
			double sum_x=0, sum_y=0;
			for(int k=0; k<approx.size(); k++)
			{
				sum_x+=approx[k].x;
				sum_y+=approx[k].y;
			}
			cout << "Square #"<< squareCount << "\nX: "<< sum_x/4 << "\t\tY: " << sum_y/4 <<endl;
			///////////
            for( int j = 2; j < 5; j++ )
            {
                double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
                maxCosine = MAX(maxCosine, cosine);
            }

            if( maxCosine < 0.3 )
                squares.push_back(approx);
				//each time push_back() stores 4 2D points into squares
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
		//C: void cvPolyLine(CvArr* img, CvPoint** pts, const int* npts, int contours, int is_closed, CvScalar color, int thickness=1, int line_type=8, int shift=0 )
    }

    //imshow(wndname, image);
}

int main(int argc, char** argv)
{
	VideoCapture cap(0); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return -1;

    while(1)
    {
		cap >> original; // get a new frame from camera

		// down-scale and upscale the image to filter out the noise
		pyrDown(original, pyr, Size(original.cols/2, original.rows/2));
		pyrUp(pyr, timg, original.size());
		//cvtColor(timg, inrange, CV_BGR2GRAY);
		inRange(	timg,				// function input
					Scalar(0,  0,  0),			// min filtering value (if color is greater than or equal to this)
					Scalar(90,90,90),			// max filtering value (if color is less than this)
					inrange);

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