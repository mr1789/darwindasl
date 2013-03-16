#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>
#include <math.h>

using namespace cv;
using namespace std;

Mat original, inrange,pyrup,pyrdown,dil,GB,canny;
Mat mdilate(25,25,CV_8UC1);

vector<string> str;

vector<vector<Point> > contours;
vector<vector<Point> > squares;
vector<Point> approx;
vector<Point> vcenter;
Point pcenter;

double sum_x,sum_y;

static double angle( Point pt1, Point pt2, Point pt0 )
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

static void drawSquares( Mat& image, const vector<vector<Point> >& squares,const vector<Point> vcenter )
{
    for( size_t i = 0; i < squares.size(); i++ )
    {
        const Point* p = &squares[i][0];
        int n = (int)squares[i].size();
        polylines(image, &p, &n, 1, true, Scalar(0,255,0), 3, CV_AA);
		stringstream ss;
		ss <<"#" << i+1 << "(" << vcenter[i].x << "," << vcenter[i].y << ")";
		putText(original, ss.str(), vcenter[i], FONT_HERSHEY_SCRIPT_SIMPLEX, 0.8/*fontscale*/, Scalar(255,0,0),3/*thickness*/,6/*lineType*/,false);
		cout << "Square #"<< i+1 << "\nX: "<< vcenter[i].x << "\t\tY: " << vcenter[i].y <<endl;
    }
}

int main(int argc, char** argv)
{
	VideoCapture cap(0);
    if(!cap.isOpened())
        return -1;

    while(1)
    {
		//restart all variable values
		squares.clear();
		vcenter.clear();
		approx.clear();
		sum_x=0;
		sum_y=0;
		str.clear();

		cap >> original;//get new image
		
		//process image	to black n white	
		inRange(original,
				Scalar(100,120,100),
				Scalar(255,255,255),
				inrange);
		//reduce noise
		pyrDown(inrange, pyrdown, Size(original.cols/2, original.rows/2));
		pyrUp(pyrdown, pyrup, original.size());		
		GaussianBlur(pyrup, GB, Size(9,9), 5, 5);
		//detect lines
		Canny(GB, canny, 80, 200, 5);
		dilate(canny, dil, mdilate, Point(-1,-1));

		findContours(dil, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

////////this part is originally from the function findsquares() in the previous code
        for( size_t i = 0; i < contours.size(); i++ )
			{
			approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);
			if( approx.size() == 4 && fabs(contourArea(Mat(approx))) > 1000 && isContourConvex(Mat(approx)) )		
				{
					double maxCosine = 0;			
					for( int j = 2; j < 5; j++ )
					{
						double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
						maxCosine = MAX(maxCosine, cosine);
					}

					if( maxCosine < 0.3 )//if angles are more than 72 degrees, QUALIFY as square
					{
						squares.push_back(approx);//XnY of 4 vertices saved into 1 squares vector
						for(size_t k = 0; k<approx.size(); k++)//summing up e vertices
						{
							sum_x+=approx[k].x;
							sum_y+=approx[k].y;
						
						}
						pcenter.x=sum_x/4;
						pcenter.y=sum_y/4;//saving center coordinate to pcenter, double to int
						vcenter.push_back(pcenter);//argument mus b reference, Xny saved as under vector vcenter
						sum_x=0;
						sum_y=0;//refresh the sum of the vertices coordinates
					}
				}			
			}
////////////

//////////ensuring there is no double counting of squares detected
		//can be made into a function if you want
		for (size_t r = 1;r<vcenter.size();r++)//chk if centers are near each other; erase centers n sqaures tt are nearby each other
		{
			if(abs(vcenter[r].x-vcenter[r-1].x) <5 || abs(vcenter[r].y-vcenter[r-1].y) <5)
			{
				vcenter.erase(vcenter.begin()+r);
				squares.erase(squares.begin()+r);
			}
		}
//////////

		drawSquares(original,squares,vcenter);
	
		imshow("original", original);
		imshow("inrange", inrange);
		imshow("pyrup",pyrup);
		imshow("pyrdown",pyrdown);
		imshow("dil",dil);
		imshow("GB",GB);
		imshow("canny", canny);

        if(waitKey(30) >= 0) break;
	}
	original.release();
	inrange.release();
	canny.release();
	pyrup.release();
	pyrdown.release();
	dil.release();
	GB.release();
	return 0;
}