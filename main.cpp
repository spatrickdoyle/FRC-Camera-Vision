#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <math.h>
#include <iomanip>
#include <fstream>

using namespace cv;
using namespace std;

cv::Rect findBiggestBlob(cv::Mat & matImage);//this function finds the bounding rectangle of the largest contiguous region in the image

const double PI = 3.141592653589793238462643383279;
const double hor_deg = 0.07914;//in DEGREES PER PIXEL. Horizontal field of view is 50.6496 degrees
const double vert_deg = 0.08189;//in DEGREES PER PIXEL. Vertical field of view is 39.3072 degrees

int main(){
	VideoCapture camera(0);//initialize camera

	//namedWindow("ctrl",WINDOW_AUTOSIZE);//control window for calibrating the camera

	Mat screen_cap;//camera image
	Mat hsv_img;//camera image converted to hsv
	Mat thresholded;//camera image thresholded
	Mat cap1;
	Mat cap2;

	//high and low hsv threshold settings
	int Hlow = 67;
	int Hhigh = 91;
	int Slow = 14;
	int Shigh = 238;
	int Vlow = 239;
	int Vhigh = 255;

	//track bars for each threshold variable
	createTrackbar("HL","ctrl",&Hlow,179);
	createTrackbar("HH","ctrl",&Hhigh,179);
	createTrackbar("SL","ctrl",&Slow,255);
	createTrackbar("SH","ctrl",&Shigh,255);
	createTrackbar("VL","ctrl",&Vlow,255);
	createTrackbar("VH","ctrl",&Vhigh,255);

	Rect friggin_box;//bounding rectangle for the largest blob
	//center point of the friggin box
	int center_x;
	int center_y;

	double goal_height = 95;//preset height to top of goal, in INCHES. For the competition goals it should be 97
	double camera_height = 15;//preset height of the camera, in INCHES. I don't know where we're mounting the camera on the robot yet
	double camera_angle = 42;//angle of deviance from the horizontal, in DEGREES. Should be 42 officially
	double length = 20;//width of the goal in INCHES
	double offset = 0;//how far offset the center of the shooter is from the camera in INCHES. Positive means the camera is to the left of the shooter

	double distance;//distance camera is from goal, to be calculated
	double phi;//vertical angle of deviance the sightline of the camera has from the bottom of the goal
	double theta;//horizontal angle of deviance the sightline of the camera has from the center of the goal

	int flag = 1;

	//ofstream i2c_file ("/dev/i2c-1",ofstream::binary);
	//char buf[10] = {1};

	while (true){
		//i2c_file.write(buf,10);

		camera.read(screen_cap);//get the current frame

		//smooth everything out (I'm not sure if this is necessary but I'm not getting rid of it now)
		erode(screen_cap,screen_cap,getStructuringElement(MORPH_ELLIPSE,Size(5,5)));
		dilate(screen_cap,screen_cap,getStructuringElement(MORPH_ELLIPSE,Size(5,5)));
		dilate(screen_cap,screen_cap,getStructuringElement(MORPH_ELLIPSE,Size(5,5)));
		erode(screen_cap,screen_cap,getStructuringElement(MORPH_ELLIPSE,Size(5,5)));

		cvtColor(screen_cap,hsv_img,CV_BGR2HSV);//convert the frame to HSV, because it's easier to threshold (theoretically)

		inRange(hsv_img,Scalar(Hlow,Slow,Vlow),Scalar(Hhigh,Shigh,Vhigh),thresholded);//threshold it

		//do it again (to get rid of noise)
		erode(thresholded,thresholded,getStructuringElement(MORPH_ELLIPSE,Size(5,5)));
		dilate(thresholded,thresholded,getStructuringElement(MORPH_ELLIPSE,Size(5,5)));
		dilate(thresholded,thresholded,getStructuringElement(MORPH_ELLIPSE,Size(5,5)));
		erode(thresholded,thresholded,getStructuringElement(MORPH_ELLIPSE,Size(5,5)));

		friggin_box = findBiggestBlob(thresholded);//get the bounding box of the biggest goal
		rectangle(screen_cap,friggin_box,Scalar(0,0,255));//draw it onto the screen because I want to

		//imshow("thresholded",thresholded);
		//imshow("screen_cap",screen_cap);
		if (flag == 3)
			imshow("screen_cap",cap1-cap2);

		//calculate the center of the box, the distance from the goal, and the angle of deviance of the sightline
		center_x = friggin_box.x + (friggin_box.width/2.0);
		center_y = friggin_box.y + (friggin_box.height/2.0);
		phi = (240 - friggin_box.y)*vert_deg;
		distance = (goal_height-camera_height)/tan((phi+camera_angle)*(PI/180.0));
		distance += sqrt(pow(length/2.0,2) - pow(distance*tan((friggin_box.width/2.0)*hor_deg*(PI/180.0)),2));
		theta = (center_x-320)*hor_deg;//in DEGREES. Positive value of theta indicates the robot must turn in the COUNTERCLOCKWISE direction because that's how math works

		//distance = sqrt(pow(offset + d*cos((90-theta)*(PI/180.0)),2) + pow(d*sin((90-theta)*(PI/180.0)),2));
		//cout << sqrt(pow(offset + d*cos((90-theta)*(PI/180.0)),2) + pow(d*sin((90-theta)*(PI/180.0)),2)) << '\n';

		cout << "DISTANCE:" << setw(9) << distance << "   ANGLE:" << setw(9) << theta << '\n';

		//exit program if pressing ESC
		if (waitKey(10) == 27)
			return 0;
		else if (waitKey(10) == 32){
			cout << flag << '\n';
			if (flag == 1)
				cap1 = screen_cap.clone();
			else if (flag == 2)
				cap2 = screen_cap.clone();
			flag++;
		}
	}

	return 0;
}


Rect findBiggestBlob(Mat &matImage){
	int largest_area = 0;
	int largest_contour_index = 0;
	Rect bounding_rect;
	Mat img_clone = matImage.clone();

	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	findContours(img_clone, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );

	for(int i = 0; i < contours.size(); i++){
		double a = contourArea(contours[i], false);
		if (a > largest_area){
			largest_area = a;
			largest_contour_index = i;
			bounding_rect = boundingRect(contours[i]);
		}
	}

	return bounding_rect;
}
