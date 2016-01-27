#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <math.h>

using namespace cv;
using namespace std;

//HSV threshold: (80-90,90-180,0-255)
//X: 0.07914 degrees per pixel
//Y: 0.08189 degrees per pixel

cv::Rect findBiggestBlob(cv::Mat & matImage);//this function finds the bounding rectangle of the largest contiguous region in the image

int main(){
	VideoCapture camera(1);//initialize camera

	namedWindow("ctrl",WINDOW_AUTOSIZE);//control window for calibrating the camera

	Mat screen_cap;//camera image
	Mat hsv_img;//camera image converted to hsv
	Mat thresholded;//camera image thresholded

	//high and low hsv threshold settings
	int Hlow = 62;
	int Hhigh = 98;
	int Slow = 0;
	int Shigh = 95;
	int Vlow = 223;
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

	double goal_height = 83;//preset height of the goal, in INCHES. For the competition goals it should be 83
	double camera_height = 0;//preset height of the camera, in INCHES. I don't know where we're mounting the camera on the robot yet
	double distance;//distance camera is from goal, to be calculated
	double phi;//vertical angle of deviance the sightline of the camera has from the bottom of the goal
	double theta;//horizontal angle of deviance the sightline of the camera has from the center of the goal

	while (true){
		camera.read(screen_cap);//get the current frame
		cvtColor(screen_cap,hsv_img,CV_BGR2HSV);//convert the frame to HSV, because it's easier to threshold (theoretically)

		inRange(hsv_img,Scalar(Hlow,Slow,Vlow),Scalar(Hhigh,Shigh,Vhigh),thresholded);//threshold it
		//to be honest I'm not entirely sure what this bit does other than lower the resolution and make everything into little blobs, but supposedly that helps get rid of noise so who am I to contest that
		erode(thresholded,thresholded,getStructuringElement(MORPH_ELLIPSE,Size(5,5)));
		dilate(thresholded,thresholded,getStructuringElement(MORPH_ELLIPSE,Size(5,5)));
		dilate(thresholded,thresholded,getStructuringElement(MORPH_ELLIPSE,Size(5,5)));
		erode(thresholded,thresholded,getStructuringElement(MORPH_ELLIPSE,Size(5,5))); 

		imshow("asdf",thresholded);//show the thresholded binary image, for purposes of calibrating

		friggin_box = findBiggestBlob(thresholded);//get the bounding box of the biggest goal
		rectangle(screen_cap,friggin_box,Scalar(0,0,255));//draw it onto the screen because I want to

		//calculate the center of the box and some other stuff which doesn't actually do anything yet but I'm workin on it
		center_x = friggin_box.x + (friggin_box.width/2.0);
		center_y = friggin_box.y + (friggin_box.height/2.0);
		phi = (240 - friggin_box.y+friggin_box.height)*0.08189;
		distance = goal_height/tan(phi);//this isn't quite right yet - this is actually the distance to the nearest point of the goal, but I will soon fix it so it is the distance to the center of the goal

		imshow("asdff",screen_cap);//show the image

		//exit program if pressing ESC
		if (waitKey(10) == 27)
			return 0;
	}

	return 0;
}


cv::Rect findBiggestBlob(cv::Mat & matImage){
	int largest_area = 0;
	int largest_contour_index = 0;
	Rect bounding_rect;

	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

	findContours(matImage, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );

	for(int i = 0; i< contours.size(); i++){
		double a = contourArea(contours[i], false);
		if (a > largest_area){
			largest_area = a;
			largest_contour_index = i;
			bounding_rect = boundingRect(contours[i]);
		}
	}

	return bounding_rect;
}