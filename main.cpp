#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <math.h>
#include <iomanip>

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <fstream>

/*TODO:
control the LED ring from the board and use it to shoot (by communicating with the rio)
control LED strips to indicate if the shooter is lined up - OKAY APPARENTLY THE STRIP TAKES 12V SO WE'RE GONNA HAVE TO FIGURE OUT ANOTHER WAY TO POWER IT
autodetect the ids for the usb and camera
catch if the serial or camera don't load
make second camera recognize balls
*/

using namespace cv;
using namespace std;

Rect findBiggestBlob(Mat & matImage);//this function finds the bounding rectangle of the largest contiguous region in the image

const double PI = 3.141592653589793238462643383279;
const double hor_deg = 0.07914;//in DEGREES PER PIXEL. Horizontal field of view is 50.6496 degrees
const double vert_deg = 0.08189;//in DEGREES PER PIXEL. Vertical field of view is 39.3072 degrees

int main(int argc, char *argv[]){
	system("fswebcam -d /dev/video0 -c cam.conf");//configure camera. It runs every time because I don't know how persistant the changes are
	VideoCapture camera(0);//initialize camera

	if (argc == 2){
		cout << "opening calibration window\n";
		namedWindow("ctrl",WINDOW_AUTOSIZE);//control window for calibrating the camera
	}

	Mat screen_cap;//camera image
	Mat hsv_img;//camera image converted to hsv
	Mat thresholded;//camera image thresholded

	//high and low hsv threshold settings, to be changed when calibrating
	int Hlow;// = 39;
	int Hhigh;// = 95;
	int Slow;// = 200;
	int Shigh;// = 255;
	int Vlow;// = 0;
	int Vhigh;// = 255;

	ifstream hsv_file;
	hsv_file.open("hsv.conf");
	hsv_file >> Hlow;
	hsv_file >> Hhigh;
	hsv_file >> Slow;
	hsv_file >> Shigh;
	hsv_file >> Vlow;
	hsv_file >> Vhigh;
	hsv_file.close();
	ofstream hsv_out;

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

	double goal_height = 95;//preset height to top of goal, in INCHES. For the competition goals it should be 95
	double camera_height = 25;//preset height of the camera, in INCHES. I don't know where we're mounting the camera on the robot yet
	double camera_angle = 21;//angle of deviance from the horizontal, in DEGREES. Should be 42 officially
	double length = 20;//width of the goal in INCHES
	double ball_diameter = 10;
	double offset = 6.75;//offset of the camera from the center of the robot in INCHES

	float distance;//distance camera is from goal, to be calculated
	double phi;//vertical angle of deviance the sightline of the camera has from the bottom of the goal
	float theta;//horizontal angle of deviance the sightline of the camera has from the center of the goal
	double shooter_angle;

	char send_buffer[8];
	int output_port = open("/dev/ttyUSB0", O_RDWR|O_NOCTTY);
	struct termios port_options;

	tcgetattr(output_port, &port_options);
	cfsetispeed(&port_options, B38400);
	cfsetospeed(&port_options, B38400);
	port_options.c_cflag &= ~PARENB;
	port_options.c_cflag &= ~CSTOPB;
	port_options.c_cflag &= ~CSIZE;
	port_options.c_cflag |= CS8;
	tcsetattr(output_port, TCSANOW, &port_options);

	while (true){
		camera.read(screen_cap);//get the current frame

		cvtColor(screen_cap,hsv_img,CV_BGR2HSV);//convert the frame to HSV, because it's easier to threshold (theoretically)

		inRange(hsv_img,Scalar(Hlow,Slow,Vlow),Scalar(Hhigh,Shigh,Vhigh),thresholded);//threshold it

		erode(thresholded,thresholded,getStructuringElement(MORPH_ELLIPSE,Size(5,5)));
		dilate(thresholded,thresholded,getStructuringElement(MORPH_ELLIPSE,Size(5,5)));
		dilate(thresholded,thresholded,getStructuringElement(MORPH_ELLIPSE,Size(5,5)));
		erode(thresholded,thresholded,getStructuringElement(MORPH_ELLIPSE,Size(5,5)));

		friggin_box = findBiggestBlob(thresholded);//get the bounding box of the biggest goal
		rectangle(screen_cap,friggin_box,Scalar(0,0,255));//draw it onto the screen because I want to

		//show the image and the thresholded image, only necessary for calibration
		if (argc == 2){
			imshow("thresholded",thresholded);
			imshow("screen_cap",screen_cap);
		}

		//calculate the center of the box, the distance from the goal, and the angle of deviance of the sightline
		center_x = friggin_box.x + (friggin_box.width/2.0);
		center_y = friggin_box.y + (friggin_box.height/2.0);
		phi = (240 - friggin_box.y)*vert_deg;
		distance = (goal_height-camera_height)/tan((phi+camera_angle)*(PI/180.0));//distance from the nearest point of the goal
		distance += sqrt(pow(length/2.0,2) - pow(distance*tan((friggin_box.width/2.0)*hor_deg*(PI/180.0)),2));//account for angle of approach
		theta = -(center_x-320)*hor_deg;//in DEGREES. Positive value of theta indicates the robot must turn in the COUNTERCLOCKWISE direction because that's how math works

		if (offset != 0){
			if (theta > 0)
				theta = 90 - (180.0/PI)*atan((distance*sin((90-theta)*(PI/180.0)))/((distance*cos((90-theta)*(PI/180.0)))+offset));
			else if (theta < 0)
				theta = (180.0/PI)*atan((distance*sin((90+theta)*(PI/180.0)))/((distance*cos((90+theta)*(PI/180.0)))+offset)) - 90;
		}

		shooter_angle = (hor_deg*(4.0/5.0)*friggin_box.width)/2.0;

		//cout << shooter_angle << ' ' << distance*tan((PI/180.0)*shooter_angle)*2 << '\n';
		if (shooter_angle > fabs(theta)){
		if (distance*tan((PI/180.0)*shooter_angle)*2 > ball_diameter){
			theta = 0;}}

		memcpy(send_buffer, &theta, 4);
		memcpy(send_buffer + 4, &distance, 4);

		cout << "DISTANCE:" << setw(9) << distance << "   ANGLE:" << setw(9) << theta << '\n';
		write(output_port, send_buffer, 8);

		//exit program if pressing ESC
		if (waitKey(10) == 27)
			return 0;
		else if (waitKey(10) == 32){
			hsv_out.open("hsv.conf");
			hsv_out << Hlow << '\n' << Hhigh << '\n' << Slow << '\n' << Shigh << '\n' << Vlow << '\n' << Vhigh << '\n';
			cout << "writing values to file... \n";
			hsv_out.close();
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
