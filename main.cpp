#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <math.h>
#include <iomanip>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <fstream>
#include <string>

#include "include/ntcore.h"
#include "include/networktables/NetworkTable.h"


/*Usage:
./Camera-Vision-2016 [option]
Options:
quiet - don't give any output (except, y'know, for the camera stuff at the beginning)
cal - bring up calibration info - threshold window, slider bars, and color view (by the way, you can press spacebar to make the changes persistant now!)
view - just bring up a video feed (also outputs data to the terminal, like pretty much everything else)

VNC:
To start the VNC server on the Jetson, ssh into it (the default static IP address is 10.24.10.4) and run vncserver. Then on the drive laptop connect to 10.24.10.4:5901.
The password for the ubuntu user on the Jetson is capsrobotics, but the password for the VNC is capsrobo

Hey! In order to make the code run at startup, just copy camera-job.conf to /etc/init*/


using namespace cv;
using namespace std;

Rect findBiggestBlob(Mat& matImage);//this function finds the bounding rectangle of the largest contiguous region in the image
void getDevices(string& vid, string& usb);//get the device names from /dev
int sgn(double num);

const double PI = 3.141592653589793238462643383279;
const double hor_deg = 0.07914;//in DEGREES PER PIXEL. Horizontal field of view is 50.6496 degrees
const double vert_deg = 0.08189;//in DEGREES PER PIXEL. Vertical field of view is 39.3072 degrees

int main(int argc, char *argv[]){

	string video_device = "/dev/video0";
	//string usb_device = "/dev/ttyUSB0";

	//getDevices(video_device,usb_device);
	//return 0;

	system(("fswebcam -d "+video_device+" -c /home/ubuntu/Downloads/Camera-Vision-2016/cam.conf").c_str());//configure camera. It runs every time because I don't know how persistant the changes are
	VideoCapture camera(0);//initialize camera
	if (!camera.isOpened()){//if the camera doesn't load, quit
		cout << "Camera device ("+video_device+") did not load. Make sure the right device from /dev is selected!\n";
		return -1;
	}

	if (argc == 2){//check command line args
		if (string(argv[1]) == "cal"){
			cout << "opening calibration window\n";
			namedWindow("ctrl",WINDOW_AUTOSIZE);//control window for calibrating the camera
		}
	}

	Mat screen_cap;//camera image
	Mat hsv_img;//camera image converted to hsv
	Mat thresholded;//camera image thresholded

	//high and low hsv threshold settings, to be changed when calibrating
	int Hlow;
	int Hhigh;
	int Slow;
	int Shigh;
	int Vlow;
	int Vhigh;

	//open calibration file and read the values into the variables
	ifstream hsv_file;
	hsv_file.open("/home/ubuntu/Downloads/Camera-Vision-2016/hsv.conf");
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
	double camera_height = 26.5;//preset height of the camera, in INCHES. I don't know where we're mounting the camera on the robot yet
	double camera_angle = 38;//angle of deviance from the horizontal, in DEGREES. Should be 42 officially
	double length = 20;//width of the goal in INCHES
	double ball_diameter = 10;
	double offset = -4.5;//offset of the camera from the center of the robot in INCHES

	float distance;//distance camera is from goal, to be calculated
	double phi;//vertical angle of deviance the sightline of the camera has from the bottom of the goal
	float theta;//horizontal angle of deviance the sightline of the camera has from the center of the goal
	double shooter_angle;//the angle the robot can turn to still have the line of sight inside the goal
	double theta_final;//final horizontal angle
	double distance_final;//final distance

	/*char send_buffer[8];//buffer to send over serial
	int output_port = open(usb_device.c_str(), O_RDWR|O_NOCTTY);//open the usb serial connection
	termios port_options;//serial object
	
	if (output_port == -1){//if the serial connection doesn't open, rip
		cout << "USB serial device ("+usb_device+") failed to load. Make sure the right device from /dev is selected!\n";
		return -1;
	}

	//set all the attributes for the serial connection
	tcgetattr(output_port, &port_options);
	cfsetispeed(&port_options, B9600);
	cfsetospeed(&port_options, B9600);
	port_options.c_cflag &= ~PARENB;
	port_options.c_cflag &= ~CSTOPB;
	port_options.c_cflag &= ~CSIZE;
	port_options.c_cflag |= CS8;
	tcsetattr(output_port, TCSANOW, &port_options);*/

	NetworkTable::SetClientMode();
	NetworkTable::SetIPAddress("10.24.10.2\n");
	auto nt = NetworkTable::GetTable("Jetson");

	while (true){
		camera.read(screen_cap);//get the current frame

		cvtColor(screen_cap,hsv_img,CV_BGR2HSV);//convert the frame to HSV, because it's easier to threshold (theoretically)

		inRange(hsv_img,Scalar(Hlow,Slow,Vlow),Scalar(Hhigh,Shigh,Vhigh),thresholded);//threshold it

		//blur it a little to get rid of noise
		erode(thresholded,thresholded,getStructuringElement(MORPH_ELLIPSE,Size(5,5)));
		dilate(thresholded,thresholded,getStructuringElement(MORPH_ELLIPSE,Size(5,5)));
		dilate(thresholded,thresholded,getStructuringElement(MORPH_ELLIPSE,Size(5,5)));
		erode(thresholded,thresholded,getStructuringElement(MORPH_ELLIPSE,Size(5,5)));

		friggin_box = findBiggestBlob(thresholded);//get the bounding box of the biggest goal
		rectangle(screen_cap,friggin_box,Scalar(0,0,255));//draw it onto the screen because I want to

		//show the image and the thresholded image, only necessary for calibration
		if (argc == 2){
			if ((string(argv[1]) == "cal")||(string(argv[1]) == "view"))
				imshow("screen_cap",screen_cap);
			if (string(argv[1]) == "cal")
				imshow("thresholded",thresholded);
		}

		//calculate the center of the box, the distance from the goal, and the angle of deviance of the sightline
		center_x = friggin_box.x + (friggin_box.width/2.0);
		center_y = friggin_box.y + (friggin_box.height/2.0);
		phi = (240 - friggin_box.y)*vert_deg;
		distance = (goal_height-camera_height)/tan((phi+camera_angle)*(PI/180.0));//distance from the nearest point of the goal
		distance += sqrt(pow(length/2.0,2) - pow(distance*tan((friggin_box.width/2.0)*hor_deg*(PI/180.0)),2));//account for angle of approach
		theta = (center_x-320)*hor_deg;//in DEGREES. Positive value of theta indicates the robot must turn in the COUNTERCLOCKWISE direction because that's how math works

		theta_final = (180.0/PI)*atan( (distance*sin( (90-theta)*(PI/180.0) )) / ( (distance*cos( (90-theta)*(PI/180.0) )) + offset ) );
		theta_final = sgn(theta_final)*(90-fabs(theta_final));
		distance_final = sqrt(pow(distance*sin((90-theta)*(PI/180.0)),2)+pow((distance*cos((90-theta)*(PI/180.0)))+offset,2));

		shooter_angle = hor_deg*(2.0/5.0)*friggin_box.width;

		if (fabs(theta_final) < (shooter_angle-90+acos(ball_diameter/sqrt(pow(distance_final,2)+pow(((length-4)/2.0),2))))){
			theta_final = 0;
		}

		//memcpy(send_buffer, &theta_final, 4);
		//memcpy(send_buffer + 4, &distance_final, 4);

		if (argc == 2){
			if (string(argv[1]) != "quiet"){
			cout << "DISTANCE:" << setw(9) << distance_final << "   ANGLE:" << setw(9) << theta_final << '\n';
			}
		}
		else{
			cout << "DISTANCE:" << setw(9) << distance_final << "   ANGLE:" << setw(9) << theta_final << '\n';
		}
		nt->PutNumber("distance", distance_final);
		nt->PutNumber("angle", theta_final);
		//write(output_port, send_buffer, 8);

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

void getDevices(string& vid, string& usb){
	system("ls /dev > devices.txt");
	ifstream devices;
	devices.open("devices.txt");
	string slash_dev;

	char c;
	while (devices.get(c))
		slash_dev += c;

}

int sgn(double num){
	return num == 0 ? 0 : num / fabs(num);
}

