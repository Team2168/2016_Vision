#define FROM_FILE
#define VISUALIZE
#define TIMING
#define _USE_MATH_DEFINES
#define MIN_WIDTH 120
#define Y_IMAGE_RES 480
#define VIEW_ANGLE 34.8665269

#include <stdio.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <ctime>
#include <iostream>
#include <sstream>


#include <pthread.h>
//#include <unistd.h>
#include "tcp_client.h"


using namespace cv;
using namespace std;


//struct to define program execution variables passed in from the command line
struct ProgParams
{
	string ROBOT_IP;
	string ROBOT_PORT;
	string CAMERA_IP;
	string IMAGE_FILE;

	bool From_Camera;
	bool From_File;
	bool Visualize;
	bool Timer;
};


//Stuct to hold information about targets found
struct Target
{
	Rect HorizontalTarget;
	Rect VerticalTarget;

	double HorizontalAngle;
	double VerticalAngle;
	double Horizontal_W_H_Ratio;
	double Horizontal_H_W_Ratio;
	double Vertical_W_H_Ratio;
	double Vertical_H_W_Ratio;


	Point HorizontalCenter;
	Point VerticalCenter;

	bool HorizGoal;
	bool VertGoal;
	bool HotGoal;
	bool matchStart;

	double targetDistance;

};

//function declarations
//TODO: add pre- and post- comments for each function
void parseCommandInputs(int argc, const char* argv[], ProgParams &params);
Mat GetOriginalImage(const ProgParams& params);
double diffclock(clock_t clock1,clock_t clock2);
Mat ThresholdImage(Mat img);
Target findTarget(Mat original, Mat thresholded, Target& targets);
void NullTargets(Target& target);
void CalculateDist(Target& targets);

//Threaded TCP Functions
void *TCP_thread(void *args);
void *TCP_Send_Thread(void *args);
void *TCP_Recv_Thread(void *args);
void error(const char *msg);

//GLOBAL CONSTANTS
const double PI = 3.141592653589793;


//Thresholding parameters
int minR = 0;
int	maxR = 30;
int	minG = 150;
int	maxG = 255;
int	minB = 0;
int	maxB = 30;

//Target Ratio Ranges
double MinHRatio = 2.8;
double MaxHRatio = 6.6;

double MinVRatio = 3.2;
double MaxVRatio = 8.5;

int MAX_SIZE = 255;

//Some common colors to draw with
const Scalar RED    = Scalar(0,     0, 255),
		GREEN  = Scalar(0,   255,   0),
		ORANGE = Scalar(0,   128, 255),
		YELLOW = Scalar(0,   255, 255),
		PINK   = Scalar(255,   0, 255),
		WHITE  = Scalar(255, 255, 255);


//GLOBAL VARIABLES
pthread_mutex_t targetMutex = PTHREAD_MUTEX_INITIALIZER;;


pthread_t TCPthread;
pthread_t TCPsend;
pthread_t TCPrecv;

tcp_client client;

//store targets
Target targets;


int main(int argc, const char* argv[])
{

	//Read command line inputs to determine how the program will execute
	ProgParams params;
	parseCommandInputs(argc, argv, params);

	//Create Program Image Vairables
	Mat img, thresholded, output;

	//create windows
	//namedWindow("Original", WINDOW_AUTOSIZE);
	//namedWindow("Treshold", WINDOW_AUTOSIZE);

	targets.matchStart = false;
	bool progRun = true;

	//start TCP Server
	pthread_create(&TCPthread, NULL, TCP_thread, &params);



	while(progRun)
	{
		clock_t start_time, end_time = 0.0;
		start_time = clock();


		img = GetOriginalImage(params);
	//	imshow("Original", img);

		thresholded = ThresholdImage(img);
	//	imshow("Treshold", thresholded);

		//Lock Targets and determine goals
		pthread_mutex_lock (&targetMutex);
		findTarget(img, thresholded, targets);
		CalculateDist(targets);

		//		cout<<"Vert: "<<targets.VertGoal<<endl;
		//		cout<<"Horiz: "<<targets.HorizGoal<<endl;
		//		cout<<"Hot Goal: "<<targets.HotGoal<<endl;
		//		cout<<"Dist:" <<targets.targetDistance<<endl<<endl;
		pthread_mutex_unlock (&targetMutex);

		usleep(25000); // run 40 times a second

		end_time = clock();
	//	cout << "Image proc. time: " << double(diffclock(end_time,start_time)) << "ms" << endl;

#ifdef VISUALIZE
		//halt execution when esc key is pressed
		if(waitKey(30) >= 0)
			progRun = 0;
#endif


	}

	//if we end the camera code, wait for threads to end
	pthread_join(TCPthread, NULL);
	pthread_join(TCPsend, NULL);
	pthread_join(TCPrecv, NULL);

	//done
	return 0;

}


///////////////////FUNCTIONS/////////////////////

void CalculateDist(Target& targets)
{
	//vertical target is 32 inches fixed
	double targetHeight = 32.0;

	//get vertical pixels from targets
	int height = targets.VerticalTarget.height;

	//d = Tft*FOVpixel/(2*Tpixel*tanΘ)
	targets.targetDistance = Y_IMAGE_RES * targetHeight / (height * 12 * 2 * tan(VIEW_ANGLE*PI/(180*2)));
}


void findTarget(Mat original, Mat thresholded, Target& targets)
{

	vector<Vec4i> hierarchy;
	vector<vector<Point> > contours;



	/// Show in a window
	//namedWindow( "Contours", WINDOW_AUTOSIZE );


	//Find rectangles
	findContours(thresholded, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

	//	cout<<"Contours: "<<contours.size()<<endl;
	//	cout<<"Hierarchy: "<<hierarchy.size()<<endl;


	//run through all contours and remove small contours
	unsigned int contourMin = 20;
	for (vector<vector<Point> >::iterator it = contours.begin(); it!=contours.end(); )
	{
		//		cout<<"Contour Size: "<<it->size()<<endl;
		if (it->size()<contourMin)
			it=contours.erase(it);
		else
			++it;
	}

	//Vector for Min Area Boxes
	vector<RotatedRect> minRect(contours.size());

	/// Draw contours
	Mat drawing = Mat::zeros(original.size(), CV_8UC3 );

	NullTargets(targets);

	//run through large contours to see if they are our targerts
	if(!contours.empty() && !hierarchy.empty())
	{



		for(unsigned int i = 0; i < contours.size(); i++)
		{
			//capture corners of contour
			minRect[i] = minAreaRect(Mat(contours[i]));

			//if(hierarchy[i][100] != -1)
			drawContours( drawing, contours, i, RED, 2, 8, hierarchy, 0, Point());



			//draw a minimum box around the target in green
			Point2f rect_points[4];
			minRect[i].points(rect_points);
			for (int j = 0; j < 4; j++)
				line(drawing,rect_points[j],rect_points[(j+1)%4],GREEN,1,8);

			//define minAreaBox
			Rect box = minRect[i].boundingRect();


			double WHRatio = box.width/((double)box.height);
			double HWRatio = ((double)box.height)/box.width;

			//check if contour is vert, we use HWRatio because it is greater that 0 for vert target
			if ((HWRatio > MinVRatio) && (HWRatio < MaxVRatio))
			{
				targets.VertGoal = true;
				targets.VerticalTarget = box;
				targets.VerticalAngle = minRect[i].angle;
				targets.VerticalCenter = Point(box.x + box.width/2, box.y + box.height/2);
				targets.Vertical_H_W_Ratio = HWRatio;
				targets.Vertical_W_H_Ratio = WHRatio;

			}
			//check if contour is horiz, we use WHRatio because it is greater that 0 for vert target
			else if ((WHRatio > MinHRatio) && (WHRatio < MaxHRatio))
			{
				targets.HorizGoal = true;
				targets.HorizontalTarget = box;
				targets.HorizontalAngle = minRect[i].angle;
				targets.HorizontalCenter = Point(box.x + box.width/2, box.y + box.height/2);
				targets.Horizontal_H_W_Ratio = HWRatio;
				targets.Horizontal_W_H_Ratio = WHRatio;
			}

			if(targets.HorizGoal && targets.VertGoal)
				targets.HotGoal = true;

			//			cout<<"Contour: "<<i<<endl;
			//			cout<<"\tX: "<<box.x<<endl;
			//			cout<<"\tY: "<<box.y<<endl;
			//			cout<<"\tHeight: "<<box.height<<endl;
			//			cout<<"\tWidth: "<<box.width<<endl;
			//			cout<<"\tangle: "<<minRect[i].angle<<endl;
			//			cout<<"\tRatio (W/H): "<<WHRatio<<endl;
			//			cout<<"\tRatio (H/W): "<<HWRatio<<endl;




			//ID the center in yellow
			Point center(box.x + box.width/2, box.y + box.height/2);
			line(drawing, center, center, YELLOW, 3);
			line(drawing ,Point(320,240),Point(320,240),YELLOW,3);

		}




		//imshow( "Contours", drawing );//Make a rectangle that encompasses the target
	}
	else
		cout<<"No Contours"<<endl;


}



Mat ThresholdImage(Mat original)
{
	//Local Temp Image
	Mat thresholded;

	//Threshold image to remove non-green objects
	inRange(original, Scalar(minB, minG, minR), Scalar(maxB, maxG, maxR), thresholded);

	//smooth edges
	blur(thresholded, thresholded, Size(3, 3));

	//Additional filtering if needed
	//Canny(thresholded, thresholded, 100, 100, 3);
	//blur(thresholded, thresholded, Size(5, 5));

	//return image
	return thresholded;

}

void NullTargets(Target& target)
{

	target.HorizontalAngle = 0.0;
	target.VerticalAngle = 0.0;
	target.Horizontal_W_H_Ratio = 0.0;
	target.Horizontal_H_W_Ratio = 0.0;
	target.Vertical_W_H_Ratio = 0.0;
	target.Vertical_H_W_Ratio = 0.0;
	target.targetDistance = 0.0;

	target.HorizGoal = false;
	target.VertGoal = false;
	target.HotGoal = false;
}

void parseCommandInputs(int argc, const char* argv[], ProgParams& params)
{
	//todo: define all input flags
	if (argc < 2)
	{ // Check the value of argc. If not enough parameters have been passed, inform user and exit.
		cout << "Usage is:"; // Inform the user of how to use the program
		cin.get();
		exit(0);
	}
	else
	{ // if we got enough parameters...

		for (int i = 1; i < argc; i++)
		{ /* We will iterate over argv[] to get the parameters stored inside.
		 * Note that we're starting on 1 because we don't need to know the
		 * path of the program, which is stored in argv[0] */
			if (i + 1 != argc) // Check that we haven't finished parsing already
			{
				if (string(argv[i]) == "-f") //read from file
				{
					// We know the next argument *should* be the filename:
					params.IMAGE_FILE = string(argv[i + 1]);
					params.From_Camera=false;
					params.From_File=true;
				}
				else if (string(argv[i]) == "-c") //camera IP
				{
					//params.CAMERA_IP = string(argv[i + 1]);
					params.From_Camera=true;
					params.From_File=false;
				}
				else if (string(argv[i]) == "-s") //robot TCP SERVER IP
				{
					params.ROBOT_IP = string(argv[i + 1]);
				}
				else if (string(argv[i]) == "-p") //robot TCP SERVER PORT
				{
					params.ROBOT_PORT = string(argv[i + 1]);
				}
				else if (string(argv[i]) == "-t") //enable timing
				{
					params.Timer = true;
				}
				else if (string(argv[i]) == "-d") //Default Params
				{
					params.ROBOT_PORT = string(argv[i + 1]);
					return;
				}
				else if (string(argv[i]) == "-help") //help
				{
					//todo: cout help on commands
					return;
				}
				else
				{
					std::cout << "Not enough or invalid arguments, please try again.\n";
					sleep(2000);
					exit(0);
				}
			}
		}

	}
}

Mat GetOriginalImage(const ProgParams& params)
{
	Mat img;

	if(params.From_Camera)
	{
		//use wget to download the image from the camera to bone
		system("wget -q http://10.21.68.90/jpg/image.jpg -O capturedImage.jpg");

		//load downloaded image
		img = imread("capturedImage.jpg");

	}
	else if(params.From_File){
		//load image from file
		img = imread(params.IMAGE_FILE);
	}

	return img;
}


void error(const char *msg)
{
	perror(msg);
	exit(0);
}

double diffclock(clock_t clock1,clock_t clock2)
{
	double diffticks=clock1-clock2;
	double diffms=(diffticks*10)/CLOCKS_PER_SEC;
	return diffms;
}

void *TCP_thread(void *args)
{
	ProgParams *struct_ptr = (ProgParams *)args;

	//string ip = struct_ptr->ROBOT_IP;
	//int port = atoi(struct_ptr->ROBOT_PORT.c_str());

	string ip = "10.21.68.2";
	int port = 1111;

	//connect to host
	client.conn(ip , port);

	//create thread to send messages
	pthread_create(&TCPsend, NULL, TCP_Send_Thread, NULL);

	//create thread to recv messages
	pthread_create(&TCPrecv, NULL, TCP_Recv_Thread, NULL);

	/* the function must return something - NULL will do */
	return NULL;

}

void *TCP_Send_Thread(void *args)
{
	int count = 0;
	while(true)
	{
		//Create a string which has following information
		//MatchStart, HotGoal, Distance, message #

		pthread_mutex_lock (&targetMutex);
		stringstream message;


		message<<targets.matchStart<<","<<targets.HotGoal<<","<<targets.targetDistance<<","<<count<<"\n";

		client.send_data(message.str());
		pthread_mutex_unlock (&targetMutex);

		count++;
		usleep(30000); // run 20 times a second

	}

	return NULL;

}

void *TCP_Recv_Thread(void *args)
{
	while(true)
	{
		//Set Match State, should be single int
		pthread_mutex_lock (&targetMutex);
		targets.matchStart = atoi(client.receive(1024).c_str());
		pthread_mutex_unlock (&targetMutex);

		usleep(333333); // run 3 times a second

	}

	return NULL;

}
