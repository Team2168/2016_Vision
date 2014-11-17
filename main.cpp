#define _USE_MATH_DEFINES
#define MIN_WIDTH 120
#define Y_IMAGE_RES 240
#define VIEW_ANGLE 34.8665269
#define AUTO_STEADY_STATE 1.9 //seconds

#include <stdio.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <ctime>
#include <iostream>
#include <sstream>

#include <pthread.h>
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
	bool Debug;
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
	bool validFrame;

	//camera bool
	bool cameraConnected;

	int targetLeftOrRight;
	int lastTargerLorR;
	int hotLeftOrRight;
	double targetDistance;

};

//function declarations
//TODO: add pre- and post- comments for each function
void parseCommandInputs(int argc, const char* argv[], ProgParams &params);
Mat GetOriginalImage(const ProgParams& params);
void initializeParams(ProgParams& params);
double diffClock(timespec start, timespec end);
Mat ThresholdImage(Mat img);
void findTarget(Mat original, Mat thresholded, Target& targets, const ProgParams& params);
void NullTargets(Target& target);
void CalculateDist(Target& targets);

//Threaded TCP Functions
void *TCP_thread(void *args);
void *TCP_Send_Thread(void *args);
void *TCP_Recv_Thread(void *args);
void error(const char *msg);

//Threaded Video Capture Function
void *VideoCap(void *args);

//Threaded Counter Function
void *HotGoalCounter(void *args);

//GLOBAL CONSTANTS
const double PI = 3.141592653589793;

//Thresholding parameters
int minR = 0;
int maxR = 30;
int minG = 150;
int maxG = 255;
int minB = 0;
int maxB = 30;

//Target Ratio Ranges
double MinHRatio = 1.5;
double MaxHRatio = 6.6;

double MinVRatio = 1.5;
double MaxVRatio = 8.5;

int MAX_SIZE = 255;

//Some common colors to draw with
const Scalar RED = Scalar(0, 0, 255),
			BLUE = Scalar(255, 0, 0),
			GREEN = Scalar(0, 255, 0),
			ORANGE = Scalar(0, 128, 255),
			YELLOW = Scalar(0, 255, 255),
			PINK = Scalar(255, 0,255),
			WHITE = Scalar(255, 255, 255);

//GLOBAL MUTEX LOCK VARIABLES
pthread_mutex_t targetMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t matchStartMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t frameMutex = PTHREAD_MUTEX_INITIALIZER;


//Thread Variables
pthread_t TCPthread;
pthread_t TCPsend;
pthread_t TCPrecv;
pthread_t MJPEG;
pthread_t AutoCounter;

//TCP Steam
tcp_client client;

//Store targets in global variable
Target targets;
Mat frame;

//Global Timestamps for auto
struct timespec autoStart, autoEnd;


//Control process thread exectution
bool progRun;

int main(int argc, const char* argv[])
{

	//Read command line inputs to determine how the program will execute
	ProgParams params;
	parseCommandInputs(argc, argv, params);

	//start mjpeg stream thread
	pthread_create(&MJPEG, NULL, VideoCap, &params);

	//Create Local Processing Image Variables
	Mat img, thresholded, output;


	//initialize variables so processing loop is false;
	targets.matchStart = false;
	targets.validFrame = false;
	targets.hotLeftOrRight = 0;
	progRun = false;


	//start TCP Server
	pthread_create(&TCPthread, NULL, TCP_thread, &params);

	struct timespec start, end;

	double processTimer = 0;
	int processCount = 1;

	//run loop forever
	while (true)
	{
		//check if program is allowed to run
		//this bool, is enabled by the mjpeg thread
		//once it is up to 10fps
		if (progRun)
		{
			//start clock to determine our processing time;
			clock_gettime(CLOCK_REALTIME, &start);

			pthread_mutex_lock(&frameMutex);
			if (!frame.empty())
			{

				frame.copyTo(img);
				pthread_mutex_unlock(&frameMutex);

				thresholded = ThresholdImage(img);

				//Lock Targets and determine goals
				pthread_mutex_lock(&targetMutex);
				findTarget(img, thresholded, targets, params);
				CalculateDist(targets);

				if(params.Debug)
				{
					cout<<"Vert: "<<targets.VertGoal<<endl;
					cout<<"Horiz: "<<targets.HorizGoal<<endl;
					cout<<"Hot Goal: "<<targets.HotGoal<<endl;
					cout<<"Dist:" <<targets.targetDistance<<endl<<endl;
				}
				pthread_mutex_unlock(&targetMutex);

				clock_gettime(CLOCK_REALTIME, &end);

				if(params.Timer)
				{
					processTimer += diffClock(start,end);
					cout << "It took " << diffClock(start,end) << " seconds to process frame \n";
					cout << " Avg processing time: "<< processTimer/processCount << " seconds \n" ;
					processCount += 1;
				}

				}

			if(params.Visualize)
				waitKey(5);

			//usleep(1000); //20000 sleep for 5ms); // run 40 times a second


		}
	}

	//if we end the process code, wait for threads to end
	pthread_join(TCPthread, NULL);
	pthread_join(TCPsend, NULL);
	pthread_join(TCPrecv, NULL);

	pthread_join(MJPEG, NULL);

	//done
	return 0;

}

///////////////////FUNCTIONS/////////////////////

/**
 * This function uses the law of lense projection to
 * estimate the distance to an object of known height only
 * using a single camera.
 *
 * This function uses only the vertical target height, the
 * pixel height of the image, and the view angle of the
 * camera lense.
 */
void CalculateDist(Target& targets)
{
	//vertical target is 32 inches fixed
	double targetHeight = 32.0;

	//get vertical pixels from targets
	int height = targets.VerticalTarget.height;

	//d = Tft*FOVpixel/(2*Tpixel*tanΘ)
	targets.targetDistance = Y_IMAGE_RES * targetHeight
			/ (height * 12 * 2 * tan(VIEW_ANGLE * PI / (180 * 2)));
}

/**
 * This function scans through an image and determins
 * if rectangles exist which match the profile of a
 * "Hot Goal".
 *
 * The "Hot Goal" is specific to the 2014 FRC game
 * and is identified as a horizontal and vertical target
 * in the same frame, with known width and height.
 */
void findTarget(Mat original, Mat thresholded, Target& targets, const ProgParams& params)
{

	vector<Vec4i> hierarchy;
	vector<vector<Point> > contours;



	//Find rectangles
	findContours(thresholded, contours, hierarchy, RETR_EXTERNAL,
			CHAIN_APPROX_SIMPLE);

	if(params.Debug)
	{
	cout << "Contours: " << contours.size() << endl;
	cout << "Hierarchy: " << hierarchy.size() << endl;
	}

	//run through all contours and remove small contours
	unsigned int contourMin = 6;
	for (vector<vector<Point> >::iterator it = contours.begin();
			it != contours.end();)
	{
		//cout<<"Contour Size: "<<it->size()<<endl;
		if (it->size() < contourMin)
			it = contours.erase(it);

		else
			++it;

	}

	//Vector for Min Area Boxes
	vector<RotatedRect> minRect(contours.size());

	/// Draw contours
	Mat drawing = Mat::zeros(original.size(), CV_8UC3);

	NullTargets(targets);

	//run through large contours to see if they are our targerts
	if (!contours.empty() && !hierarchy.empty())
	{

		for (unsigned int i = 0; i < contours.size(); i++)
		{
			//capture corners of contour
			minRect[i] = minAreaRect(Mat(contours[i]));

			if(params.Visualize)
			{

				//if(hierarchy[i][100] != -1)
				//drawContours(original, contours, i, RED, 2, 8, hierarchy, 0,Point());

				//draw a minimum box around the target in green
				Point2f rect_points[4];
				minRect[i].points(rect_points);
				for (int j = 0; j < 4; j++)
					line(original, rect_points[j], rect_points[(j + 1) % 4], BLUE, 1, 8);
			}
			//define minAreaBox
			Rect box = minRect[i].boundingRect();

			double WHRatio = box.width / ((double) box.height);
			double HWRatio = ((double) box.height) / box.width;

			//check if contour is vert, we use HWRatio because it is greater that 0 for vert target
			if ((HWRatio > MinVRatio) && (HWRatio < MaxVRatio))
			{
				targets.VertGoal = true;
				targets.VerticalTarget = box;
				targets.VerticalAngle = minRect[i].angle;
				targets.VerticalCenter = Point(box.x + box.width / 2,
						box.y + box.height / 2);
				targets.Vertical_H_W_Ratio = HWRatio;
				targets.Vertical_W_H_Ratio = WHRatio;

			}
			//check if contour is horiz, we use WHRatio because it is greater that 0 for vert target
			else if ((WHRatio > MinHRatio) && (WHRatio < MaxHRatio))
			{
				targets.HorizGoal = true;
				targets.HorizontalTarget = box;
				targets.HorizontalAngle = minRect[i].angle;
				targets.HorizontalCenter = Point(box.x + box.width / 2,
						box.y + box.height / 2);
				targets.Horizontal_H_W_Ratio = HWRatio;
				targets.Horizontal_W_H_Ratio = WHRatio;
			}

			if (targets.HorizGoal && targets.VertGoal)
			{
				targets.HotGoal = true;

				//determine left or right
				if (targets.VerticalCenter.x < targets.HorizontalCenter.x) //target is right
					targets.targetLeftOrRight = 1;
				else if (targets.VerticalCenter.x > targets.HorizontalCenter.x) //target is left
					targets.targetLeftOrRight = -1;

				targets.lastTargerLorR = targets.targetLeftOrRight;

			}

			if(params.Debug)
			{
				cout<<"Contour: "<<i<<endl;
				cout<<"\tX: "<<box.x<<endl;
				cout<<"\tY: "<<box.y<<endl;
				cout<<"\tHeight: "<<box.height<<endl;
				cout<<"\tWidth: "<<box.width<<endl;
				cout<<"\tangle: "<<minRect[i].angle<<endl;
				cout<<"\tRatio (W/H): "<<WHRatio<<endl;
				cout<<"\tRatio (H/W): "<<HWRatio<<endl;
				cout<<"\tArea: "<<box.height*box.width<<endl;
			}

			//ID the center in yellow
			Point center(box.x + box.width / 2, box.y + box.height / 2);
			line(original, center, center, YELLOW, 3);
			line(original, Point(320/2, 240/2), Point(320/2, 240/2), YELLOW, 3);

		}
		//if(params.Visualize)
			//imshow("Contours", original); //Make a rectangle that encompasses the target
	}
	else
	{
		cout << "No Contours" << endl;
		targets.targetLeftOrRight = 0;
	}

	if(params.Visualize)
				imshow("Contours", original); //Make a rectangle that encompasses the target

	pthread_mutex_lock(&matchStartMutex);
	if (!targets.matchStart)
		targets.hotLeftOrRight = targets.targetLeftOrRight;
	pthread_mutex_unlock(&matchStartMutex);

}

/**
 * This function performs numerous filtering on
 * a color image in order to only return
 * areas of interest based on their color
 *
 */
Mat ThresholdImage(Mat original)
{
	//Local Temp Image
	Mat thresholded;

	//Threshold image to remove non-green objects
	inRange(original, Scalar(minB, minG, minR), Scalar(maxB, maxG, maxR),
			thresholded);

	//smooth edges
	blur(thresholded, thresholded, Size(3, 3));

	//Additional filtering if needed
	//Canny(thresholded, thresholded, 100, 100, 3);
	//blur(thresholded, thresholded, Size(5, 5));

	//return image
	return thresholded;

}

/**
 * This functions "zeros", the targets identified
 * so that a clean slate can be used to determine
 * if the next image contains targets as well.
 */
void NullTargets(Target& target)
{

	target.HorizontalAngle = 0.0;
	target.VerticalAngle = 0.0;
	target.Horizontal_W_H_Ratio = 0.0;
	target.Horizontal_H_W_Ratio = 0.0;
	target.Vertical_W_H_Ratio = 0.0;
	target.Vertical_H_W_Ratio = 0.0;
	target.targetDistance = 0.0;
	target.targetLeftOrRight = 0;
	target.lastTargerLorR = 0;

	target.HorizGoal = false;
	target.VertGoal = false;
	target.HotGoal = false;

}
void initializeParams(ProgParams& params)
{
	params.Debug = false;
	params.From_Camera = false;
	params.From_File = false;
	params.Timer = false;
	params.Visualize = false;

}

/**
 * This function parses the command line inputs and determines
 * the runtime parameters the program should use as specified
 * by the user.
 */
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

		initializeParams(params);

		for (int i = 1; i < argc; i++)
		{ /* We will iterate over argv[] to get the parameters stored inside.
		 * Note that we're starting on 1 because we don't need to know the
		 * path of the program, which is stored in argv[0] */

			if ((string(argv[i]) == "-f") && (i + 1 < argc)) //read from file
			{
				// We know the next argument *should* be the filename:
				params.IMAGE_FILE = string(argv[i + 1]);
				params.From_Camera = false;
				params.From_File = true;
				i++;
			}
			else if ((string(argv[i]) == "-c") && (i + 1 < argc)) //camera IP
			{
				//params.CAMERA_IP = string(argv[i + 1]);
				params.From_Camera = true;
				params.From_File = false;
				i++;
			}
			else if ((string(argv[i]) == "-s") && (i + 1 < argc)) //robot TCP SERVER IP
			{
				params.ROBOT_IP = string(argv[i + 1]);
				i++;
			}
			else if ((string(argv[i]) == "-p") && (i + 1 < argc)) //robot TCP SERVER PORT
			{
				params.ROBOT_PORT = string(argv[i + 1]);
				i++;
			}
			else if (string(argv[i]) == "-t") //Enable Timing
			{
				params.Timer = true;
			}
			else if (string(argv[i]) == "-v") //Enable Verbose output
			{
				params.Visualize = true;
			}
			else if (string(argv[i]) == "-debug") //Enable debug output
			{
				params.Debug = true;
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
				std::cout
						<< "Not enough or invalid arguments, please try again.\n";
				sleep(2000);
				exit(0);
			}

		}

	}
}
/**
 * This function either gets an image from the camera
 * loads from a file
 *
 * The condition is determined by variables within the
 * program struct.
 *
 * The image returned is then used for processing.
 *
 * THIS FUNCTION IS OBSOLTETE AND HAS BEEN REPLACED
 * BY AN FFMPEG STREAM FUNCTION
 */
Mat GetOriginalImage(const ProgParams& params)
{
	Mat img;

	if (params.From_Camera)
	{

		system("wget -q http://10.21.68.90/jpg/image.jpg -O capturedImage.jpg");

		//load downloaded image
		img = imread("capturedImage.jpg");

	}
	else if (params.From_File)
	{
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

double diffClock(timespec start, timespec end)
{
 return	(end.tv_sec - start.tv_sec) + (double) (end.tv_nsec - start.tv_nsec)/ 1000000000.0f;
}

/**
 * This function creates a TCP stream between the cRIO and the
 * beaglebone.
 *
 * Once the stream is established it will automatically
 * create two new threads, one to send a predetermined message
 * to the cRIO, and another to receive a predetermined message
 * from the cRIO.
 */

void *TCP_thread(void *args)
{
	ProgParams *struct_ptr = (ProgParams *) args;

	//string ip = struct_ptr->ROBOT_IP;
	//int port = atoi(struct_ptr->ROBOT_PORT.c_str());

	string ip = "10.21.68.2";
	int port = 1111;

	//connect to host
	client.conn(ip, port);

	//create thread to send messages
	pthread_create(&TCPsend, NULL, TCP_Send_Thread, NULL);

	//create thread to recv messages
	pthread_create(&TCPrecv, NULL, TCP_Recv_Thread, NULL);

	/* the function must return something - NULL will do */
	return NULL;

}

/**
 * This function sends data to the cRIO over TCP.
 *
 * This function assumes the TCP stream has already been created.
 *
 * Currently the only data we receive from the CRIO is match start
 * boolean which allows us to detrmine the time autonomous starts.
 *
 * This function should be ran it its own thread. It uses a sleep
 * function to pause execution.
 */

void *TCP_Send_Thread(void *args)
{
	int count = 0;
	while (true)
	{
		//Create a string which has following information
		//MatchStart, HotGoal, Distance, message #

		pthread_mutex_lock(&targetMutex);
		pthread_mutex_lock(&matchStartMutex);
		stringstream message;

		//create string stream message;
		message << targets.matchStart << ","<< targets.validFrame << "," << targets.HotGoal << ","
				<< targets.cameraConnected << "," << progRun << ","<< targets.hotLeftOrRight << ","
				<< targets.targetDistance << "," << count << "\n";

		//send message over pipe
		client.send_data(message.str());
		pthread_mutex_unlock(&matchStartMutex);
		pthread_mutex_unlock(&targetMutex);

		count++;
		usleep(50000); //  run ~20 times a second

	}

	return NULL;

}

/**
 * This function captures data from the cRIO over TCP and saves it in a
 * variable.
 *
 * NOTE: THIS FUNCTION BLOCKS WAITING FOR DATA ON THE PIPE TO BE
 * RECEIVED. IF YOU PASS IT A MUTABLE LOCK, IT WILL BLOCK ON
 * THAT LOCK UNTIL A /n CHARACTER IS RECEIVED. POSSIBLY BLOCKING
 * ANY OTHER THREAD USING THAT LOCK.
 *
 * YOU CAN AVOID THIS BY MAKING SURE THE CRIO PASSES DATA TO THE BONE
 * FASTER OR AS FAST AS THIS FUNCTION LOOPS.
 *
 * ALSO BE CAREFUL WHAT MUTABLE LOCKS ARE USED.
 *
 * This function assumes the TCP stream has already been created.
 *
 * Currently the only data we receive from the CRIO is match start
 * boolean which allows us to determine the time autonomous starts.
 *
 * This function should be ran it its own thread. It uses a sleep
 * function to pause execution.
 */

void *TCP_Recv_Thread(void *args)
{
	int count1 = 0;
	int count2 = 0;
	struct timespec end;

	while (true)
	{
		//Set Match State, should be single int
//		pthread_mutex_lock(&matchStartMutex);
		targets.matchStart = atoi(client.receive(5).c_str());

		if(!targets.matchStart)
		{
			count1=0;
			count2=0;
			targets.validFrame = false;

		}

		//once the match starts, we start a timer and run it in
		//a new thread, we use a count variable so we only run this once
		if(targets.matchStart && count1==0)
		{
			clock_gettime(CLOCK_REALTIME, &autoStart);
			count1++;
			pthread_create(&AutoCounter, NULL, HotGoalCounter, args);
		}


		clock_gettime(CLOCK_REALTIME, &end);

		//Only set validFrame after we wait a certain amount of time, and after
		//process thread starts
		if(targets.matchStart && diffClock(autoStart,end)>=AUTO_STEADY_STATE && progRun && count2==0 )
		{
			targets.validFrame = true;
			count2++;
		}


//		pthread_mutex_unlock(&matchStartMutex);

		usleep(200000); // run 5 times a second

	}

	return NULL;

}

/**
 * This function uses FFMPEG codec apart of openCV to open a
 * MJPEG stream and buffer it. This function should be ran
 * in its own thread so it can run as fast as possibe and store frames.
 *
 * A mutable lock should be used in another thread to copy the latest frame
 *
 * Note: Opening the stream blocks execution. Also
 * Based on my own tests it appears the beaglebone can capture
 * frames at 30fps with 320 x 240 resolution, however
 * the framerate needs to be reduced to allow for processing time.
 *
 * Only run the camera as 10FPS, with a 10kbs limit per frame
 */
void *VideoCap(void *args)
{
	//copy passed in variable to programStruct
	ProgParams *struct_ptr = (ProgParams *) args;

	if (struct_ptr->From_File)
	{
		cout<<"Loading Image from file"<<endl;

		//read img and store it in global variable
		pthread_mutex_lock(&frameMutex);
		frame = imread(struct_ptr->IMAGE_FILE);
		pthread_mutex_unlock(&frameMutex);

		cout<<"File Loaded: Starting Processing Thread"<<endl;
		progRun = true;


	}

	else if(struct_ptr->From_Camera)
	{
		//create timer variables
		struct timespec start, end, bufferStart, bufferEnd;

		//seconds to wait for buffer to clear before we start main process thread
		int waitForBufferToClear = 15;

		//start timer to time how long it takes to open stream
		clock_gettime(CLOCK_REALTIME, &start);

		cv::VideoCapture vcap;

		// This works on a AXIS M1013
		const std::string videoStreamAddress = "http://10.21.68.90/mjpg/video.mjpg";

		std::cout<<"Trying to connect to Camera stream... at: "<<videoStreamAddress<<std::endl;

		int count = 1;

		//open the video stream and make sure it's opened
		while (!vcap.open(videoStreamAddress))
		{
			std::cout << "Error connecting to camera stream, retrying " << count<< std::endl;
			count++;
			usleep(100000);
		}

			//Stream started
			cout << "Successfully connected to Camera Stream" << std::endl;

			//set true boolean
			pthread_mutex_lock(&targetMutex);
			targets.cameraConnected = true;
			pthread_mutex_unlock(&targetMutex);

			//end clock to determine time to setup stream
			clock_gettime(CLOCK_REALTIME, &end);

			cout << "It took " << diffClock(start,end) << " seconds to set up stream " << endl;

			clock_gettime(CLOCK_REALTIME, &bufferStart);


		cout<<"Waiting for stream buffer to clear..."<<endl;

		int timerCount = 1;
		double sumTime = 0;

		//run in continuous loop
		while (true)
		{

			//start timer to get time per frame
			clock_gettime(CLOCK_REALTIME, &start);

			//read frame and store it in global variable
			pthread_mutex_lock(&frameMutex);
			vcap.read(frame);
			pthread_mutex_unlock(&frameMutex);

			//end timer to get time per frame
			clock_gettime(CLOCK_REALTIME, &end);



			if(struct_ptr->Timer)
			{
				sumTime += diffClock(start,end);
				cout << "It took FFMPEG " << diffClock(start,end) << " seconds to grab stream \n";
				cout<< "Avg FFMPEG Time Per Frame" << sumTime/timerCount << "seconds \n";
				timerCount +=1;
			}

			//end timer to get time since stream started
			clock_gettime(CLOCK_REALTIME, &bufferEnd);
			double bufferDifference = diffClock(bufferStart, bufferEnd);

			//The stream takes a while to start up, and because of it, images from the camera
			//buffer. We don't have a way to jump to the end of the stream to get the latest image, so we
			//run this loop as fast as we can and throw away all the old images. This wait, waits some number of seconds
			//before we are at the end of the stream, and can allow processing to begin.
			if ((bufferDifference >= waitForBufferToClear) && !progRun)
			{
				cout<<"Buffer Cleared: Starting Processing Thread"<<endl;
				progRun = true;

			}

					//usleep(20000); //sleep for 5ms
		}

	}

	return NULL;
}

void *HotGoalCounter(void *args)
{

	//If this method started, then the match started


	while (true)
	{
		clock_gettime(CLOCK_REALTIME, &autoEnd);
		double timeNow = diffClock(autoStart,autoEnd);
		if(timeNow<5)
		{
			pthread_mutex_lock(&targetMutex);
			if(targets.targetLeftOrRight == 0)
				targets.hotLeftOrRight = targets.lastTargerLorR * -1;
			else
				targets.hotLeftOrRight = targets.targetLeftOrRight;
			pthread_mutex_unlock(&targetMutex);

			cout<<"this side hot"<<endl;
		}
		else if(timeNow<10)
		{
			//Auto has been running for 5 seconds, so the other side is hot
			//we update the variable to switch to other side
			pthread_mutex_lock(&targetMutex);
			targets.hotLeftOrRight = targets.hotLeftOrRight * -1;
			pthread_mutex_unlock(&targetMutex);

			cout<<"otherside hot"<<endl;
		}
		else if (timeNow >= 10)
		{
			//Auto is over, no more hot targets, end thread
			pthread_mutex_lock(&targetMutex);
			targets.hotLeftOrRight = 0;
			pthread_mutex_unlock(&targetMutex);
			cout<<"auto over"<<endl;
			break;
		}

		usleep(50000); // run 10 times a second

	}

	return NULL;

}

