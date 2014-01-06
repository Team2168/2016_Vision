#define FROM_FILE
#define VISUALIZE
#define TIMING
#define _USE_MATH_DEFINES
#define MIN_WIDTH 120

#include <stdio.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <ctime>

#include <syslog.h>
#include <sys/stat.h>
#include <queue>
#include <sys/socket.h>
#include <unistd.h>
#include <netinet/in.h>
#include <netinet/in_systm.h>
#include <netinet/ip.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>



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

//function declarations
//TODO: add pre- and post- comments for each function


void parseCommandInputs(int argc, const char* argv[], ProgParams &params);
Mat GetOriginalImage(const ProgParams& params);
double diffclock(clock_t clock1,clock_t clock2);
Mat ThresholdImage(Mat img);
void findTarget(Mat original, Mat thresholded);
void CalculateDist(Mat& img, Mat t_img);

void TCP_thread(void *obj);
void error(const char *msg);

//GLOBAL CONSTANTS
const double pi = 3.141592653589793;
const double HeightGroundToPivot = 38.69;

//Thresholding parameters
int minR = 0;
int	maxR = 30;
int	minG = 0;
int	maxG = 30;
int	minB = 60;
int	maxB = 150;

//Some common colors to draw with
const Scalar RED    = Scalar(0,     0, 255),
             GREEN  = Scalar(0,   255,   0),
             ORANGE = Scalar(0,   128, 255),
             YELLOW = Scalar(0,   255, 255),
             PINK   = Scalar(255,   0, 255),
             WHITE  = Scalar(255, 255, 255);


//GLOBAL VARIABLES
int sockfd;
pthread_mutex_t angleTCPmutex;
pthread_mutex_t distanceTCPmutex;
pthread_t thread;
pthread_t TCPthread;

double TCPangleVal;
double TCPdistanceVal;


vector<char> image_data;
vector<vector<Point> > contours;

CvRect boundbox, target;
Point upperLeft, lowerRight;


int main(int argc, const char* argv[])
{

	//Read command line inputs to determine how the program will execute
	ProgParams params;
	parseCommandInputs(argc, argv, params);

	//Create Program Image Vairables
	Mat img, thresholded, output;

	bool progRun = true;

    while(progRun)
    {

        #ifdef TIMING
            clock_t start_time, end_time = 0.0;
        #endif

        #ifdef TIMING
            start_time = clock();
        #endif


        img = GetOriginalImage(params);


        thresholded = ThresholdImage(img);


        findTarget(img, thresholded);


        CalculateDist(img, thresholded);

        #ifdef TIMING
            end_time = clock();
            cout << "Image proc. time: " << double(diffclock(end_time,start_time)) << "ms" << endl;
        #endif



    #ifdef VISUAIZE
    // Create Window
    NamedWindow("IMAGE", WINDOW_AUTOSIZE);
    #endif

	//cout << "Image center = " << img.size().width/2 << endl;

	#ifdef VISUALIZE

	imshow("IMAGE", img);
        //halt execution when esc key is pressed
        if(waitKey(30) >= 0)
        	progRun = 1;
	#endif
    }

    return 0;

}


///////////////////FUNCTIONS/////////////////////

void CalculateDist(Mat& img, Mat t_img)
{
    if(target.width != 0) {
		//A target was found!
		upperLeft.x = target.x;
		upperLeft.y = target.y;
		lowerRight.x = upperLeft.x + target.width;
		lowerRight.y = upperLeft.y + target.height;

		//ID the center in yellow
		Point center(target.x + target.width/2, target.y + target.height/2);

		#ifdef VISUALIZE
		line(img, center, center, YELLOW, 3);
		line(img ,Point(320,240),Point(320,240),YELLOW,3);
		//draw a box around the target in green
		rectangle(img, upperLeft, lowerRight, GREEN, 2);
		#endif

		cout << "Width = " << target.width << ", Height = "
			<< target.height << ", Center = (" << center.x
			<< ", " << center.y << ")" << endl;

		//The center point of the screen is (320,240)

		//Subtract the Center X point from 320
		Point offset(0,0);
		offset.x = 320 - center.x;

		//Subtract the Center Y point from 240
		offset.y = 240 - center.y;

		double WidthHeightRatio;
		double twidth = target.width;
		double theight = target.height;
		WidthHeightRatio = twidth / theight;
		cout << WidthHeightRatio << endl;
		double distance;
		double angle;

		if((WidthHeightRatio >= 2.0) & (WidthHeightRatio <= 3.0)){
			cout << "Low Target!" << endl;
			distance = 497.8226371654 * exp(-0.0040780276 * target.width);
			distance = distance + 12;
			cout << "Distance (IN): " << distance << endl;
			double PivotToCenterGoalHeight = 0;
			PivotToCenterGoalHeight = 110.125 - HeightGroundToPivot;
			angle = atan( PivotToCenterGoalHeight / distance );
			angle = angle * 180 / pi;
			cout << "Angle To Shoot At: " << angle << endl;
		}

		if((WidthHeightRatio >= 3.0) & (WidthHeightRatio <= 4.0)){
			cout << "High Target!" << endl;
			distance = 497.8226371654 * exp(-0.0040780276 * target.width);
			cout << "Distance (IN): " << distance << endl;
			double PivotToCenterGoalHeight = 0;
			PivotToCenterGoalHeight = 99.125 - HeightGroundToPivot;
			angle = atan( PivotToCenterGoalHeight / distance );
			angle = angle * 180 / pi;
			cout << "Angle To Shoot At: " << angle << endl;

		}

		double offsety;
		offsety = 240 - center.y;

		double offsetx;
		offsetx = 320 - center.x;

		double rotateangle;
		rotateangle = atan(offsety / distance);
		rotateangle = rotateangle * 180 / pi;
		//cout << "Rotate Angle: " << rotateangle << endl;

		//y=-0.0166x + 6.9019
		double ppi_calc;
		ppi_calc = (-1*0.0166) * offsetx + 6.9019;
		ppi_calc = offsetx / ppi_calc;

		double offsetangle;
		offsetangle = atan(ppi_calc/distance);

		offsetangle = offsetangle * 180 / pi;
		cout << "Rotation Angle: " << offsetangle << endl;

		pthread_mutex_lock(&angleTCPmutex);
		TCPangleVal = offsetangle;
		pthread_mutex_unlock(&angleTCPmutex);
		pthread_mutex_lock(&distanceTCPmutex);
		TCPdistanceVal = distance;
		pthread_mutex_unlock(&distanceTCPmutex);

		cout << endl;
}
}

void findTarget(Mat original, Mat thresholded)
{
    //Find rectangles
        findContours(thresholded, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);


	//Set target width to a know value.
	target.width = 0;

	int highestY = original.size().height;
	//process the contours we found, widdle down to a probable target.
	for(unsigned int i = 0; i < contours.size(); i++)
	{
		//Make a rectangle that encompasses the target
		boundbox = boundingRect(contours[i]);

		//make sure its not a tiny rectangle (noise)
		if((boundbox.width >= MIN_WIDTH) && (boundbox.y < highestY ))
		{
			target = boundbox;
		}
	}
}

Mat ThresholdImage(Mat original)
{
    //Local Temp Image
    Mat thresholded;

    //Threshold image to remove non-blue objects
	inRange(original, Scalar(minB, minG, minR), Scalar(maxB, maxG, maxR), thresholded);

	//smooth edges
	//See if we can get rid of the blur operation, slows things down quite
	// a bit and only seems to matter for targets whigh are not fully closed
	blur(thresholded, thresholded, Size(3, 3));
	morphologyEx(thresholded, thresholded, MORPH_CLOSE, Mat::ones(3, 3, CV_8U));

    return thresholded;

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
    	system("wget -q http://192.168.1.90/jpg/image.jpg -O capturedImage.jpg");

    	//load downloaded image
    	img = imread("capturedImage.jpg");

	}
	else if(params.From_File){
		//load image from file
		img = imread(params.IMAGE_FILE);
	}

return img;
}

void TCP_thread(void *obj)
{

//	char TCPbuffer[1024];
//	int TCPbytes = 0;
//	double angleVal = 0;
//	double distanceVal = 0;
//
//
//
//	while(true){
//
//		bzero(TCPbuffer, 1024);
//		TCPbytes = recv(sockfd, TCPbuffer, 1023, 0);
//
//		pthread_mutex_lock(&angleTCPmutex);
//		angleVal = TCPangleVal;
//		pthread_mutex_unlock(&angleTCPmutex);
//
//		pthread_mutex_lock(&distanceTCPmutex);
//		distanceVal = TCPdistanceVal;
//		pthread_mutex_unlock(&distanceTCPmutex);
//
//		std::ostringstream sstream;
//		sstream << angleVal << ", " << distanceVal << "\n";
//		std::string data_string = sstream.str();
//			write(sockfd, data_string.data(), data_string.length());
//
//	}

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

void TCP_Server()
{
	/*

	//	const char *message1 = "Thread 1";
	//	pthread_t tcp_thread;
	//	int itcp_thread;
	//	pthread_create( &tcp_thread, NULL, TCP_thread, (void*) message1);

	//	int sockfd, portno, n;
		int portno, n;
	    struct sockaddr_in serv_addr;
	    struct hostent *server;

	    char buffer[256];
	    if (argc < 3) {
	       fprintf(stderr,"usage %s hostname port\n", argv[0]);
	       exit(0);
	    }
	    portno = atoi(argv[3]);
	    sockfd = socket(AF_INET, SOCK_STREAM, 0);
	    if (sockfd < 0)
	        error("ERROR opening socket");
	    server = gethostbyname(argv[2]);
	    if (server == NULL) {
	        fprintf(stderr,"ERROR, no such host\n");
	        exit(0);
	    }
	    bzero((char *) &serv_addr, sizeof(serv_addr));
	    serv_addr.sin_family = AF_INET;
	    bcopy((char *)server->h_addr,
	         (char *)&serv_addr.sin_addr.s_addr,
	         server->h_length);
	    serv_addr.sin_port = htons(portno);
	    if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0)
	        error("ERROR connecting");
	*/
}
