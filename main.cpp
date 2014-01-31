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


#include <fstream>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <netdb.h>

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

};

//function declarations
//TODO: add pre- and post- comments for each function
void parseCommandInputs(int argc, const char* argv[], ProgParams &params);
Mat GetOriginalImage(const ProgParams& params);
double diffclock(clock_t clock1,clock_t clock2);
Mat ThresholdImage(Mat img);
Target findTarget(Mat original, Mat thresholded);
void NullTargets(Target& target);
double CalculateDist(Target targets);

void TCP_thread(void *obj);
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
int sockfd;
pthread_mutex_t angleTCPmutex;
pthread_mutex_t distanceTCPmutex;
pthread_t thread;
pthread_t TCPthread;

double TCPangleVal;
double TCPdistanceVal;





int main(int argc, const char* argv[])
{

	//Read command line inputs to determine how the program will execute
	ProgParams params;
	parseCommandInputs(argc, argv, params);

	//Create Program Image Vairables
	Mat img, thresholded, output;

	//create windows
	namedWindow("Original", WINDOW_AUTOSIZE);
	namedWindow("Treshold", WINDOW_AUTOSIZE);

	//store targets
	Target targets;

	bool progRun = true;

	while(progRun)
	{
		clock_t start_time, end_time = 0.0;
		start_time = clock();


		img = GetOriginalImage(params);
		imshow("Original", img);

		thresholded = ThresholdImage(img);
		imshow("Treshold", thresholded);

		targets = findTarget(img, thresholded);
		cout<<"Vert: "<<targets.VertGoal<<endl;
		cout<<"Horiz: "<<targets.HorizGoal<<endl;
		cout<<"Hot Goal: "<<targets.HotGoal<<endl;


		cout<<"Dist:" <<CalculateDist(targets)<<endl<<endl;


		end_time = clock();
		cout << "Image proc. time: " << double(diffclock(end_time,start_time)) << "ms" << endl;

#ifdef VISUALIZE
		//halt execution when esc key is pressed
		if(waitKey(30) >= 0)
			progRun = 1;
#endif
	}

    tcp_client c;
    string host;

    cout<<"Enter hostname : ";
    cin>>host;

    //connect to host
    c.conn(host , 80);

    //send some data
    c.send_data("GET / HTTP/1.1\r\n\r\n");

    //receive and echo reply
    cout<<"----------------------------\n\n";
    cout<<c.receive(1024);
    cout<<"\n\n----------------------------\n\n";

    //done
    return 0;

	return 0;

}


///////////////////FUNCTIONS/////////////////////

double CalculateDist(Target targets)
{
	//vertical target is 32 inches fixed
	double targetHeight = 32.0;

	//get vertical pixels from targets
	int height = targets.VerticalTarget.height;

	//d = Tft*FOVpixel/(2*Tpixel*tanΘ)
	return Y_IMAGE_RES * targetHeight / (height * 12 * 2 * tan(VIEW_ANGLE*PI/(180*2)));
}


Target findTarget(Mat original, Mat thresholded)
{

	vector<Vec4i> hierarchy;
	vector<vector<Point> > contours;
	Target targets;


	/// Show in a window
	namedWindow( "Contours", WINDOW_AUTOSIZE );


	//Find rectangles
	findContours(thresholded, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

	cout<<"Contours: "<<contours.size()<<endl;
	cout<<"Hierarchy: "<<hierarchy.size()<<endl;


	//run through all contours and remove small contours
	unsigned int contourMin = 20;
	for (vector<vector<Point> >::iterator it = contours.begin(); it!=contours.end(); )
	{
		cout<<"Contour Size: "<<it->size()<<endl;
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
			//capture corners of copntour
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

			cout<<"Contour: "<<i<<endl;
			cout<<"\tX: "<<box.x<<endl;
			cout<<"\tY: "<<box.y<<endl;
			cout<<"\tHeight: "<<box.height<<endl;
			cout<<"\tWidth: "<<box.width<<endl;
			cout<<"\tangle: "<<minRect[i].angle<<endl;
			cout<<"\tRatio (W/H): "<<WHRatio<<endl;
			cout<<"\tRatio (H/W): "<<HWRatio<<endl;




			//ID the center in yellow
			Point center(box.x + box.width/2, box.y + box.height/2);
			line(drawing, center, center, YELLOW, 3);
			line(drawing ,Point(320,240),Point(320,240),YELLOW,3);

		}




		imshow( "Contours", drawing );//Make a rectangle that encompasses the target
	}
	else
		cout<<"No Contours"<<endl;

	return targets;

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

void sendTCPPacket(string data)
{
	
	// Client socket descriptor which is just integer number used to access a socket
        int sock_descriptor;
        struct sockaddr_in serv_addr;

        // Structure from netdb.h file used for determining host name from local host's ip address
        struct hostent *server;

        // Buffer to input data from console and write to server
        char buff[MAX_SIZE];

        // Create socket of domain - Internet (IP) address, type - Stream based (TCP) and protocol unspecified
        // since it is only useful when underlying stack allows more than one protocol and we are choosing one.
        // 0 means choose the default protocol.
        sock_descriptor = socket(AF_INET, SOCK_STREAM, 0);

        if(sock_descriptor < 0)
          printf("Failed creating socket\n");

        bzero((char *)&serv_addr, sizeof(serv_addr));


        server = gethostbyname("127.0.0.1");
        
        if(server == NULL)
        {       
            printf("Failed finding server name\n");
        }

        serv_addr.sin_family = AF_INET;
        memcpy((char *) &(serv_addr.sin_addr.s_addr), (char *)(server->h_addr), server->h_length);
        
        //TODO change 1234 to the port number
        serv_addr.sin_port = htons(1234);
 
        if (connect(sock_descriptor, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    	{
        	printf("Failed to connect to server\n");
	}
       	
        //Need to change stdin to whatever is needed to send to the data

        if( send(sock_descriptor , data.c_str() , strlen( data.c_str() ) , 0) < 0)
        {
            perror("Send failed : ");

        }
        cout<<"Data send\n";


        int count = write(sock_descriptor, buff, strlen(buff));
       
        if(count < 0)
        	printf("Failed writing rquested bytes to server\n");
        

        close(sock_descriptor); 
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
