#include <stdio.h>
#include <stdlib.h>
#include <curl/curl.h>
#include <curl/easy.h>
#include <opencv2/opencv.hpp>
//#include <opencv2/highgui/highgui_c.h>
//#include "/usr/include/opencv2/highgui.h"
//#include "/usr/include/opencv2/imgproc/imgproc.hpp"
#include <string>
#include "math.h"
#include <syslog.h>
#include <sys/stat.h>
#include <cstdio>
#include <queue>

#define FROM_FILE

using namespace cv;
using namespace std;

vector<char> image_data;
CURL *curl_handle;
static size_t WriteMemoryCallback(void *contents, size_t size, size_t nmemb,
		void *userp) {
	size_t realsize = size * nmemb;
	vector<char> *mem = (vector<char> *) userp;
	size_t before_size = mem->size();

	mem->resize(mem->size() + realsize);
	memcpy(&((*mem)[before_size]), contents, realsize);

	return realsize;
}

int main(int argc, char** argv) {
	int stop = 0;

	//Some common colors to draw with
	const Scalar red = Scalar(0, 0, 255), green = Scalar(0, 255, 0), blue =
			Scalar(255, 0, 0), orange = Scalar(0, 128, 255), yellow = Scalar(0,
			255, 255), pink = Scalar(255, 0, 255), white = Scalar(255, 255,
			255);

	Mat img, thresholded, output;
	vector<vector<Point> > contours;
	CvSeq* seq;
	CvRect boundbox;
	vector<CvRect> boxes;
	Point upperLeft, lowerRight;
	upperLeft.x = upperLeft.y = lowerRight.x = lowerRight.y = 0;

	//Thresholding parameters
	int minR = 0, maxR = 30, minG = 0, maxG = 30, minB = 60, maxB = 150;

	//Minimum width of goal in px.
	//Note: This will limit how far we can shoot from.
#define MIN_WIDTH 90

	//Target width/height ratios outside tape border
#define HIGH_WH_RATIO 3.100    //W = 62", H = 20"
#define MID_WH_RATIO  2.138    //W = 62", H = 29"
#define LOW_WH_RATIO  1.156    //W = 37", H = 32"
#define TOL           0.150    //tolerance percentage
	float ratioWH = 0.0; //var for calculationg ratios during runtime
	const float highGoalRatioUpperLimit = HIGH_WH_RATIO * (1.0 + TOL),
			highGoalRatioLowerLimit = HIGH_WH_RATIO * (1.0 - TOL),
			midGoalRatioUpperLimit = MID_WH_RATIO * (1.0 + TOL),
			midGoalRatioLowerLimit = MID_WH_RATIO * (1.0 - TOL),
			lowGoalRatioUpperLimit = LOW_WH_RATIO * (1.0 + TOL),
			lowGoalRatioLowerLimit = LOW_WH_RATIO * (1.0 - TOL);

	// set up image stream from camera
	curl_global_init(CURL_GLOBAL_ALL);
	curl_handle = curl_easy_init();
	curl_easy_setopt(curl_handle, CURLOPT_URL,
			"http://192.168.7.90/jpg/image.jpg");
	curl_easy_setopt(curl_handle, CURLOPT_NOPROGRESS, 1L);
	/* send all data to this function  */
	curl_easy_setopt(curl_handle, CURLOPT_WRITEFUNCTION, WriteMemoryCallback);

	// Create Window
	NamedWindow("IMAGE", WINDOW_AUTOSIZE);

	setbuf(stdout, NULL);

	while (!stop) {
#ifndef FROM_FILE
		image_data.clear();
		/* Put data into the image vector of char */
		curl_easy_setopt(curl_handle, CURLOPT_WRITEDATA , (void *)&image_data);

		/* get the image */
		curl_easy_perform(curl_handle);

		img = imdecode(Mat(image_data), 1);
#else
		if (argc < 2) {
			cout << "No file specified, exiting." << endl;
			exit(1);
		}
		img = imread(argv[1]);
#endif
		//Threshold image to remove non-blue objects
		inRange(img, Scalar(minB, minG, minR), Scalar(maxB, maxG, maxR),
				thresholded);

		//smooth edges
		//See if we can get rid of the blur operation, slows things down quite
		// a bit and only seems to matter for targets whigh are not fully closed
		blur(thresholded, thresholded, Size(3, 3));
		morphologyEx(thresholded, thresholded, MORPH_CLOSE,
				Mat::ones(3, 3, CV_8U));

		//Find rectangles
		findContours(thresholded, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
		drawContours(img, contours, -1, pink);

		//process the contours we found, widdle down to a probable target.
		for (int i = 0; i < contours.size(); i++) {
			//Make a rectangle that encompasses the target
			boundbox = boundingRect(contours[i]);

			//make sure its not a tiny rectangle (noise)
			if (boundbox.width >= MIN_WIDTH) {
				//find it's width/height ratio
				boxes.push_back(boundbox);
			}
		}

		//Display the rectangles we found over the original image
		while (!boxes.empty()) {
			upperLeft.x = boxes.back().x;
			upperLeft.y = boxes.back().y;
			lowerRight.x = upperLeft.x + boxes.back().width;
			lowerRight.y = upperLeft.y + boxes.back().height;

			ratioWH = (float) boxes.back().width / boxes.back().height;

			cout << "W/H = " << ratioWH << endl;

			//See if the rectangle we found is close to the known ratios
			//  of the different goals.
			if ((ratioWH <= highGoalRatioUpperLimit)
					&& (ratioWH >= highGoalRatioLowerLimit)) {
				//draw high goal in red
				rectangle(img, upperLeft, lowerRight, red, 2);
			} else if ((ratioWH <= midGoalRatioUpperLimit)
					&& (ratioWH >= midGoalRatioLowerLimit)) {
				//draw mid goal in green
				rectangle(img, upperLeft, lowerRight, green, 2);
			} else if ((ratioWH <= lowGoalRatioUpperLimit)
					&& (ratioWH >= lowGoalRatioLowerLimit)) {
				//draw low goal in yellow
				rectangle(img, upperLeft, lowerRight, yellow, 2);
			}
			//otherwise, the rectangle isn't a target

			boxes.pop_back();
		}
		cout << endl;

		imshow("IMAGE", img);

		//halt execution when esc key is pressed
		if (waitKey(30) >= 0)
			stop = 1;
	}

	curl_easy_cleanup(curl_handle);

	return 0;
}
