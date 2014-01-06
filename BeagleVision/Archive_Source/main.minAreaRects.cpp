#include <stdio.h>
#include <stdlib.h>
#include <curl/curl.h>
#include <curl/easy.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <string>
#include <cstdio>
#include "math.h"
#include <syslog.h>
#include <sys/stat.h>
#include <queue>

#define FROM_FILE

using namespace cv;
using namespace std;

vector<char> image_data;
CURL *curl_handle;
static size_t WriteMemoryCallback(void *contents, size_t size, size_t nmemb, void *userp){
    size_t realsize = size * nmemb ;
    vector<char> *mem = (vector<char> *)userp;
    size_t before_size = mem->size();

    mem->resize(mem->size() + realsize);
    memcpy(&((*mem)[before_size]), contents, realsize);

    return realsize;
}

int main(int argc, char** argv){
    //Some common colors to draw with
    Scalar red   = Scalar(0, 255, 0),
	   green = Scalar(29, 185, 5),
	   blue  = Scalar(245, 72, 19);

    Mat img, thresholded, output;
    vector<vector<Point> > contours;
    CvSeq* seq;

    Point upperLeft, lowerRight;
    upperLeft.x = upperLeft.y = lowerRight.x = lowerRight.y = 0;

    //Thresholding parameters
    int minR = 0,
	maxR = 30,
	minG = 0,
	maxG = 30,
	minB = 60,
	maxB = 150;


    //Target width/height ratios outside tape border
    long double highWHRatio = 3.100, //W = 62", H = 20"
	   	midWHRatio  = 2.138, //W = 62", H = 29"
	        lowWHRatio  = 1.156; //W = 37", H = 32"


    curl_global_init(CURL_GLOBAL_ALL);
    curl_handle = curl_easy_init();
    curl_easy_setopt(curl_handle, CURLOPT_URL,"http://192.168.7.90/jpg/image.jpg");
    curl_easy_setopt(curl_handle, CURLOPT_NOPROGRESS, 1L);
    /* send all data to this function  */
    curl_easy_setopt(curl_handle, CURLOPT_WRITEFUNCTION, WriteMemoryCallback);
	
    // Create Window
    NamedWindow("IMAGE", WINDOW_AUTOSIZE);


    RotatedRect box;
    Point2f vtx[4];
    vector<Point2f> vtxs;

    while(true){
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
	inRange(img, Scalar(minB, minG, minR), Scalar(maxB, maxG, maxR), thresholded);

	//smooth edges
	blur(thresholded, thresholded, Size(3, 3));
	morphologyEx(thresholded, thresholded, MORPH_CLOSE, Mat::ones(3, 3, CV_8U));

	//Find rectangles
        findContours(thresholded, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
       	//drawContours(img, contours, -1, red);

	//Make a rectangle that encompasses the target
	for(int i = 0; i < contours.size(); i++) {
		box = minAreaRect(Mat(contours[i]));
		box.points(vtx);
		vtxs.push_back(vtx);
		for(int j = 0; j < 4; j++)
			line(img, vtx[j], vtx[(j+1)%4], Scalar(0,0,255), 1, CV_AA);
	}

	imshow("IMAGE", img);

        //halt execution when esc key is pressed
        if(waitKey(30) >= 0) break;
    }

    curl_easy_cleanup(curl_handle);

    return 0;
}
