
#include <stdio.h>
#include <curl/curl.h>
#include <sstream>
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

using namespace cv;

//curl writefunction to be passed as a parameter
size_t write_data(char *ptr, size_t size, size_t nmemb, void *userdata) {
	std::ostringstream *stream = (std::ostringstream*) userdata;
	size_t count = size * nmemb;
	stream->write(ptr, count);
	return count;
}

//function to retrieve the image as Cv::Mat data type
cv::Mat curlImg() {
	CURL *curl;
	CURLcode res;
	std::ostringstream stream;
	curl = curl_easy_init();
	curl_easy_setopt(curl, CURLOPT_URL, "http://10.21.68.90/jpg/image.jpg");
	//the JPEG Frame url
	curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_data);
	// pass the writefunction
	curl_easy_setopt(curl, CURLOPT_WRITEDATA, &stream);
	// pass the stream ptr when the writefunction is called
	res = curl_easy_perform(curl); // start curl
	std::string output = stream.str(); // convert the stream into a string
	curl_easy_cleanup(curl); // cleanup
	std::vector<char> data = std::vector<char>(output.begin(), output.end()); //convert string into a vector
	cv::Mat data_mat = cv::Mat(data); // create the cv::Mat datatype from the vector
	cv::Mat image = cv::imdecode(data_mat, 1); //read an image from memory buffer
	return image;
}

//int main(void) {
////cv::namedWindow( "Image output", WINDOW_AUTOSIZE );
//	struct timespec start, end;
//
//	while (1) {
//		clock_gettime(CLOCK_REALTIME, &start);
//
//		cv::Mat image = curlImg(); // get the image frame
////cv::imshow("Image output",image); //display image frame
//
//		char c = waitKey(20); // sleep for 33ms or till a key is pressed (put more then ur camera framerate mine is 30ms)
//		if (c == 27)
//			break; // break if ESC is pressed
//
//		clock_gettime(CLOCK_REALTIME, &end);
//		double difference = (end.tv_sec - start.tv_sec)
//				+ (double) (end.tv_nsec - start.tv_nsec) / 1000000000.0f;
//		std::cout << "It took " << difference << " seconds to process "
//				<< std::endl;
//	}
//	cv::destroyWindow("Image output");
//}
