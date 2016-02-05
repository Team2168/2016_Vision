#include <unistd.h>
#include <stdio.h>
#include <sstream>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <opencv2/opencv.hpp>
#include <pthread>

#define SSTR( x ) static_cast< std::ostringstream & >( \
        ( std::ostringstream() << std::dec << x ) ).str()

// I got tired of the warning so I just set a compiler argument to stop showing it.
// Im using a depricated way of convert string to const char * or something like that.
#pragma GCC diagnostic ignored "-Wwrite-strings"

int sockfd, newsockfd, portno, n;
socklen_t clilen;
char buffer[256];

struct sockaddr_in serv_addr, cli_addr;

cv::vector<uchar> memoryJPEG;
cv::Mat decodedMatrix;

std::string initResponse =
	"HTTP/1.0 200 OK\n"
	"Cache-Control: no-store, no-cache, must-revalidate, pre-check=0, post-check=0, max-age=0\n"
	"Content-Type: multipart/x-mixed-replace;boundary=--boundarydonotcross\n\n";


std::string sendImageInitHeader =
	"--boundarydonotcross\n"
	"Content-Type: image/jpeg\n";

std::string contentLength =
	"Content-Length: ";

bool imageReady = false;

void error(char *msg) {
	perror(msg);
	exit(1);
}

int initMJPEGServer(int port) {
	sockfd = socket(AF_INET, SOCK_STREAM, 0);

	bzero((char *) &serv_addr, sizeof(serv_addr));

	portno = port;

	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(portno);

	serv_addr.sin_addr.s_addr = INADDR_ANY;

	if (bind(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
		error("Cannot bind socket");
		return 0;
	}
	return 1;
}

void *host(void *args) {
	listen(sockfd,5);
     	clilen = sizeof(cli_addr);
     	newsockfd = accept(sockfd,
                 	(struct sockaddr *) &cli_addr,
                 	&clilen);
     	if (newsockfd < 0)
          	error("ERROR on accept");
    	bzero(buffer,256);
     	n = recv(newsockfd,buffer,sizeof(buffer), 0);

	std::cout << buffer << std::endl;
	std::cout << initResponse << std::endl;

	n = send(newsockfd, initResponse.c_str(), strlen(initResponse.c_str()), 0);

	while (1) {
		if (imageReady) {
			send(newsockfd, sendImageInitHeader.c_str(), strlen(sendImageInitHeader.c_str()), 0);
			contentLength.append(SSTR(memoryJPEG.size()));
			contentLength.append("\n");
			send(newsockfd, contentLength.c_str(), strlen(contentLength.c_str()), 0);
			send(newsockfd, "\n", 1, 0);
			send(newsockfd, (void *)(&memoryJPEG), sizeof(memoryJPEG), 0);
			send(newsockfd, "\n\n", 2, 0);
			contentLength = "Content-Length: ";
		}
		usleep(1000);
	}

}

void setImageToHost(cv::Mat image){
	imageReady = false;
	cv::imencode(".jpg", image, memoryJPEG, std::vector<int>() );
	imageReady = true;
}
