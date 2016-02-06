
#include "mjpeg_server.h"

#define SSTR( x ) static_cast< std::ostringstream & >( \
        ( std::ostringstream() << std::dec << x ) ).str()

// I got tired of the warning so I just set a compiler argument to stop showing it.
// Im using a depricated way of convert string to const char * or something like that.
#pragma GCC diagnostic ignored "-Wwrite-strings"

int sockfd, newsockfd, portno, n;
socklen_t clilen;
char buffer[256];


struct sockaddr_in serv_addr, cli_addr;

cv::vector<uchar> buf;

std::string initResponse;
std::string contentType;
std::string boundary;

bool imageReady;




mjpeg_server::mjpeg_server()
{
    sockfd = -1;
    newsockfd = -1;
    portno = 0;
    n=0;
    imageReady = false;


	initResponse =
		"HTTP/1.0 200 OK\n"
		"Cache-Control: no-store, no-cache, must-revalidate, pre-check=0, post-check=0, max-age=0\n"
		"Content-Type: multipart/x-mixed-replace;boundary=boundarydonotcross\n\n";


	contentType = "Content-Type: image/jpeg\n\n";

	boundary = "--boundarydonotcross";

}


int mjpeg_server::initMJPEGServer(int port) {
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

void *mjpeg_server::host(void *args) {
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

	send(newsockfd, initResponse.c_str(), strlen(initResponse.c_str()), 0);

	imageReady = true;

	while (1) {
//		if (imageReady) {
//			send(newsockfd, contentType.c_str(), strlen(contentType.c_str()), 0);
//
//			send(newsockfd, (&memoryJPEG), sizeof(memoryJPEG), 0);
//
//			send(newsockfd, boundary.c_str(), strlen(boundary.c_str()), 0);
//		}
//		usleep(10000);
	}

}

int mjpeg_server::send_image(int socket){

	FILE *picture;
   	int size, read_size, stat, packet_index;
	char send_buffer[10240], read_buffer[256];
	packet_index = 1;

	picture = fopen("output.jpg", "r");
	printf("Getting Picture Size\n");

	if(picture == NULL) {
		printf("Error Opening Image File"); }

	fseek(picture, 0, SEEK_END);
	size = ftell(picture);
	fseek(picture, 0, SEEK_SET);
	printf("Total Picture size: %i\n",size);

	//Send Picture Size
	printf("Sending Picture Size\n");
	//write(socket, (void *)&size, sizeof(int));

	//Send Picture as Byte Array
	printf("Sending Picture as Byte Array\n");

	//do { //Read while we get errors that are due to signals.
	//	stat=read(socket, &read_buffer , 255);
	//	printf("Bytes read: %i\n",stat);
	//} while (stat < 0);

	//printf("Received data in socket\n");
	//printf("Socket data: %c\n", read_buffer);

	while(!feof(picture)) {
		read_size = fread(send_buffer, 1, sizeof(send_buffer)-1, picture);

		//Send data through our socket
		do{
		stat = write(socket, send_buffer, read_size);
		}while (stat < 0);

		printf("Packet Number: %i\n",packet_index);
		printf("Packet Size Sent: %i\n",read_size);
		printf(" \n");
		printf(" \n");

		packet_index++;

		//Zero out our send buffer
		bzero(send_buffer, sizeof(send_buffer));
	}
}

void mjpeg_server::setImageToHost(cv::Mat image)
{
	if (imageReady) {
		cv::imencode(".jpg", image, buf, std::vector<int>() );

		std::string pathname("output.jpg");

		std::ofstream textout(pathname.c_str(), std::ios::out | std::ios::binary);
		textout.write((const char*)&buf[0], buf.size());

		textout.close();

		send(newsockfd, contentType.c_str(), strlen(contentType.c_str()), 0);
		send_image(newsockfd);
		send(newsockfd, boundary.c_str(), strlen(boundary.c_str()), 0);
	}
}
void mjpeg_server::error(char *msg)
{
	perror(msg);
	exit(1);
}
