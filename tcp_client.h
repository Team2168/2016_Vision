/*
 * tcp_client.h
 *
 *  Created on: Jan 31, 2014
 *      Author: kevin
 */

#ifndef TCP_CLIENT_H_
#define TCP_CLIENT_H_

/**
    TCP Client class
*/
class tcp_client
{
private:
    int sock;
    std::string address;
    int port;
    struct sockaddr_in server;

public:
    tcp_client();
    bool conn(std::string, int);
    bool send_data(std::string data);
    std::string receive(int);
};



#endif /* TCP_CLIENT_H_ */
