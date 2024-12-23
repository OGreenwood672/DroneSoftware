#ifndef COMMUNICATOR_H
#define COMMUNICATOR_H

#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>


#define BATCHSIZE 15


class Communicator {

private:

    const char* server_ip = "192.168.4.1";
    const int port = 12345;

    int sock;
    struct sockaddr_in server;


public:

    Communicator();
    ~Communicator();

    void send_data(int data[3]);
    void receive_data(int receivedData[BATCHSIZE][3]);


};


#endif //COMMUNICATOR_H