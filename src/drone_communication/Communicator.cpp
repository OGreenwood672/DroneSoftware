#include "Communicator.h"

#include <iostream>

Communicator::Communicator() {

    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock == -1) {
        std::cerr << "Failed to create socket.\n";
        return;
    }

    std::cout << "Socket created.\n";

    server.sin_addr.s_addr = inet_addr(server_ip);
    server.sin_family = AF_INET;
    server.sin_port = htons(port);

    std::cout << "Connecting to the ESP32 server...\n";

    // Connect to the ESP32 server
    if (connect(sock, (struct sockaddr*)&server, sizeof(server)) < 0) {
        std::cerr << "Connection failed.\n";
        close(sock);
        return;
    }

    std::cout << "Connected to the ESP32 server.\n";

}

Communicator::~Communicator() {
    close(sock);
}

void Communicator::receive_data(int receivedData[BATCHSIZE][3]) {

    std::cout << "Receiving data...\n";

    for (int i = 0; i < 15; i++) {
        if (recv(sock, &receivedData[i], sizeof(receivedData[i]), 0) <= 0) {
            std::cerr << "Failed to receive data.\n";
            close(sock);
            return;
        }
        std::cout << "Received: [" << receivedData[i][0] << ", " << receivedData[i][1] << ", " << receivedData[i][2] << "]\n";
    }

    std::cout << "Recieved data.\n";

}

void Communicator::send_data(int data[3]) {

    std::cout << "Data sending...";
    if (send(sock, data, sizeof(int) * 3, 0) < 0) {
        std::cerr << "Failed to send response.\n";
        close(sock);
        return;
    }
    std::cout << "Data sent...";

}