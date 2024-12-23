#include "enviroment/octree/Octree.h"
#include "drone_communication/Communicator.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>


void test_network() {

        Communicator comm;

    // Array to store received data
    int receivedData[BATCHSIZE][3];

    // Receive 15 int[3] arrays from the ESP32
    comm.receive_data(receivedData);

    // Process the received data (e.g., return the last array with modified values)
    int response[3] = {receivedData[BATCHSIZE - 1][0] + 1, 
                       receivedData[BATCHSIZE - 1][1] + 1, 
                       receivedData[BATCHSIZE - 1][2] + 1};

    // Send the response back to the ESP32
    comm.send_data(response);

}

void test_alpha_shape() {

    std::ifstream file("./src/3d_model/test_models/bunny.txt"); // Open the file
    if (!file.is_open()) {
        std::cerr << "Error: Could not open the file!" << std::endl;
        return;
    }

    std::vector<Point> points; // Vector to store the points
    std::string line;

    // Read each line from the file
    while (std::getline(file, line)) {
        std::istringstream iss(line); // Use stringstream to parse the line
        float x, y, z;

        if (iss >> x >> y >> z) { // Extract x, y, z
            // Transform: Scale by 100 and shift by adding 100 to make all positive
            x = (x + 1) * 500;
            y = (y + 1) * 500;
            z = (z + 1) * 500;

            points.emplace_back(x, y, z); // Add the transformed point to the vector
        }
    }

    file.close(); // Close the file

    // Output the points to verify
    for (const auto& point : points) {
        std::cout << "Point(" << point.x << ", " << point.y << ", " << point.z << ")\n";
    }


}

int main() {


    test_alpha_shape();

    return 0;

}
