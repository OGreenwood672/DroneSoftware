#include "drone_communication/Communicator.h"
#include "enviroment/enviroment.h"
#include "3d_model/alpha_shapes.h"

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

    Enviroment env;

    // Update points with at BATCH_SIZE at a time
    for (int i = 0; i < points.size(); i += BATCH_SIZE) {
        Point batch[BATCH_SIZE];
        bool skip = false;
        for (int j = 0; j < BATCH_SIZE; j++) {
            if (i + j < points.size()) {
                batch[j] = points[i + j];
            } else {
                skip = true;
            }
        }

        // Update the enviroment with the batch
        if (!skip) {
            env.update_enviroment(Point(0, 0, 0), batch);
        }
    }

    env.export_point_cloud("point_cloud.ply");

    // AlphaShapes alpha_shapes(env, 50);

    // alpha_shapes.computeAlphaShapes();
    // alpha_shapes.calculateMergedAlphaShape();

    // alpha_shapes.export_to_obj("output.obj");


}

int main() {


    test_alpha_shape();

    return 0;

}
