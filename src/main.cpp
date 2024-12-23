#include "enviroment/octree/Octree.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

int main() {


    std::ifstream file("./src/3d_model/test_models/bunny.txt"); // Open the file
    if (!file.is_open()) {
        std::cerr << "Error: Could not open the file!" << std::endl;
        return 1;
    }

    std::vector<Point> points; // Vector to store the points
    std::string line;

    // Read each line from the file
    while (std::getline(file, line)) {
        std::istringstream iss(line); // Use stringstream to parse the line
        double x, y, z;

        if (iss >> x >> y >> z) { // Extract x, y, z
            points.emplace_back(x, y, z); // Add the point to the vector
        }
    }

    file.close(); // Close the file

    // Output the points to verify
    for (const auto& point : points) {
        std::cout << "Point(" << point.x << ", " << point.y << ", " << point.z << ")\n";
    }

    return 0;
}
