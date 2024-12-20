#ifndef ENVIROMENT_H
#define ENVIROMENT_H

#include "point/point.h"
#include "octree/Octree.h"

// CENTIMETERS
#define DRONE_WIDTH 100
#define DRONE_DEPTH 100
#define DRONE_HEIGHT 100

#define WORLD_WIDTH 1000
#define WORLD_DEPTH 1000
#define WORLD_HEIGHT 1000

#define BATCH_SIZE 15


class Enviroment {

private:

    std::vector<std::vector<std::vector<std::unique_ptr<Octree>>>> world;
    int width, depth, height;

    bool is_air(int x, int y, int z);

public:
    Enviroment();

    Point update_enviroment(Point points[]);

    std::vector<Point> get_air_neighbours(Point pnt);

};



#endif // ENVIROMENT_H