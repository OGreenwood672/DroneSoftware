#ifndef ENVIROMENT_H
#define ENVIROMENT_H

#include "point/point.h"
#include "octree/Octree.h"

#include <functional>

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

    std::vector<int[3]> get_air_neighbours(int x, int y, int z);
    std::vector<int[3]> get_neighbours(int x, int y, int z);

    std::unordered_set<Point> get_points();
    std::unordered_set<Point> get_shared_points(int x1, int y1, int z1, int x2, int y2, int z2);

    void apply_to_world(std::function<void(int x, int y, int z, std::unique_ptr<Octree>&)> func);

    int get_width() const;
    int get_depth() const;
    int get_height() const;

};



#endif // ENVIROMENT_H