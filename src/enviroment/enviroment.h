#ifndef ENVIROMENT_H
#define ENVIROMENT_H

#include "point/point.h"
#include "octree/Octree.h"
#include "pathfinding/bresenham_3d.h"

#include <functional>
#include <memory>

// CENTIMETERS
#define DRONE_WIDTH 100
#define DRONE_DEPTH 100
#define DRONE_HEIGHT 100

#define WORLD_WIDTH 2000
#define WORLD_DEPTH 2000
#define WORLD_HEIGHT 2000

#define BATCH_SIZE 15


class Enviroment {

private:

    std::unique_ptr<Octree> world[WORLD_WIDTH / DRONE_WIDTH][WORLD_DEPTH / DRONE_DEPTH][WORLD_HEIGHT / DRONE_HEIGHT];
    int width, depth, height;

    bool is_air(int x, int y, int z);

public:
    Enviroment();

    Point update_enviroment(Point origin, Point points[]);

    std::vector<std::array<int, 3>> get_air_neighbours(int x, int y, int z);
    std::vector<std::array<int, 3>> get_neighbours(int x, int y, int z);

    std::unordered_set<Point> get_points() const;
    std::unordered_set<Point> get_shared_points(int x1, int y1, int z1, int x2, int y2, int z2);

    void export_point_cloud(const std::string& filename) const;

    void apply_to_world(std::function<void(int x, int y, int z, std::unique_ptr<Octree>&)> func);

    int get_width() const;
    int get_depth() const;
    int get_height() const;

};



#endif // ENVIROMENT_H