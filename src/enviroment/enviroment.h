#ifndef ENVIROMENT_H
#define ENVIROMENT_H

#include "point/point.h"
#include "EnviromentBlock/octree/Octree.h"
#include "EnviromentBlock/enviroment_block.h"
#include "pathfinding/bresenham_3d.h"
#include "pathfinding/generate_fibonacci_sphere_vectors.h"
#include "globals.h"

#include <functional>
#include <memory>

class Enviroment {

private:

    EnviromentBlock* world[WORLD_WIDTH / DRONE_WIDTH][WORLD_DEPTH / DRONE_DEPTH][WORLD_HEIGHT / DRONE_HEIGHT];
    int width, depth, height;


public:
    Enviroment();
    ~Enviroment();

    Point update_enviroment(Point origin, Point points[]);

    std::vector<EnviromentBlock*> get_air_neighbours(int x, int y, int z);
    std::vector<EnviromentBlock*> get_neighbours(int x, int y, int z);

    std::unordered_set<Point> get_points() const;

    void export_point_cloud(const std::string& filename) const;

    void apply_to_world(std::function<void(int x, int y, int z, EnviromentBlock* block)> func);

    float calculate_scan_score(std::array<int, 3> origin, int directions) const;

    EnviromentBlock* get_block(int x, int y, int z) const;

    int get_width() const;
    int get_depth() const;
    int get_height() const;

};



#endif // ENVIROMENT_H