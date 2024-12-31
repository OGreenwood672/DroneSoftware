#ifndef ENVIROMENT_BLOCK_H
#define ENVIROMENT_BLOCK_H

#include "../globals.h"
#include "octree/Octree.h"

#include <memory>
#include <unordered_set>
#include <array>
#include <iostream>
#include <cmath>

class EnviromentBlock {

private:

    int x, y, z;

    std::unique_ptr<Octree> octree;
    int point_count;

    int air_count;


public:

    EnviromentBlock(int x, int y, int z);
    ~EnviromentBlock();

    inline bool operator==(const EnviromentBlock& other) const {
        return x == other.x && y == other.y && z == other.z;
    }


    void set_position(int x, int y, int z);
    std::array<int, 3> get_position() const;
    float get_distance(EnviromentBlock* other) const;

    std::unordered_set<Point> get_points() const;

    bool is_air() const;
    bool is_unscanned() const;
    bool is_block() const;

    std::unordered_set<Point> get_shared_points(EnviromentBlock* block) const;

    void increment_air_count();
    void add_point(Point p);

};


#endif // ENVIROMENT_BLOCK_H