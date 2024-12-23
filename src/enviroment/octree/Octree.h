#ifndef OCTREE_H
#define OCTREE_H

#include <iostream>
#include <vector>
#include <unordered_set>

#include "../point/point.h"

// Constants for child node positions
#define TopLeftFront 0
#define TopRightFront 1
#define BottomRightFront 2
#define BottomLeftFront 3
#define TopLeftBottom 4
#define TopRightBottom 5
#define BottomRightBack 6
#define BottomLeftBack 7



// Class to represent an Octree
class Octree {
private:
    // If point == nullptr, node is internal.
    // If point == (-1, -1, -1), node is empty.
    Point* point;

    // Boundaries of the cube
    Point *topLeftFront, *bottomRightBack;

    // Children nodes (8 subdivisions)
    std::vector<Octree*> children;

    // Helper to determine child position for a given point
    int determineChildPosition(int x, int y, int z, int midx, int midy, int midz) const;

public:
    Octree();
    Octree(int x, int y, int z);                     // Point-only constructor
    Octree(int x1, int y1, int z1, int x2, int y2, int z2); // Boundary constructor
    ~Octree();                                       // Destructor to clean up dynamic memory

    // Insert a point into the Octree
    void insert(int x, int y, int z);

    // Find if a point exists in the Octree
    bool find(int x, int y, int z) const;

    std::unordered_set<Point> get_points() const;

    bool is_empty() const;
};

#endif // OCTREE_H
