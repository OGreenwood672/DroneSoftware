#ifndef ALPHA_SHAPES_H
#define ALPHA_SHAPES_H

#include "../point/point.h"
#include "../enviroment.h"

#include <vector>
#include <unordered_set>

struct Tetrahedron {
    Point A;
    Point B;
    Point C;
    Point D;

    Tetrahedron(Point a, Point b, Point c, Point d) 
        : A(a), B(b), C(c), D(d) {}

    double getCircumsphereRadius() const;
};

class AlphaShapes {

private:

    Enviroment* world;
    double alpha;
    std::vector<Tetrahedron> tetrahedra;

    std::vector<std::vector<std::vector<std::vector<Tetrahedron>>>> voxelTetrahedra;
    
    void computeVoxelAlphaShape(int x, int y, int z, std::unique_ptr<Octree>& octree);

    bool isPointInCircumsphere(const Point p, const Tetrahedron& t) const;

    void mergeBoundaryTetrahedra(
        const std::unordered_set<Point>& sharedPoints,
        std::vector<Tetrahedron>& voxel1Tetrahedra,
        std::vector<Tetrahedron>& voxel2Tetrahedra
    );

public:
    AlphaShapes(Enviroment* world, double alpha);
    ~AlphaShapes();

    void computeAlphaShapes();

    std::unordered_set<Tetrahedron> getMergedAlphaShape() const;

};

#endif // ALPHA_SHAPES_H