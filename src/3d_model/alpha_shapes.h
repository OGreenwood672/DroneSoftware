#ifndef ALPHA_SHAPES_H
#define ALPHA_SHAPES_H

#include "../enviroment/point/point.h"
#include "../enviroment/enviroment.h"

#include <vector>
#include <unordered_set>
#include <fstream>


struct Tetrahedron {
    Point A;
    Point B;
    Point C;
    Point D;

    Tetrahedron(Point a, Point b, Point c, Point d) 
        : A(a), B(b), C(c), D(d) {}

    double getCircumsphereRadius() const;

    bool operator==(const Tetrahedron& other) const {
        return A == other.A && B == other.B && C == other.C && D == other.D;
    }
};

template <>
struct std::hash<Tetrahedron> {
    size_t operator()(const Tetrahedron& t) const {
        // Combine the hash values of the vertices
        return std::hash<Point>()(t.A) ^ std::hash<Point>()(t.B) ^ std::hash<Point>()(t.C) ^ std::hash<Point>()(t.D);
    }
};

class AlphaShapes {

private:

    Enviroment& world;
    double alpha;
    std::unordered_set<Tetrahedron> tetrahedra;

    std::vector<Tetrahedron> voxelTetrahedra[WORLD_WIDTH / DRONE_WIDTH][WORLD_DEPTH / DRONE_DEPTH][WORLD_HEIGHT / DRONE_HEIGHT];
    
    void computeVoxelAlphaShape(int x, int y, int z, std::unordered_set<Point> points);

    bool isPointInCircumsphere(const Point p, const Tetrahedron& t) const;

    void mergeBoundaryTetrahedra(
        const std::unordered_set<Point>& sharedPoints,
        std::vector<Tetrahedron>& voxel1Tetrahedra,
        std::vector<Tetrahedron>& voxel2Tetrahedra
    );

public:
    AlphaShapes(Enviroment& world, double alpha);
    ~AlphaShapes();

    void computeAlphaShapes();

    void calculateMergedAlphaShape();

    void export_to_obj(const std::string& filename) const;

};

#endif // ALPHA_SHAPES_H