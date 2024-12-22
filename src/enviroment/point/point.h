#ifndef POINT_H
#define POINT_H

#include <iostream>

// Struct to represent a 3D point
struct Point {
    int x, y, z;

    Point(int a, int b, int c);
    Point();

    bool operator==(const Point& other) const {
        return x == other.x && y == other.y && z == other.z;
    }

};

template <>
struct std::hash<Point> {
    size_t operator()(const Point& p) const {
        return std::hash<int>{}(p.x) ^ (std::hash<int>{}(p.y) << 1) ^ (std::hash<int>{}(p.z) << 2);
    }
};

#endif // POINT_H