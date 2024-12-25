
#include "point.h"

#include <cmath>

// Point constructors
Point::Point(int a, int b, int c) : x(a), y(b), z(c) {}
Point::Point() : x(-1), y(-1), z(-1) {}


double Point::distance(const Point& other) const {
    return std::sqrt((x - other.x) * (x - other.x) + (y - other.y) * (y - other.y) + (z - other.z) * (z - other.z));
}