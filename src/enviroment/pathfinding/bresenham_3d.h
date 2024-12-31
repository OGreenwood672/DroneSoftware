#ifndef BRESENHAM_3D_H
#define BRESENHAM_3D_H

#include <array>
#include <vector>
#include <cmath>

inline std::vector<std::array<int, 3>> bresenham_3d(const std::array<int, 3> start, const std::array<int, 3> end) {
    int x1 = start[0], y1 = start[1], z1 = start[2];
    int x2 = end[0], y2 = end[1], z2 = end[2];

    std::vector<std::array<int, 3>> points;

    int dx = abs(x2 - x1);
    int dy = abs(y2 - y1);
    int dz = abs(z2 - z1);

    int sx = (x1 < x2) ? 1 : -1;
    int sy = (y1 < y2) ? 1 : -1;
    int sz = (z1 < z2) ? 1 : -1;

    int errX = (dx >= dy && dx >= dz) ? dx / 2 : 0;
    int errY = (dy >= dx && dy >= dz) ? dy / 2 : 0;
    int errZ = (dz >= dx && dz >= dy) ? dz / 2 : 0;

    while (true) {
        points.push_back({x1, y1, z1});

        if (x1 == x2 && y1 == y2 && z1 == z2) {
            break;
        }

        if (dx >= dy && dx >= dz) { // X-major
            errX -= dy;
            errZ -= dz;

            if (errX < 0) {
                y1 += sy;
                errX += dx;
            }
            if (errZ < 0) {
                z1 += sz;
                errZ += dx;
            }
            x1 += sx;
        } else if (dy >= dx && dy >= dz) { // Y-major
            errY -= dx;
            errZ -= dz;

            if (errY < 0) {
                x1 += sx;
                errY += dy;
            }
            if (errZ < 0) {
                z1 += sz;
                errZ += dy;
            }
            y1 += sy;
        } else { // Z-major
            errY -= dy;
            errX -= dx;

            if (errY < 0) {
                y1 += sy;
                errY += dz;
            }
            if (errX < 0) {
                x1 += sx;
                errX += dz;
            }
            z1 += sz;
        }
    }

    return points;
};

inline std::array<int, 3> get_furthest_point(const std::array<int, 3>& min_bound, 
                                             const std::array<int, 3>& upper_bound, 
                                             const std::array<int, 3>& origin, 
                                             const std::array<float, 3>& vector) {

    double tMin = std::numeric_limits<double>::infinity();

    // Check for intersection along each axis
    for (int i = 0; i < 3; ++i) {
        double tFar;
        double o = (i == 0) ? origin[0] : (i == 1) ? origin[1] : origin[2];
        double d = (i == 0) ? vector[0] : (i == 1) ? vector[1] : vector[2];
        double minB = (i == 0) ? min_bound[0] : (i == 1) ? min_bound[1] : min_bound[2];
        double maxB = (i == 0) ? upper_bound[0] : (i == 1) ? upper_bound[1] : upper_bound[2];

        if (fabs(d) < 1e-8) {
            // Parallel to planes; skip axis if no motion in this direction
            continue;
        }

        // Calculate t for far plane (only exiting planes matter)
        if (d > 0) {
            tFar = (maxB - o) / d;
        } else {
            tFar = (minB - o) / d;
        }

        // Update the minimum t
        tMin = std::min(tMin, tFar);
    }

    std::array<int, 3> intersection_point;

    // Calculate the intersection point using tMin
    intersection_point[0] = origin[0] + tMin * vector[0];
    intersection_point[1] = origin[1] + tMin * vector[1];
    intersection_point[2] = origin[2] + tMin * vector[2];
    return intersection_point;

}


#endif
