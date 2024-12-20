#include "../point/point.h"
#include "../enviroment.h"

#include <vector>
#include <unordered_map>


float heuristic(Point a, Point b) {
    return abs(a.x - b.x) + abs(a.y - b.y) + abs(a.z - b.z);
}

void binary_insert(std::unordered_map<Point, float> scores, std::vector<Point>& v, Point p) {
    int low = 0;
    int high = v.size();
    while (low < high) {
        int mid = low + (high - low) / 2;
        if (scores[v[mid]] < scores[p]) {
            low = mid + 1;
        } else {
            high = mid;
        }
    }
    v.insert(v.begin() + low, p);
}

float get_with_default(std::unordered_map<Point, float>& map, Point p, float default_value) {
    if (map.count(p) == 0) {
        map[p] = default_value;
        return default_value;
    }
    return map[p];
}


std::vector<Point> a_star(Point start, Point end, Enviroment world) {
    std::vector<Point> path;

    // If the start and end points are the same, return the start point
    if (start.x == end.x && start.y == end.y && start.z == end.z) {
        path.push_back(start);
        return path;
    }

    // Initialize the open and closed lists
    std::vector<Point> open;
    std::vector<Point> closed;

    std::unordered_map<Point, Point*> came_from;
    std::unordered_map<Point, float> g;
    std::unordered_map<Point, float> f;

    came_from[start] = nullptr;
    g[start] = 0;
    f[start] = g[start] + heuristic(start, end);

    // Add the start point to the open list
    open.push_back(start);

    // While the open list is not empty
    while (open.size() > 0) {

        // Get the current node
        Point current = open[0];

        // Pop the current node from the open list
        open.erase(open.begin());

        // Add the current node to the closed list
        closed.push_back(current);

        // If the current node is the end node, return the path
        if (current == end) {
            Point current_ = current;
            while (came_from[current_] != nullptr) {
                path.push_back(current_);
                current_ = *came_from[current_];
            }
            return path;
        }

        // Generate the children of the current node
        std::vector<Point> children = world.get_air_neighbours(current);

        // Loop through the children
        for (Point child : children) {

            // If the child is in the closed list, skip it
            // TODO: Could convert closed list to an octree for faster lookups
            bool skip = false;
            for (Point closed_child : closed) {
                if (child == closed_child) {
                    skip = true;
                    break;
                }
            }
            if (skip) {
                continue;
            }

            // Calculate the child's g, h, and f values
            float tentative_g = get_with_default(g, current, INFINITY) + heuristic(current, child);
            if (tentative_g < get_with_default(g, child, INFINITY)) {
                came_from[child] = &current;
                g[child] = tentative_g;
                f[child] = g[child] + heuristic(child, end);

                // If the child is not in the open list, add it
                if (std::find(open.begin(), open.end(), child) == open.end()) {
                    binary_insert(f, open, child);
                }
            }
        
        }

    }

    // If the open list is empty and the end node was not reached, return an empty path
    return path;
}