#include "../point/point.h"
#include "../enviroment.h"

#include <vector>
#include <unordered_map>
#include <limits>
#include <array>

float heuristic(EnviromentBlock* a, EnviromentBlock* b) {
    std::array<int, 3> a_point = a->get_position();
    std::array<int, 3> b_point = b->get_position();

    return std::abs(a_point[0] - b_point[0]) + std::abs(a_point[1] - b_point[1]) + std::abs(a_point[2] - b_point[2]);
}

void binary_insert(std::unordered_map<EnviromentBlock*, float> scores, std::vector<EnviromentBlock*>& v, EnviromentBlock* p) {
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

float get_with_default(std::unordered_map<EnviromentBlock*, float>& map, EnviromentBlock* p, float default_value) {
    if (map.count(p) == 0) {
        map[p] = default_value;
        return default_value;
    }
    return map[p];
}


std::vector<EnviromentBlock*> a_star(Enviroment& world, EnviromentBlock* start, EnviromentBlock* end) {
    std::vector<EnviromentBlock*> path;

    // get start and end points
    std::array<int, 3> start_point = start->get_position();
    std::array<int, 3> end_point = end->get_position();

    // If the start and end points are the same, return the start point
    if (start_point[0] == end_point[0] && start_point[1] == end_point[1] && start_point[2] == end_point[2]) {
        path.push_back(start);
        return path;
    }

    // Initialize the open and closed lists
    std::vector<EnviromentBlock*> open;
    std::vector<EnviromentBlock*> closed;

    std::unordered_map<EnviromentBlock*, EnviromentBlock*> came_from;
    std::unordered_map<EnviromentBlock*, float> g;
    std::unordered_map<EnviromentBlock*, float> f;

    came_from[start] = nullptr;
    g[start] = 0;
    f[start] = g[start] + heuristic(start, end);

    // Add the start point to the open list
    open.push_back(start);

    // While the open list is not empty
    while (open.size() > 0) {

        // Get the current node
        EnviromentBlock* current = open[0];

        // Pop the current node from the open list
        open.erase(open.begin());

        // Add the current node to the closed list
        closed.push_back(current);

        // If the current node is the end node, return the path
        if (*current == *end) {
            EnviromentBlock* current_ = current;
            while (came_from[current_] != nullptr) {
                path.push_back(current_);
                current_ = came_from[current_];
            }
            return path;
        }

        // Generate the children of the current node
        std::array<int, 3> current_point = current->get_position();
        std::vector<EnviromentBlock*> children = world.get_air_neighbours(current_point[0], current_point[1], current_point[2]);

        // Loop through the children
        for (EnviromentBlock* child : children) {

            // If the child is in the closed list, skip it
            // TODO: Could convert closed list to an unordered_set for O(1) lookup
            bool skip = false;
            for (EnviromentBlock* closed_child : closed) {
                if (*child == *closed_child) {
                    skip = true;
                    break;
                }
            }
            if (skip) {
                continue;
            }

            // Calculate the child's g, h, and f values
            float tentative_g = get_with_default(g, current, std::numeric_limits<double>::infinity()) + heuristic(current, child);
            if (tentative_g < get_with_default(g, child, std::numeric_limits<double>::infinity())) {
                came_from[child] = current;
                g[child] = tentative_g;
                f[child] = g[child] + heuristic(child, end);

                // If the child is in the open list, remove it
                // Add the child to the open list
                bool in_open = false;
                for (int i = 0; i < open.size(); ++i) {
                    if (child == open[i]) {
                        open.erase(open.begin() + i);
                        in_open = true;
                        break;
                    }
                }
                binary_insert(f, open, child);
            }
        
        }

    }

    // If the open list is empty and the end node was not reached, return an empty path
    return path;
}