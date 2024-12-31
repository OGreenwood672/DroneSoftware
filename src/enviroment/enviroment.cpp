#include "enviroment.h"
#include <vector>
#include <fstream>


Enviroment::Enviroment() {

    if (
        WORLD_WIDTH % DRONE_WIDTH != 0 ||
        WORLD_HEIGHT % DRONE_HEIGHT != 0 ||
        WORLD_DEPTH % DRONE_DEPTH != 0
    ) {
        std::cerr << "World dimensions must be divisible by drone dimensions" << std::endl;
        return;
    }

    // Also defined on world declaration
    width = WORLD_WIDTH / DRONE_WIDTH;
    depth = WORLD_DEPTH / DRONE_DEPTH;
    height = WORLD_HEIGHT / DRONE_HEIGHT;

    for (int i = 0; i < width; ++i) {
        for (int j = 0; j < depth; ++j) {
            for (int k = 0; k < height; ++k) {
                world[i][j][k] = new EnviromentBlock(i, j, k);
            }
        }
    }

};

Enviroment::~Enviroment() {
    for (int i = 0; i < width; ++i) {
        for (int j = 0; j < depth; ++j) {
            for (int k = 0; k < height; ++k) {
                delete world[i][j][k];
            }
        }
    }
};


//TODO: Add point to multiple voxels if it is on the boundary
//TODO: Add confidence level of air block
//TODO: Add SLAM (return offset of drone from origin)
Point Enviroment::update_enviroment(Point origin, Point points[]) {

    int origin_x = origin.x / DRONE_WIDTH;
    int origin_y = origin.y / DRONE_HEIGHT;
    int origin_z = origin.z / DRONE_DEPTH;

    for (int i = 0; i < BATCH_SIZE; ++i) {
        int x = points[i].x / DRONE_WIDTH;
        int y = points[i].y / DRONE_HEIGHT;
        int z = points[i].z / DRONE_DEPTH;

        std::vector<std::array<int, 3>> path = bresenham_3d({origin_x, origin_y, origin_z}, {x, y, z});

        for (std::array<int, 3> p : path) {
            int x_ = p[0];
            int y_ = p[1];
            int z_ = p[2];

            world[x_][y_][z_]->increment_air_count();
        }
        
        world[x][y][z]->add_point(points[i]);
    }
    return Point(0, 0, 0);
};

std::vector<EnviromentBlock*> Enviroment::get_air_neighbours(int x, int y, int z) {
    std::vector<EnviromentBlock*> neighbours;

    for (int i = -1; i <= 1; ++i) {
        for (int j = -1; j <= 1; ++j) {
            for (int k = -1; k <= 1; ++k) {
                if (i == 0 && j == 0 && k == 0) {
                    continue;
                }

                int x_ = x + i;
                int y_ = y + j;
                int z_ = z + k;

                if (x_ < 0 || x_ >= width || y_ < 0 || y_ >= depth || z_ < 0 || z_ >= height) {
                    continue;
                }

                if (get_block(x_, y_, z_)->is_air()) {
                    neighbours.push_back(get_block(x_, y_, z_));
                }
            }
        }
    }

    return neighbours;
};

std::vector<EnviromentBlock*> Enviroment::get_neighbours(int x, int y, int z) {
    std::vector<EnviromentBlock*> neighbours;

    for (int i = -1; i <= 1; ++i) {
        for (int j = -1; j <= 1; ++j) {
            for (int k = -1; k <= 1; ++k) {
                if (i == 0 && j == 0 && k == 0) {
                    continue;
                }

                int x_ = x + i;
                int y_ = y + j;
                int z_ = z + k;

                if (x_ < 0 || x_ >= width || y_ < 0 || y_ >= depth || z_ < 0 || z_ >= height) {
                    continue;
                }

                neighbours.push_back(get_block(x_, y_, z_));
            }
        }
    }

    return neighbours;
}

std::unordered_set<Point> Enviroment::get_points() const {
    std::unordered_set<Point> points;
    for (int i = 0; i < width; ++i) {
        for (int j = 0; j < depth; ++j) {
            for (int k = 0; k < height; ++k) {
                if (world[i][j][k] != nullptr) {
                    std::unordered_set<Point> pnts = world[i][j][k]->get_points();
                    // coord and number of points
                    std::cout << i << " " << j << " " << k << " " << pnts.size() << std::endl;
                    points.insert(pnts.begin(), pnts.end());
                }
            }
        }
    }
    return points;
}

void Enviroment::export_point_cloud(const std::string& filename) const {
    std::ofstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << " for writing!" << std::endl;
        return;
    }

    std::unordered_set<Point> points = get_points();

    // get max coord
    int max_coord = 0;
    for (const Point& p : points) {
        max_coord = std::max(max_coord, std::max(p.x, std::max(p.y, p.z)));
    }

    // Add ply header
    file << "ply" << std::endl;
    file << "format ascii 1.0" << std::endl;
    file << "element vertex " << get_points().size() << std::endl;
    file << "property float x" << std::endl;
    file << "property float y" << std::endl;
    file << "property float z" << std::endl;
    file << "end_header" << std::endl;

    for (const Point& p : points) {
        file << static_cast<double>(p.x) / max_coord << " " << static_cast<double>(p.y) / max_coord << " " << static_cast<double>(p.z) / max_coord << std::endl;
    }

    std::cout << "Exported point cloud to " << filename << " :)" << std::endl;

    file.close();

}

void Enviroment::apply_to_world(std::function<void(int x, int y, int z, EnviromentBlock* block)> func) {
    for (int i = 0; i < width; ++i) {
        for (int j = 0; j < depth; ++j) {
            for (int k = 0; k < height; ++k) {
                func(i, j, k, get_block(i, j, k));
            }
        }
    }
}

EnviromentBlock* Enviroment::get_block(int x, int y, int z) const {
    return world[x][y][z];
}

int Enviroment::get_width() const {
    return width;
}

int Enviroment::get_depth() const {
    return depth;
}

int Enviroment::get_height() const {
    return height;
}

float Enviroment::calculate_scan_score(std::array<int, 3> origin, int directions) const {

    std::vector<std::array<float, 3>> vectors = generate_fibonacci_sphere_vectors(directions);

    float score = 0;

    std::array<int, 3> min_bound = {0, 0, 0};
    std::array<int, 3> max_bound = {get_width(), get_depth(), get_height()};

    for (std::array<float, 3> vector : vectors) {
        
        // Calculate furthest part of the world from the origin with vector
        std::array<int, 3> furthest = get_furthest_point(min_bound, max_bound, origin, vector);
        std::vector<std::array<int, 3>> points = bresenham_3d(origin, furthest);

        for (std::array<int, 3> point : points) {
            EnviromentBlock* block = get_block(point[0], point[1], point[2]);
            if (block->is_unscanned()) {
                score += 1;
                break;
            } else if (block->is_block()) {
                break;
            }
        }

    }

    return score / directions;

}