#include "enviroment.h"
#include <vector>


Enviroment::Enviroment() {

    if (
        WORLD_WIDTH % DRONE_WIDTH != 0 ||
        WORLD_HEIGHT % DRONE_HEIGHT != 0 ||
        WORLD_DEPTH % DRONE_DEPTH != 0
    ) {
        std::cerr << "World dimensions must be divisible by drone dimensions" << std::endl;
        return;
    }

    width = WORLD_WIDTH / DRONE_WIDTH;
    depth = WORLD_DEPTH / DRONE_DEPTH;
    height = WORLD_HEIGHT / DRONE_HEIGHT;

    // Resize the 3D grid and initialize with nullptr (air)
    world.resize(width);
    for (int i = 0; i < width; ++i) {
        world[i].resize(depth);
        for (int j = 0; j < depth; ++j) {
            world[i][j].resize(height, nullptr);
        }
    }
};

Point Enviroment::update_enviroment(Point points[]) {

    for (int i = 0; i < BATCH_SIZE; ++i) {
        int x = points[i].x / DRONE_WIDTH;
        int y = points[i].y / DRONE_HEIGHT;
        int z = points[i].z / DRONE_DEPTH;

        if (world[x][y][z] == nullptr) {
            world[x][y][z] = std::make_unique<Octree>(x * DRONE_WIDTH, y * DRONE_HEIGHT, z * DRONE_DEPTH,
                                                      (x + 1) * DRONE_WIDTH, (y + 1) * DRONE_HEIGHT, (z + 1) * DRONE_DEPTH);
        }

        world[x][y][z]->insert(points[i].x, points[i].y, points[i].z);
    }
    return Point(0, 0, 0);
};

bool Enviroment::is_air(int x, int y, int z) {
    return world[x][y][z] == nullptr;
};

std::vector<int[3]> Enviroment::get_air_neighbours(int x, int y, int z) {
    std::vector<int[3]> neighbours;

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

                if (is_air(x_, y_, z_)) {
                    neighbours.push_back({x_, y_, z_});
                }
            }
        }
    }

    return neighbours;
};

std::vector<int[3]> Enviroment::get_neighbours(int x, int y, int z) {
    std::vector<int[3]> neighbours;

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

                neighbours.push_back({x_, y_, z_});
            }
        }
    }

    return neighbours;
}


std::unordered_set<Point> Enviroment::get_shared_points(int x1, int y1, int z1, int x2, int y2, int z2) {
    std::unordered_set<Point> sharedPoints;
    
    if ((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1) + (z2 - z1) * (z2 - z1) != 1) {
        return sharedPoints;
    }


    std::unordered_set<Point> points1 = world[x1][y1][z1]->get_points();
    std::unordered_set<Point> points2 = world[x2][y2][z2]->get_points();

    for (const Point& p : points1) {
        if (points2.find(p) != points2.end()) {
            sharedPoints.insert(p);
        }
    }
    
    return sharedPoints;
}

std::unordered_set<Point> Enviroment::get_points() {
    std::unordered_set<Point> points;
    for (int i = 0; i < width; ++i) {
        for (int j = 0; j < depth; ++j) {
            for (int k = 0; k < height; ++k) {
                if (world[i][j][k] != nullptr) {
                    std::unordered_set<Point> pnts = world[i][j][k]->get_points();
                    points.insert(pnts.begin(), pnts.end());
                }
            }
        }
    }
    return points;
}

void Enviroment::apply_to_world(std::function<void(int x, int y, int z, std::unique_ptr<Octree>&)> func) {
    for (int i = 0; i < width; ++i) {
        for (int j = 0; j < depth; ++j) {
            for (int k = 0; k < height; ++k) {
                func(i, j, k, world[i][j][k]);
            }
        }
    }
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