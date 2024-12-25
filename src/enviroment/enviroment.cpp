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

    // Voxel is unscanned -> octree = nullptr
    // Voxel is air -> octree = empty
    // Voxel is occupied -> octree = points

    // Also defined on world declaration
    width = WORLD_WIDTH / DRONE_WIDTH;
    depth = WORLD_DEPTH / DRONE_DEPTH;
    height = WORLD_HEIGHT / DRONE_HEIGHT;

};


//TODO: Add point to multiple voxels if it is on the boundary
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

            if (world[x_][y_][z_] == nullptr) {
                world[x_][y_][z_] = std::make_unique<Octree>(x_ * DRONE_WIDTH, y_ * DRONE_HEIGHT, z_ * DRONE_DEPTH,
                                                            (x_ + 1) * DRONE_WIDTH, (y_ + 1) * DRONE_HEIGHT, (z_ + 1) * DRONE_DEPTH);
            }
        }
        
        world[x][y][z]->insert(points[i].x, points[i].y, points[i].z);
    }
    return Point(0, 0, 0);
};

bool Enviroment::is_air(int x, int y, int z) {

    if (world[x][y][z] == nullptr) {
        return false;
    }
    
    return world[x][y][z]->is_empty();

};

std::vector<std::array<int, 3>> Enviroment::get_air_neighbours(int x, int y, int z) {
    std::vector<std::array<int, 3>> neighbours;

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

std::vector<std::array<int, 3>> Enviroment::get_neighbours(int x, int y, int z) {
    std::vector<std::array<int, 3>> neighbours;

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

    if (world[x1][y1][z1] == nullptr || world[x2][y2][z2] == nullptr) {
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