#include "enviroment_block.h"


EnviromentBlock::EnviromentBlock(int x, int y, int z) : x(x), y(y), z(z), point_count(0), air_count(0) {};

EnviromentBlock::~EnviromentBlock() {};

void EnviromentBlock::set_position(int x, int y, int z) {
    this->x = x;
    this->y = y;
    this->z = z;
};

std::array<int, 3> EnviromentBlock::get_position() const {
    return {x, y, z};
};

std::unordered_set<Point> EnviromentBlock::get_points() const {
    if (octree == nullptr) {
        return std::unordered_set<Point>();
    }
    return octree->get_points();
};

float EnviromentBlock::get_distance(EnviromentBlock* other) const {
    std::array<int, 3> other_pos = other->get_position();
    return sqrt(pow(x - other_pos[0], 2) + pow(y - other_pos[1], 2) + pow(z - other_pos[2], 2));
};

bool EnviromentBlock::is_air() const {
    if (octree == nullptr) {
        return false;
    }
    return air_count > 0;
};

bool EnviromentBlock::is_unscanned() const {
    return octree == nullptr;
};

bool EnviromentBlock::is_block() const {
    if (octree == nullptr) {
        return false;
    }
    return point_count > 0;
};

Point EnviromentBlock::get_center() const {
    return Point(
        x * DRONE_WIDTH + DRONE_WIDTH / 2,
        y * DRONE_DEPTH + DRONE_DEPTH / 2,
        z * DRONE_HEIGHT + DRONE_HEIGHT / 2
    );
};

std::unordered_set<Point> EnviromentBlock::get_shared_points(EnviromentBlock* block) const {

    // If not neighbouts
    if (abs(x - block->x) + abs(y - block->y) + abs(z - block->z) != 1) {
        return std::unordered_set<Point>();
    }

    std::unordered_set<Point> points1 = get_points();
    std::unordered_set<Point> points2 = block->get_points();

    std::unordered_set<Point> sharedPoints;
    for (const Point& p : points1) {
        if (points2.find(p) != points2.end()) {
            sharedPoints.insert(p);
        }
    }
    
    return sharedPoints;
};

void EnviromentBlock::increment_air_count() {
    air_count++;
};

void EnviromentBlock::add_point(Point p) {
    if (octree == nullptr) {
        int x = p.x / DRONE_WIDTH;
        int y = p.y / DRONE_DEPTH;
        int z = p.z / DRONE_HEIGHT;
        octree = std::make_unique<Octree>(
            x * DRONE_WIDTH,
            y * DRONE_DEPTH,
            z * DRONE_HEIGHT,
            (x + 1) * DRONE_WIDTH,
            (y + 1) * DRONE_DEPTH,
            (z + 1) * DRONE_HEIGHT
        );
    }
    octree->insert(p.x, p.y, p.z);
    point_count++;
};

// Make hash function for EnviromentBlock
namespace std {
    template <>
    struct hash<EnviromentBlock> {
        std::size_t operator()(const EnviromentBlock& block) const {
            std::array<int, 3> pos = block.get_position();
            return std::hash<int>()(pos[0]) ^ std::hash<int>()(pos[1]) ^ std::hash<int>()(pos[2]);
        }
    };
}