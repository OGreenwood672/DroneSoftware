#include "Octree.h"

// Octree constructors
Octree::Octree() : point(new Point()) {}

Octree::Octree(int x, int y, int z) : point(new Point(x, y, z)) {}

Octree::Octree(int x1, int y1, int z1, int x2, int y2, int z2) {
    if (x2 < x1 || y2 < y1 || z2 < z1) {
        std::cerr << "Boundary points are not valid" << std::endl;
        return;
    }

    point = nullptr;
    topLeftFront = new Point(x1, y1, z1);
    bottomRightBack = new Point(x2, y2, z2);

    children.assign(8, nullptr);
    for (int i = TopLeftFront; i <= BottomLeftBack; ++i)
        children[i] = new Octree();
}

// Destructor to clean up dynamic memory
Octree::~Octree() {
    delete point;
    delete topLeftFront;
    delete bottomRightBack;
    for (auto child : children) {
        delete child;
    }
}

// Helper to determine the child position
int Octree::determineChildPosition(int x, int y, int z, int midx, int midy, int midz) const {
    if (x <= midx) {
        if (y <= midy) {
            return (z <= midz) ? TopLeftFront : TopLeftBottom;
        } else {
            return (z <= midz) ? BottomLeftFront : BottomLeftBack;
        }
    } else {
        if (y <= midy) {
            return (z <= midz) ? TopRightFront : TopRightBottom;
        } else {
            return (z <= midz) ? BottomRightFront : BottomRightBack;
        }
    }
}

// Insert a point into the Octree
void Octree::insert(int x, int y, int z) {
    if (find(x, y, z)) {
        std::cout << "Point already exists in the tree" << std::endl;
        return;
    }

    if (x < topLeftFront->x || x > bottomRightBack->x ||
        y < topLeftFront->y || y > bottomRightBack->y ||
        z < topLeftFront->z || z > bottomRightBack->z) {
        std::cout << "Point is out of bounds" << std::endl;
        return;
    }

    int midx = (topLeftFront->x + bottomRightBack->x) / 2;
    int midy = (topLeftFront->y + bottomRightBack->y) / 2;
    int midz = (topLeftFront->z + bottomRightBack->z) / 2;

    int pos = determineChildPosition(x, y, z, midx, midy, midz);

    if (children[pos]->point == nullptr) {
        children[pos]->insert(x, y, z);
    } else if (children[pos]->point->x == -1) {
        delete children[pos];
        children[pos] = new Octree(x, y, z);
    } else {
        int x_ = children[pos]->point->x;
        int y_ = children[pos]->point->y;
        int z_ = children[pos]->point->z;
        delete children[pos];
        children[pos] = nullptr;

        if (pos == TopLeftFront) {
            children[pos] = new Octree(topLeftFront->x, topLeftFront->y, topLeftFront->z, midx, midy, midz);
        } // Repeat for other octants...

        children[pos]->insert(x_, y_, z_);
        children[pos]->insert(x, y, z);
    }
}

// Find if a point exists in the Octree
bool Octree::find(int x, int y, int z) const {
    if (x < topLeftFront->x || x > bottomRightBack->x ||
        y < topLeftFront->y || y > bottomRightBack->y ||
        z < topLeftFront->z || z > bottomRightBack->z) {
        return false;
    }

    int midx = (topLeftFront->x + bottomRightBack->x) / 2;
    int midy = (topLeftFront->y + bottomRightBack->y) / 2;
    int midz = (topLeftFront->z + bottomRightBack->z) / 2;

    int pos = determineChildPosition(x, y, z, midx, midy, midz);

    if (children[pos]->point == nullptr) {
        return children[pos]->find(x, y, z);
    } else if (children[pos]->point->x == -1) {
        return false;
    } else {
        return (x == children[pos]->point->x && y == children[pos]->point->y && z == children[pos]->point->z);
    }
}


std::unordered_set<Point> Octree::get_points() const {
    std::unordered_set<Point> points;
    if (point != nullptr) {
        points.insert(*point);
    } else {
        for (auto child : children) {
            if (child != nullptr) {
                std::unordered_set<Point> child_points = child->get_points();
                points.insert(child_points.begin(), child_points.end());
            }
        }
    }
    return points;
}