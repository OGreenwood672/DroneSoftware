#include "Octree.h"
#include <algorithm>

// Octree constructors
// Octree::Octree() : point(new Point()) {}

// Octree is a leaf if it is a nullptr
// Octree is a node if it is not a nullptr
// Octree is an internal node if it's point is a nullptr

Octree::Octree(int x, int y, int z, int x1, int y1, int z1, int x2, int y2, int z2) : point(new Point(x, y, z)) {

    if (x2 < x1 || y2 < y1 || z2 < z1) {
        std::cerr << "Boundary points are not valid" << std::endl;
        return;
    }

    topLeftFront = new Point(x1, y1, z1);
    bottomRightBack = new Point(x2, y2, z2);

    children.assign(8, nullptr);

}

Octree::Octree(int x1, int y1, int z1, int x2, int y2, int z2) {
    if (x2 < x1 || y2 < y1 || z2 < z1) {
        std::cerr << "Boundary points are not valid" << std::endl;
        return;
    }

    point = nullptr;
    topLeftFront = new Point(x1, y1, z1);
    bottomRightBack = new Point(x2, y2, z2);

    children.assign(8, nullptr);
    // for (int i = TopLeftFront; i <= BottomLeftBack; ++i) {
    //     int _x1 = (i & 1) ? x1 : (x1 + x2) / 2;
    //     int _x2 = (i & 1) ? (x1 + x2) / 2 : x2;
    //     int _y1 = (i & 2) ? y1 : (y1 + y2) / 2;
    //     int _y2 = (i & 2) ? (y1 + y2) / 2 : y2;
    //     int _z1 = (i & 4) ? z1 : (z1 + z2) / 2;
    //     int _z2 = (i & 4) ? (z1 + z2) / 2 : z2;
    //     children[i] = new Octree(_x1, _y1, _z1, _x2, _y2, _z2);
    // }
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
        std::cout << "Point is out of bounds" << "x: " << x << " y: " << y << " z: " << z << std::endl;
        return;
    }

    int midx = (topLeftFront->x + bottomRightBack->x) / 2;
    int midy = (topLeftFront->y + bottomRightBack->y) / 2;
    int midz = (topLeftFront->z + bottomRightBack->z) / 2;

    int pos = determineChildPosition(x, y, z, midx, midy, midz);
    
    if (children[pos] == nullptr) { // Leaf Node

        if (pos == TopLeftFront) {
            children[pos] = new Octree(x, y, z, topLeftFront->x, topLeftFront->y, topLeftFront->z, midx, midy, midz);
        } else if (pos == TopRightFront) {
            children[pos] = new Octree(x, y, z, midx, topLeftFront->y, topLeftFront->z, bottomRightBack->x, midy, midz);
        } else if (pos == BottomRightFront) {
            children[pos] = new Octree(x, y, z, midx, midy, topLeftFront->z, bottomRightBack->x, bottomRightBack->y, midz);
        } else if (pos == BottomLeftFront) {
            children[pos] = new Octree(x, y, z, topLeftFront->x, midy, topLeftFront->z, midx, bottomRightBack->y, midz);
        } else if (pos == TopLeftBottom) {
            children[pos] = new Octree(x, y, z, topLeftFront->x, topLeftFront->y, midz, midx, midy, bottomRightBack->z);
        } else if (pos == TopRightBottom) {
            children[pos] = new Octree(x, y, z, midx, topLeftFront->y, midz, bottomRightBack->x, midy, bottomRightBack->z);
        } else if (pos == BottomRightBack) {
            children[pos] = new Octree(x, y, z, midx, midy, midz, bottomRightBack->x, bottomRightBack->y, bottomRightBack->z);
        } else if (pos == BottomLeftBack) {
            children[pos] = new Octree(x, y, z, topLeftFront->x, midy, midz, midx, bottomRightBack->y, bottomRightBack->z);
        }

    } else if (children[pos]->get_point() == nullptr) { // Internal Node

        children[pos]->insert(x, y, z);

    } else {

        int x_ = children[pos]->get_point()->x;
        int y_ = children[pos]->get_point()->y;
        int z_ = children[pos]->get_point()->z;
        delete children[pos];


        if (pos == TopLeftFront) {
            children[pos] = new Octree(topLeftFront->x, topLeftFront->y, topLeftFront->z, midx, midy, midz);
        } else if (pos == TopRightFront) {
            children[pos] = new Octree(midx, topLeftFront->y, topLeftFront->z, bottomRightBack->x, midy, midz);
        } else if (pos == BottomRightFront) {
            children[pos] = new Octree(midx, midy, topLeftFront->z, bottomRightBack->x, bottomRightBack->y, midz);
        } else if (pos == BottomLeftFront) {
            children[pos] = new Octree(topLeftFront->x, midy, topLeftFront->z, midx, bottomRightBack->y, midz);
        } else if (pos == TopLeftBottom) {
            children[pos] = new Octree(topLeftFront->x, topLeftFront->y, midz, midx, midy, bottomRightBack->z);
        } else if (pos == TopRightBottom) {
            children[pos] = new Octree(midx, topLeftFront->y, midz, bottomRightBack->x, midy, bottomRightBack->z);
        } else if (pos == BottomRightBack) {
            children[pos] = new Octree(midx, midy, midz, bottomRightBack->x, bottomRightBack->y, bottomRightBack->z);
        } else if (pos == BottomLeftBack) {
            children[pos] = new Octree(topLeftFront->x, midy, midz, midx, bottomRightBack->y, bottomRightBack->z);
        }

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

    if (children[pos] == nullptr) {
        return false;
    } else if (children[pos]->point == nullptr) {
        return children[pos]->find(x, y, z);
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

bool Octree::is_empty() const {
    return std::all_of(children.begin(), children.end(), [](Octree* child) {
        return child == nullptr;
    });
}

Point* Octree::get_point() const {
    return point;
}

Point* Octree::get_topLeftFront() const {
    return topLeftFront;
}

Point* Octree::get_bottomRightBack() const {
    return bottomRightBack;
}