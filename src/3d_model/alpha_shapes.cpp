#include "alpha_shapes.h"

#include <algorithm>
#include <cmath>

AlphaShapes::AlphaShapes(Enviroment& world, double alpha) : world(world), alpha(alpha) {}

AlphaShapes::~AlphaShapes() {}

// bool AlphaShapes::isPointInCircumsphere(const Point p, const Tetrahedron& t) const {

//     return false;

// }

void AlphaShapes::computeVoxelAlphaShape(int x, int y, int z, std::unique_ptr<Octree>& octree) {

    if (octree == nullptr) {
        return;
    }

    std::unordered_set<Point> points = octree->get_points();

    std::vector<Point> pointsVec(points.begin(), points.end());

    double min_r = 9999;
    Tetrahedron min_r_tetra = Tetrahedron(Point(0, 0, 0), Point(0, 0, 0), Point(0, 0, 0), Point(0, 0, 0));

    // Implement the Incremental Flip Algorithm or Delaunay triangulation here
    // For simplicity, assume we triangulate all points and filter based on alpha
    for (size_t i = 0; i < points.size(); ++i) {
        for (size_t j = i + 1; j < points.size(); ++j) {
            for (size_t k = j + 1; k < points.size(); ++k) {
                for (size_t l = k + 1; l < points.size(); ++l) {
                    // Construct a tetrahedron
                    Tetrahedron t(pointsVec[i], pointsVec[j], pointsVec[k], pointsVec[l]);
                    double r = t.getCircumsphereRadius();
                    if (r < min_r) {
                        min_r = r;
                        min_r_tetra = t;
                    }

                    // Apply alpha condition (dummy filter for now)
                    if (t.getCircumsphereRadius() < alpha) {
                        voxelTetrahedra[x][y][z].push_back(t);
                    }
                }
            }
        }
    }
    std::cout << "Min r: " << min_r << std::endl;
    std::cout << "Min r tetra A: " << min_r_tetra.A.x << " " << min_r_tetra.A.y << " " << min_r_tetra.A.z << std::endl;
    std::cout << "Min r tetra B: " << min_r_tetra.B.x << " " << min_r_tetra.B.y << " " << min_r_tetra.B.z << std::endl;
    std::cout << "Min r tetra C: " << min_r_tetra.C.x << " " << min_r_tetra.C.y << " " << min_r_tetra.C.z << std::endl;
    std::cout << "Min r tetra D: " << min_r_tetra.D.x << " " << min_r_tetra.D.y << " " << min_r_tetra.D.z << std::endl;

}

// double Tetrahedron::getCircumsphereRadius() const {
//     // Lambda function for determinant calculation of a 4x4 matrix
//     auto det = [](double a, double b, double c, double d,
//                   double e, double f, double g, double h,
//                   double i, double j, double k, double l,
//                   double m, double n, double o, double p) {
//         return a * (f * (k * p - l * o) - g * (j * p - l * n) + h * (j * o - k * n)) -
//                b * (e * (k * p - l * o) - g * (i * p - l * m) + h * (i * o - k * m)) +
//                c * (e * (j * p - l * n) - f * (i * p - l * m) + h * (i * n - j * m)) -
//                d * (e * (j * o - k * n) - f * (i * o - k * m) + g * (i * n - j * m));
//     };

//     // Lambda for squaring a value
//     auto sq = [](double val) { return val * val; };

//     // Tetrahedron vertex coordinates relative to D
//     double a11 = A.x - D.x, a12 = A.y - D.y, a13 = A.z - D.z;
//     double a21 = B.x - D.x, a22 = B.y - D.y, a23 = B.z - D.z;
//     double a31 = C.x - D.x, a32 = C.y - D.y, a33 = C.z - D.z;

//     // Determinants for circumsphere calculations
//     double D1 = det(a11, a12, a13, 1,
//                     a21, a22, a23, 1,
//                     a31, a32, a33, 1,
//                     0,    0,    0, 1);

//     double Dx = det(sq(A.x) + sq(A.y) + sq(A.z) - sq(D.x) - sq(D.y) - sq(D.z), a12, a13, 1,
//                     sq(B.x) + sq(B.y) + sq(B.z) - sq(D.x) - sq(D.y) - sq(D.z), a22, a23, 1,
//                     sq(C.x) + sq(C.y) + sq(C.z) - sq(D.x) - sq(D.y) - sq(D.z), a32, a33, 1,
//                     0, 0, 0, 1);

//     double Dy = det(a11, sq(A.x) + sq(A.y) + sq(A.z) - sq(D.x) - sq(D.y) - sq(D.z), a13, 1,
//                     a21, sq(B.x) + sq(B.y) + sq(B.z) - sq(D.x) - sq(D.y) - sq(D.z), a23, 1,
//                     a31, sq(C.x) + sq(C.y) + sq(C.z) - sq(D.x) - sq(D.y) - sq(D.z), a33, 1,
//                     0, 0, 0, 1);

//     double Dz = det(a11, a12, sq(A.x) + sq(A.y) + sq(A.z) - sq(D.x) - sq(D.y) - sq(D.z), 1,
//                     a21, a22, sq(B.x) + sq(B.y) + sq(B.z) - sq(D.x) - sq(D.y) - sq(D.z), 1,
//                     a31, a32, sq(C.x) + sq(C.y) + sq(C.z) - sq(D.x) - sq(D.y) - sq(D.z), 1,
//                     0, 0, 0, 1);

//     double c = det(a11, a12, a13, sq(A.x) + sq(A.y) + sq(A.z) - sq(D.x) - sq(D.y) - sq(D.z),
//                    a21, a22, a23, sq(B.x) + sq(B.y) + sq(B.z) - sq(D.x) - sq(D.y) - sq(D.z),
//                    a31, a32, a33, sq(C.x) + sq(C.y) + sq(C.z) - sq(D.x) - sq(D.y) - sq(D.z),
//                    0, 0, 0, 1);

//     // Circumsphere center coordinates relative to D
//     double x = Dx / (2 * D1);
//     double y = Dy / (2 * D1);
//     double z = Dz / (2 * D1);

//     // Circumsphere radius
//     return std::sqrt(sq(x) + sq(y) + sq(z) - c / D1);
// }

double Tetrahedron::getCircumsphereRadius() const {
    auto sq = [](double v) { return v * v; };

    double a11 = A.x - D.x, a12 = A.y - D.y, a13 = A.z - D.z;
    double a21 = B.x - D.x, a22 = B.y - D.y, a23 = B.z - D.z;
    double a31 = C.x - D.x, a32 = C.y - D.y, a33 = C.z - D.z;

    double b1 = 0.5 * (sq(A.x) + sq(A.y) + sq(A.z) - sq(D.x) - sq(D.y) - sq(D.z));
    double b2 = 0.5 * (sq(B.x) + sq(B.y) + sq(B.z) - sq(D.x) - sq(D.y) - sq(D.z));
    double b3 = 0.5 * (sq(C.x) + sq(C.y) + sq(C.z) - sq(D.x) - sq(D.y) - sq(D.z));

    double det = a11 * (a22 * a33 - a23 * a32)
               - a12 * (a21 * a33 - a23 * a31)
               + a13 * (a21 * a32 - a22 * a31);

    double x = (b1 * (a22 * a33 - a23 * a32)
              - b2 * (a12 * a33 - a13 * a32)
              + b3 * (a12 * a23 - a13 * a22)) / det;

    double y = (a11 * (b2 * a33 - b3 * a32)
              - a12 * (b1 * a33 - b3 * a31)
              + a13 * (b1 * a32 - b2 * a31)) / det;

    double z = (a11 * (a22 * b3 - a23 * b2)
              - a12 * (a21 * b3 - a23 * b1)
              + a13 * (a21 * b2 - a22 * b1)) / det;

    Point center = { static_cast<int>(x), static_cast<int>(y), static_cast<int>(z) };
    double radius = center.distance(A);

    return radius;
}


// Helper: Merge boundary tetrahedra between two voxels
void AlphaShapes::mergeBoundaryTetrahedra(
    const std::unordered_set<Point>& sharedPoints,
    std::vector<Tetrahedron>& voxel1Tetrahedra,
    std::vector<Tetrahedron>& voxel2Tetrahedra
) {
    std::cout << "1 " << voxel1Tetrahedra.size() << std::endl;
    // Collect all tetrahedra near the boundary
    std::unordered_set<Tetrahedron> mergedTetrahedra(voxel1Tetrahedra.begin(), voxel1Tetrahedra.end());


    std::cout << "2 " << voxel2Tetrahedra.size() << std::endl;
    for (const Tetrahedron& t : voxel2Tetrahedra) {
        if (std::all_of(sharedPoints.begin(), sharedPoints.end(), [&](const Point& p) {
            return t.A == p || t.B == p || t.C == p || t.D == p;
        })) {
            mergedTetrahedra.insert(t);
        }
    }

    std::cout << "3" << std::endl;
    // Update both voxel tetrahedra (merged list)
    voxel1Tetrahedra.assign(mergedTetrahedra.begin(), mergedTetrahedra.end());
    voxel2Tetrahedra.assign(mergedTetrahedra.begin(), mergedTetrahedra.end());
    std::cout << "4" << std::endl;
}

void AlphaShapes::computeAlphaShapes() {

    world.apply_to_world([&](int x, int y, int z, std::unique_ptr<Octree>& octree) {
        computeVoxelAlphaShape(x, y, z, octree);
        if (voxelTetrahedra[x][y][z].size() > 0)
            std::cout << "Voxel: " << x << " " << y << " " << z << " Tetrahedra: " << voxelTetrahedra[x][y][z].size() << std::endl;
    });

    // Step 2: Merge tetrahedra at boundaries
    for (int i = 0; i < world.get_width(); ++i) {
        for (int j = 0; j < world.get_depth(); ++j) {
            for (int k = 0; k < world.get_height(); ++k) {
                std::vector<std::array<int, 3>> neighbours = world.get_neighbours(i, j, k);

                for (std::array<int, 3> n : neighbours) {
                    if (n[0] < i || n[1] < j || n[2] < k) {
                        continue;
                    }
                    std::cout << "Shared points: " << i << " " << j << " " << k << " " << n[0] << " " << n[1] << " " << n[2] << std::endl;
                    std::unordered_set<Point> sharedPoints = world.get_shared_points(i, j, k, n[0], n[1], n[2]);
                    std::cout << "Shared points: " << sharedPoints.size() << std::endl;

                    if (!sharedPoints.empty()) {
                        mergeBoundaryTetrahedra(
                            sharedPoints,
                            voxelTetrahedra[i][j][k],
                            voxelTetrahedra[n[0]][n[1]][n[2]]
                        );
                    }
                }
            }
        }
    }

}

// Retrieve all tetrahedra after merging
void AlphaShapes::calculateMergedAlphaShape() {

    tetrahedra.clear();
    for (int i = 0; i < world.get_width(); ++i) {
        for (int j = 0; j < world.get_depth(); ++j) {
            for (int k = 0; k < world.get_height(); ++k) {
                if (voxelTetrahedra[i][j][k].empty()) {
                    continue;
                }
                std::cout << "Tetrahedra: " << voxelTetrahedra[i][j][k].size() << std::endl;
                tetrahedra.insert(voxelTetrahedra[i][j][k].begin(), voxelTetrahedra[i][j][k].end());
            }
        }
    }

    std::cout << "Merged tetrahedra: " << tetrahedra.size() << std::endl;

}

void AlphaShapes::export_to_obj(const std::string& filename) const {
    std::ofstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Error: Could not open file " << filename << " for writing!" << std::endl;
        return;
    }

    int vertexIndex = 1;

    // Write vertices and faces
    for (const auto& tetra : tetrahedra) {

        std::cout << "Tetrahedron: " << tetra.A.x << " " << tetra.A.y << " " << tetra.A.z << std::endl;
        // Write vertices
        file << "v " << tetra.A.x << " " << tetra.A.y << " " << tetra.A.z << "\n";
        file << "v " << tetra.B.x << " " << tetra.B.y << " " << tetra.B.z << "\n";
        file << "v " << tetra.C.x << " " << tetra.C.y << " " << tetra.C.z << "\n";
        file << "v " << tetra.D.x << " " << tetra.D.y << " " << tetra.D.z << "\n";


        // Write faces (assuming vertices are written in sequence)
        file << "f " << vertexIndex << " " << vertexIndex + 1 << " " << vertexIndex + 2 << "\n";
        file << "f " << vertexIndex << " " << vertexIndex + 1 << " " << vertexIndex + 3 << "\n";
        file << "f " << vertexIndex << " " << vertexIndex + 2 << " " << vertexIndex + 3 << "\n";
        file << "f " << vertexIndex + 1 << " " << vertexIndex + 2 << " " << vertexIndex + 3 << "\n";

        vertexIndex += 4; // Increment vertex index for the next tetrahedron
    }

    file.close();
    std::cout << "Exported tetrahedra to " << filename << std::endl;
}