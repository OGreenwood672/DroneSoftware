#ifndef GENERATE_FIBONACCI_SPHERE_VECTORS_H
#define GENERATE_FIBONACCI_SPHERE_VECTORS_H

inline std::vector<std::array<float, 3>> generate_fibonacci_sphere_vectors(int n) {
    std::vector<std::array<float, 3>> vectors;

    float offset = 2.0 / n;
    float increment = M_PI * (3.0 - sqrt(5.0));

    for (int i = 0; i < n; ++i) {
        float y = ((i * offset) - 1) + (offset / 2);
        float r = sqrt(1 - pow(y, 2));
        float phi = ((i + 1) % n) * increment;

        float x = cos(phi) * r;
        float z = sin(phi) * r;

        vectors.push_back({x, y, z});
    }

    return vectors;
}

#endif // GENERATE_FIBONACCI_SPHERE_VECTORS_H