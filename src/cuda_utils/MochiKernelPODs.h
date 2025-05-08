#pragma once

#include "vec3_cu.hpp" // Assuming this exists from the sample
#include "point_cu.hpp" // Assuming this exists from the sample

// POD version of Maya's TriangleData (or similar info)
struct MochiTrianglePOD {
    Point_cu vertices[3];
    int faceIndex;
    int triangleIndex; // Index within the original face
};

// Structure to hold collision results (indices of original triangles)
struct MochiCollisionPairPOD {
    int triangleIndexA; // Index into original triangle list of mesh A
    int triangleIndexB; // Index into original triangle list of mesh B
    int faceIndexA;     // Original face index for triangle A
    int faceIndexB;     // Original face index for triangle B
};
