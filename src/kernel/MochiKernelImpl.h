#pragma once

#include <vector>
#include "../cuda_utils/vec3_cu.hpp"      // Shared
#include "../cuda_utils/point_cu.hpp"     // Shared
#include "../cuda_utils/transfo.hpp"      // Shared
#include "../cuda_utils/MochiKernelPODs.h" // Shared PODs

// Forward declaration (implementation in .cu)
struct MochiKernelData_impl;

// --- Bridging Functions (Implemented in MochiKernelImpl.cu) ---
extern "C" bool initializeMochiOptiX(); // Initializes OptiX/CUDA context
extern "C" void cleanupMochiOptiX();    // Cleans up OptiX/CUDA context

// Creates OptiX acceleration structure and returns an opaque handle.
// Takes ownership of data conceptually, copies it to GPU.
extern "C" MochiKernelData_impl* mochiKernelBuild_impl(
    const std::vector<Point_cu>& vertices,    // Host vertices (already transformed)
    const std::vector<int>& indices,          // Host triangle indices (into vertices)
    const std::vector<int>& faceIndices);    // Host face index for each triangle

// Destroys OptiX/CUDA resources associated with the handle.
extern "C" void mochiKernelDestroy_impl(MochiKernelData_impl* handle);

// Intersects rays from a single triangle against the BVH.
// Returns indices of *original* triangles in the BVH that were hit.
extern "C" void mochiKernelIntersectTriangle_impl(
    const MochiKernelData_impl* bvhHandle,
    const MochiTrianglePOD& inputTriangle,          // Triangle to shoot rays from
    std::vector<int>& outCollidingTriangleIndices); // Indices of original triangles hit in BVH

// Intersects two Mochi Kernels using OptiX.
extern "C" void mochiKernelIntersect_impl(
    const MochiKernelData_impl* handleA,             // BVH for kernel A
    const MochiKernelData_impl* handleB,             // BVH for kernel B
    bool selfIntersection,                           // True if handleA == handleB
    std::vector<MochiCollisionPairPOD>& outCollisionPairs); // Output pairs of colliding original triangle indices


// --- End Bridging Functions ---
