#pragma once

#include "../SpatialDivisionKernel.h"
#include "../cuda_utils/vec3_cu.hpp"            // Shared math types
#include "../cuda_utils/point_cu.hpp"           // Shared math types

#include <vector>
#include <memory> // For std::shared_ptr if needed, though maybe not directly here

// Forward declaration for the opaque internal data handle
struct MochiKernelData_impl;

class MochiKernel : public SpatialDivisionKernel
{
public:
    MochiKernel();
    ~MochiKernel() override;

    // Disable copy/assignment
    MochiKernel(const MochiKernel&) = delete;
    MochiKernel& operator=(const MochiKernel&) = delete;

    // --- SpatialDivisionKernel Interface ---
    MStatus build(const MObject& meshObject, const MBoundingBox& bbox, const MMatrix& offsetMatrix) override;
    std::vector<TriangleData> intersectKernelTriangle(const TriangleData& triangle) const override;
    K2KIntersection intersectKernelKernel(SpatialDivisionKernel& otherKernel) const override;
    // --- End Interface ---

private:
    // Opaque pointer to internal OptiX/CUDA data managed by the implementation (.cu file)
    // Could also be void*
    MochiKernelData_impl* _optixData = nullptr;

    // --- Host-side copies of mesh data for result reconstruction ---
    // We need this because the kernel returns indices, and we need the
    // original vertex positions to reconstruct Maya's TriangleData.
    std::vector<Point_cu> _hostOriginalVertices; // Vertices transformed by offsetMatrix
    std::vector<int> _hostOriginalIndices;       // Indices into _hostOriginalVertices (groups of 3)
    std::vector<int> _hostOriginalFaceIndices;   // Face index corresponding to each triangle in _hostOriginalIndices
    // Helper function to reconstruct TriangleData from indices
    TriangleData reconstructTriangleData(int originalTriangleIndex) const;
};
