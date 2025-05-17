#define NO_CUDA // Prevent CUDA headers

#include "MochiKernel.h"
#include "MochiKernelImpl.h" // For bridging function declarations
#include "../cuda_utils/MochiKernelPODs.h"  // For POD struct definitions
#include "../cuda_utils/transfo.hpp"        // For MMatrix -> Transfo conversion

#include <maya/MObject.h>
#include <maya/MFnMesh.h>
#include <maya/MItMeshPolygon.h>
#include <maya/MPointArray.h>
#include <maya/MIntArray.h>
#include <maya/MStatus.h>
#include <maya/MMatrix.h>
#include <maya/MGlobal.h>
#include <maya/MBoundingBox.h> // Include for MBoundingBox if used

#include <stdexcept> // For runtime_error
#include <iostream>  // For debug

// Helper to convert Maya MMatrix to shared Transfo
Transfo MMatrixToTransfo(const MMatrix& mmat) {
    Transfo mat;
    // Note: MMatrix is column-major, Transfo might be row-major (check transfo.hpp sample)
    // Assuming Transfo is row-major as in the sample:
    mat[0] = (float)mmat[0][0]; mat[1] = (float)mmat[0][1]; mat[2] = (float)mmat[0][2]; mat[3] = (float)mmat[0][3];
    mat[4] = (float)mmat[1][0]; mat[5] = (float)mmat[1][1]; mat[6] = (float)mmat[1][2]; mat[7] = (float)mmat[1][3];
    mat[8] = (float)mmat[2][0]; mat[9] = (float)mmat[2][1]; mat[10] = (float)mmat[2][2]; mat[11] = (float)mmat[2][3];
    mat[12] = (float)mmat[3][0]; mat[13] = (float)mmat[3][1]; mat[14] = (float)mmat[3][2]; mat[15] = (float)mmat[3][3];
    // The sample Transfo applies translation differently in operator* vs project.
    // We likely only care about the 3x4 affine part for transforming vertices/normals.
    // Let's adjust based on the sample's operator*(Point_cu):
     mat[0] = (float) mmat[0][0]; mat[1] = (float) mmat[1][0]; mat[2] = (float) mmat[2][0]; mat[3] = (float) mmat[3][0];
     mat[4] = (float) mmat[0][1]; mat[5] = (float) mmat[1][1]; mat[6] = (float) mmat[2][1]; mat[7] = (float) mmat[3][1];
     mat[8] = (float) mmat[0][2]; mat[9] = (float) mmat[1][2]; mat[10] = (float) mmat[2][2]; mat[11] = (float) mmat[3][2];
     mat[12] = (float) mmat[0][3]; mat[13] = (float) mmat[1][3]; mat[14] = (float) mmat[2][3]; mat[15] = (float) mmat[3][3];

    // Assuming Transfo stores matrix for p' = M * p where p is (x,y,z,1)
    // And that Transfo expects row-major storage m[row*4 + col]
    // MMatrix m[row][col]
    Transfo tr(
        (float)mmat[0][0], (float)mmat[1][0], (float)mmat[2][0], (float)mmat[3][0], // Col 0 -> Row 0
        (float)mmat[0][1], (float)mmat[1][1], (float)mmat[2][1], (float)mmat[3][1], // Col 1 -> Row 1
        (float)mmat[0][2], (float)mmat[1][2], (float)mmat[2][2], (float)mmat[3][2], // Col 2 -> Row 2
        (float)mmat[0][3], (float)mmat[1][3], (float)mmat[2][3], (float)mmat[3][3]  // Col 3 -> Row 3
    );
    return tr; // Ensure transfo.hpp constructor matches this layout
}

MochiKernel::MochiKernel() : _optixData(nullptr) {
    // std::cout << "MochiKernel Created" << std::endl;
}

MochiKernel::~MochiKernel() {
    // std::cout << "MochiKernel Destroying..." << std::endl;
    if (_optixData) {
        mochiKernelDestroy_impl(_optixData);
        _optixData = nullptr;
    }
    // std::cout << "MochiKernel Destroyed" << std::endl;
}

MStatus MochiKernel::build(const MObject& meshObject, const MBoundingBox& bbox, const MMatrix& offsetMatrix) {
    MStatus status;

    // 1. Cleanup previous data if any
    if (_optixData) {
        mochiKernelDestroy_impl(_optixData);
        _optixData = nullptr;
        _hostOriginalVertices.clear();
        _hostOriginalIndices.clear();
        _hostOriginalFaceIndices.clear();
    }

    // 2. Extract Mesh Data from Maya Object
    MFnMesh meshFn(meshObject, &status);
    if (!status) {
        MGlobal::displayError("MochiKernel::build: MFnMesh failed");
        return status;
    }

    MPointArray mayaVertices;
    meshFn.getPoints(mayaVertices, MSpace::kObject); // Get vertices in object space

    MIntArray triangleCounts;
    MIntArray triangleVertices;
    meshFn.getTriangles(triangleCounts, triangleVertices); // Get triangulation indices

    if (mayaVertices.length() == 0 || triangleVertices.length() == 0) {
         MGlobal::displayWarning("MochiKernel::build: Mesh has no vertices or triangles.");
         return MStatus::kSuccess; // Nothing to build
    }

    // 3. Convert and Transform Data
    std::vector<Point_cu> hostVertices;
    hostVertices.reserve(mayaVertices.length());
    Transfo transform = MMatrixToTransfo(offsetMatrix);

    _hostOriginalVertices.reserve(mayaVertices.length()); // Store transformed vertices

    for (unsigned int i = 0; i < mayaVertices.length(); ++i) {
        Point_cu p(
            (float)mayaVertices[i].x,
            (float)mayaVertices[i].y,
            (float)mayaVertices[i].z
        );
        Point_cu transformed_p = transform * p; // Apply offset matrix
        hostVertices.push_back(transformed_p);
        _hostOriginalVertices.push_back(transformed_p); // Keep a copy
    }

    // Prepare triangle indices and face indices
    std::vector<int> hostIndices;
    hostIndices.reserve(triangleVertices.length());
    _hostOriginalFaceIndices.reserve(triangleVertices.length() / 3); // Store face index per triangle

    int currentVertexIndex = 0;
    for (unsigned int faceIdx = 0; faceIdx < triangleCounts.length(); ++faceIdx) { // Iterate through polygons (faces)
        int numTrianglesInFace = triangleCounts[faceIdx];
        for (int triInFaceIdx = 0; triInFaceIdx < numTrianglesInFace; ++triInFaceIdx) { // Iterate through triangles in the polygon

            int v0_idx = triangleVertices[currentVertexIndex];
            int v1_idx = triangleVertices[currentVertexIndex + 1];
            int v2_idx = triangleVertices[currentVertexIndex + 2];
            hostIndices.push_back(v0_idx);
            hostIndices.push_back(v1_idx);
            hostIndices.push_back(v2_idx);

            // Store the original face index for this triangle
            _hostOriginalFaceIndices.push_back(faceIdx);

            currentVertexIndex += 3;
        }
    }

    _hostOriginalIndices = hostIndices; // Keep a copy

    // 4. Call Bridging Function to Build OptiX BVH and initialize _optixData
    _optixData = mochiKernelBuild_impl(hostVertices, hostIndices, _hostOriginalFaceIndices);

    if (!_optixData) {
        MGlobal::displayError("MochiKernel::build: Failed to build OptiX BVH (mochiKernelBuild_impl returned null).");
        // Cleanup host copies if build failed? Maybe not necessary as they are std::vector
        return MStatus::kFailure;
    }

    return MStatus::kSuccess;
}

std::vector<TriangleData> MochiKernel::intersectKernelTriangle(const TriangleData& triangle) const {
    std::vector<TriangleData> result;
     if (!_optixData) {
         MGlobal::displayWarning("MochiKernel::intersectKernelTriangle: OptiX data not built.");
         return result;
     }
    MGlobal::displayWarning(MString("MochiKernel::intersectKernelTriangle: OptiX data built."));

    // 1. Convert input Maya TriangleData to POD
    MochiTrianglePOD trianglePOD;
    trianglePOD.vertices[0] = Point_cu((float)triangle.vertices[0].x, (float)triangle.vertices[0].y, (float)triangle.vertices[0].z);
    trianglePOD.vertices[1] = Point_cu((float)triangle.vertices[1].x, (float)triangle.vertices[1].y, (float)triangle.vertices[1].z);
    trianglePOD.vertices[2] = Point_cu((float)triangle.vertices[2].x, (float)triangle.vertices[2].y, (float)triangle.vertices[2].z);
    trianglePOD.faceIndex = triangle.faceIndex;
    trianglePOD.triangleIndex = triangle.triangleIndex; // Assuming TriangleData has this field

    // 2. Call bridging function
    std::vector<int> collidingIndices; // Indices of *original* triangles in the BVH
    mochiKernelIntersectTriangle_impl(_optixData, trianglePOD, collidingIndices);

    // 3. Convert results back to Maya TriangleData
    result.reserve(collidingIndices.size());
    for (int originalTriangleIndex : collidingIndices) {
       result.push_back(reconstructTriangleData(originalTriangleIndex));
    }

    return result;
}

K2KIntersection MochiKernel::intersectKernelKernel(SpatialDivisionKernel& otherKernel) const {
    MGlobal::displayWarning("MochiKernel::intersectKernelKernel: begin");
    K2KIntersection result; // Pair of std::vector<TriangleData>
     if (!_optixData) {
         MGlobal::displayWarning("MochiKernel::intersectKernelKernel: OptiX data not built for 'this'.");
         return result;
     }

    // 1. Check if otherKernel is also MochiKernel
    MochiKernel* otherMochiKernel = dynamic_cast<MochiKernel*>(&otherKernel);
    if (!otherMochiKernel) {
        MGlobal::displayWarning("MochiKernel::intersectKernelKernel: otherKernel is not a MochiKernel.");
        // Optionally, implement fallback or return empty
        return result;
    }
     if (!otherMochiKernel->_optixData) {
         MGlobal::displayWarning("MochiKernel::intersectKernelKernel: OptiX data not built for 'otherKernel'.");
         return result;
     }

    // 2. Call bridging function
    std::vector<MochiCollisionPairPOD> collisionPairsPOD;
    bool selfIntersection = (this == otherMochiKernel);
    mochiKernelIntersect_impl(_optixData, otherMochiKernel->_optixData, selfIntersection, collisionPairsPOD);
    MGlobal::displayInfo("MochiKernel::intersectKernelKernel: after mochiKernelIntersect_impl");

    // 3. Convert results back to K2KIntersection
    result.first.reserve(collisionPairsPOD.size());  // Collisions from kernel A's perspective
    result.second.reserve(collisionPairsPOD.size()); // Collisions from kernel B's perspective
    MGlobal::displayInfo("MochiKernel::intersectKernelKernel: after reserve");

    for (const auto& pairPOD : collisionPairsPOD) {
        // Reconstruct TriangleData for the triangle from kernel A (this)
        result.first.push_back(reconstructTriangleData(pairPOD.triangleIndexA));

        // Reconstruct TriangleData for the triangle from kernel B (other)
        result.second.push_back(otherMochiKernel->reconstructTriangleData(pairPOD.triangleIndexB));
    }

    return result;
}

// Helper to reconstruct Maya TriangleData from the stored host data
TriangleData MochiKernel::reconstructTriangleData(int originalTriangleIndex) const {
    if (originalTriangleIndex < 0 || originalTriangleIndex >= _hostOriginalIndices.size() / 3) {
        MGlobal::displayError("MochiKernel::reconstructTriangleData: Invalid triangle index.");
        return TriangleData(); // Return empty/default
    }

    TriangleData td;
    int baseVertexIndex = originalTriangleIndex * 3;
    int v0_idx = _hostOriginalIndices[baseVertexIndex + 0];
    int v1_idx = _hostOriginalIndices[baseVertexIndex + 1];
    int v2_idx = _hostOriginalIndices[baseVertexIndex + 2];

    if (v0_idx < 0 || v0_idx >= _hostOriginalVertices.size() ||
        v1_idx < 0 || v1_idx >= _hostOriginalVertices.size() ||
        v2_idx < 0 || v2_idx >= _hostOriginalVertices.size())
    {
         MGlobal::displayError("MochiKernel::reconstructTriangleData: Invalid vertex index lookup.");
         return TriangleData();
    }

    const Point_cu& p0 = _hostOriginalVertices[v0_idx];
    const Point_cu& p1 = _hostOriginalVertices[v1_idx];
    const Point_cu& p2 = _hostOriginalVertices[v2_idx];

    td.vertices[0] = MPoint(p0.x, p0.y, p0.z);
    td.vertices[1] = MPoint(p1.x, p1.y, p1.z);
    td.vertices[2] = MPoint(p2.x, p2.y, p2.z);

    td.faceIndex = _hostOriginalFaceIndices[originalTriangleIndex];
    td.triangleIndex = -1; // We don't easily know the sub-index within the original face here

    return td;
}
