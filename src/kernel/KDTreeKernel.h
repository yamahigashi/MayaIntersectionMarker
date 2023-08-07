#pragma once
#include "../utility.h"
#include "../SpatialDivisionKernel.h"

#include <maya/MPoint.h>
#include <maya/MStatus.h>
#include <maya/MMatrix.h>


struct KDTreeNode {
                 MBoundingBox   boundingBox;
    std::vector<TriangleData>   triangles;
                   KDTreeNode*  left = nullptr;
                   KDTreeNode*  right = nullptr;
                          int   splitDimension = 0; // 0 for x, 1 for y, 2 for z
                       double   splitValue = 0.0;

    bool isLeaf() const {
        return left == nullptr && right == nullptr;
    }
};



class KDTreeKernel : public SpatialDivisionKernel
{
public:
     KDTreeKernel() : root(nullptr) {}
    ~KDTreeKernel() override {
        if (root) {
            clear(root);
        }
    }

    MStatus build(const MObject& meshObject, const MBoundingBox& bbox, const MMatrix& offsetMatrix) override;
    std::vector<TriangleData> intersectKernelTriangle(const TriangleData& triangle) const override;
    K2KIntersection intersectKernelKernel(SpatialDivisionKernel& otherKernel) const override { return K2KIntersection(); }

private:
    KDTreeNode* root;
    const int maxTrianglesPerLeaf = 4;

    void insertTriangle(KDTreeNode* node, const TriangleData& triangle);
    void splitNode(KDTreeNode* node);
    void clear(KDTreeNode* node);
    void setChildBoundingBoxes(KDTreeNode* node);
std::vector<TriangleData> extractTriangles(const MObject& meshObject, const MMatrix& offsetMatrix) const;
};
