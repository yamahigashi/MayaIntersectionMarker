#pragma once

#include "SpatialDivisionKernel.h"

#include <maya/MDagPath.h>
#include <maya/MBoundingBox.h>
#include <maya/MPoint.h>
#include <maya/MStatus.h>


struct OctreeNode {
    MBoundingBox boundingBox;
    std::vector<TriangleData> triangles;
    OctreeNode* children[8] = { nullptr };

    bool isLeaf() const
    {
        for (int i = 0; i < 8; ++i) {
            if (children[i] != nullptr) {
                return false;
            }
        }
        return true;
    }
};


class OctreeKernel : public SpatialDivisionKernel
{
public:
    // Constructor
    OctreeKernel() : root(nullptr) {}

    // Destructor
    ~OctreeKernel() {
        if (root != nullptr) {
            clear(root);
        }
    }

    MStatus build(const MObject& meshObject, const MBoundingBox& bbox) override;
    std::vector<TriangleData> queryIntersected(const TriangleData& triangle) const override;

private:
    OctreeNode* root;

    void insertTriangle(OctreeNode* node, const TriangleData& triangle);
    void clear(OctreeNode* node);
    bool OctreeKernel::boxTriangleIntersect(const MBoundingBox& box, const TriangleData& triangle) const;
    void OctreeKernel::splitNode(OctreeNode* node);
};
