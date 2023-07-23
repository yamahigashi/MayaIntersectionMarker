#pragma once

#include <vector>
#include <maya/MDagPath.h>
#include <maya/MPoint.h>
#include <maya/MStatus.h>

struct TriangleData {
    int faceIndex;
    MPoint vertices[3];
};


class SpatialDivisionKernel
{
public:
    // Virtual destructor to allow for subclassing
    virtual ~SpatialDivisionKernel() {};

    // Interface methods
    virtual MStatus build(const MObject& meshObject, const MBoundingBox& bbox) = 0;
    virtual std::vector<TriangleData> queryIntersected(const TriangleData& triangle) const = 0;
};
