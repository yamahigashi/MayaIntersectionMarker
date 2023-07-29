#pragma once

#include "utility.h"

#include <vector>
#include <maya/MDagPath.h>
#include <maya/MPoint.h>
#include <maya/MStatus.h>
#include <maya/MMatrix.h>


class SpatialDivisionKernel
{
public:
    // Virtual destructor to allow for subclassing
    virtual ~SpatialDivisionKernel() {};

    // Interface methods
    virtual MStatus build(const MObject& meshObject, const MBoundingBox& bbox, const MMatrix& offsetMatrix) = 0;
    virtual std::vector<TriangleData> queryIntersected(const TriangleData& triangle) const = 0;
};
