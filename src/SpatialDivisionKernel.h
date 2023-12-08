#pragma once

#include "utility.h"

#include <vector>
#include <utility> // for std::pair

#include <maya/MDagPath.h>
#include <maya/MPoint.h>
#include <maya/MStatus.h>
#include <maya/MMatrix.h>

using K2KIntersection = std::pair<std::vector<TriangleData>, std::vector<TriangleData>>;
class SpatialDivisionKernel
{
public:
    // Virtual destructor to allow for subclassing
    virtual                           ~SpatialDivisionKernel() {};

    // Interface methods
    virtual                   MStatus build(const MObject& meshObject, const MBoundingBox& bbox, const MMatrix& offsetMatrix) = 0;
    virtual std::vector<TriangleData> intersectKernelTriangle(const TriangleData& triangle) const = 0;
    virtual           K2KIntersection intersectKernelKernel(SpatialDivisionKernel& otherKernel) const = 0;
};

