#pragma once

#include "../SpatialDivisionKernel.h"

#include <maya/MPoint.h>
#include <maya/MStatus.h>
#include <maya/MMatrix.h>

class KDTreeKernel : public SpatialDivisionKernel
{
public:
    ~KDTreeKernel() override {};

    MStatus build(const MObject& meshObject, const MBoundingBox& bbox, const MMatrix& offsetMatrix) override;
    std::vector<TriangleData> queryIntersected(const TriangleData& triangle) const override;
};
