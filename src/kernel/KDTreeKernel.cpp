#include "KDTreeKernel.h"

#include <maya/MStatus.h>


MStatus KDTreeKernel::build(const MObject& meshObject, const MBoundingBox& bbox)
{
    // TODO: Implement KDTree building
    return MStatus::kNotImplemented;
}

std::vector<TriangleData> KDTreeKernel::queryIntersected(const TriangleData& triangle) const
{
    // TODO: Implement KDTree querying
    return std::vector<TriangleData>();
}
