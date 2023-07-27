#pragma once

#include "SpatialDivisionKernel.h"

#include <maya/MPoint.h>
#include <maya/MStatus.h>

#include <embree4/rtcore.h>
#include <embree4/rtcore_geometry.h>
#include <embree4/rtcore_scene.h>

#include <cstdint>

class EmbreeKernel : public SpatialDivisionKernel
{
public:
    ~EmbreeKernel() override {};

    MStatus build(const MObject& meshObject, const MBoundingBox& bbox) override;
    std::vector<TriangleData> queryIntersected(const TriangleData& triangle) const override;


    struct Node
    {
        inline bool isLeaf() const
        {
            return dim == 3;
        }
        // first dword
        uint32_t ofs : 30;  // offset in node[] array (if inner), or brick ID
                            // (if leaf)
        uint32_t dim : 2;   // upper two bits: split dimension. '3' means 'leaf
                            // second dword
        union
        {
            float pos;
            uint32_t numItems;
        };
    };

private:
    RTCBVH rtcBVH;
    RTCDevice rtcDevice;
    Node *rtcRoot;
    int bvhDepth;
};
