#pragma once

#include "../SpatialDivisionKernel.h"
#include "../utility.h"

#include <maya/MPoint.h>
#include <maya/MStatus.h>
#include <maya/MMatrix.h>
#include <maya/MBoundingBox.h>

#include <embree4/rtcore.h>
#include <embree4/rtcore_geometry.h>
#include <embree4/rtcore_scene.h>

#include <cassert>
#include <cstdint>

class EmbreeKernel : public SpatialDivisionKernel
{
public:
    ~EmbreeKernel() override {};

    MStatus build(const MObject& meshObject, const MBoundingBox& bbox, const MMatrix& offsetMatrix) override;
    std::vector<TriangleData> queryIntersected(const TriangleData& triangle) const override;


    struct Node
    {
        virtual float sah() = 0;
    };

    struct InnerNode : public Node
    {
        MBoundingBox bounds[2];
        Node* children[2];

        InnerNode() {
            bounds[0] = bounds[1] = MBoundingBox();
            children[0] = children[1] = nullptr;
        }

        float sah() {
            const float area0 = (float)area(bounds[0]);
            const float area1 = (float)area(bounds[1]);
            const float mergeArea = (float)area(merge(bounds[0], bounds[1]));

            return 1.0f + (area0 * children[0]->sah() + area1 * children[1]->sah()) / mergeArea;
        }

        static void* create (RTCThreadLocalAllocator alloc, unsigned int numChildren, void* userPtr)
        {
            assert(numChildren == 2);
            void* ptr = rtcThreadLocalAlloc(alloc,sizeof(InnerNode),16);
            return (void*) new (ptr) InnerNode;
        }

        static void  setChildren (void* nodePtr, void** childPtr, unsigned int numChildren, void* userPtr)
        {
            assert(numChildren == 2);
            for (size_t i=0; i<2; i++) {
                ((InnerNode*)nodePtr)->children[i] = (Node*) childPtr[i];
            }
        }

        static void  setBounds (void* nodePtr, const RTCBounds** bounds, unsigned int numChildren, void* userPtr)
        {
            assert(numChildren == 2);
            for (size_t i=0; i<2; i++) {
                ((InnerNode*)nodePtr)->bounds[i] = *(const MBoundingBox*) bounds[i];
            }
        }
    };

    struct LeafNode : public Node
    {
        unsigned id;
        MBoundingBox bounds;

        LeafNode (unsigned id, const MBoundingBox& bounds)
            : id(id), bounds(bounds) {}

        float sah() {
            return 1.0f;
        }

        static void* create (RTCThreadLocalAllocator alloc, const RTCBuildPrimitive* prims, size_t numPrims, void* userPtr)
        {
            assert(numPrims == 1);
            void* ptr = rtcThreadLocalAlloc(alloc,sizeof(LeafNode),16);
            return (void*) new (ptr) LeafNode(prims->primID,*(MBoundingBox*)prims);
        }
    };

private:
    RTCBVH bvh;
    RTCDevice device;
    Node *root;
    int bvhDepth;

};
