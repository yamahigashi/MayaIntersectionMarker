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

struct InnerNode;
struct LeafNode;

using TriangleList = std::vector<TriangleData>;
struct Node
{
    virtual  InnerNode* branch() { return nullptr; }
    virtual   LeafNode* leaf()   { return nullptr; }

             InnerNode* parent() { return nullptr; }
           TriangleList triangles;
};


struct NodePair
{
    Node* nodeA;
    Node* nodeB;
};


struct InnerNode : public Node
{
    MBoundingBox bounds[2];
    Node* children[2];

    InnerNode() {
        bounds[0] = bounds[1] = MBoundingBox();
        children[0] = children[1] = nullptr;
    }

    static void* create (RTCThreadLocalAllocator alloc, unsigned int numChildren, void* userPtr)
    {
        assert(numChildren == 2);
        void* ptr = rtcThreadLocalAlloc(alloc, sizeof(InnerNode), 16);

        Node *node = new (ptr) InnerNode;
        return (void *)node;
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
            MBoundingBox box(
                MPoint(bounds[i]->lower_x, bounds[i]->lower_y, bounds[i]->lower_z),
                MPoint(bounds[i]->upper_x, bounds[i]->upper_y, bounds[i]->upper_z)
            );
            ((InnerNode*)nodePtr)->bounds[i] = box;
        }
    }

    InnerNode *branch() { return this; }
};

struct LeafNode : public Node
{
    unsigned id;
    MBoundingBox bounds;

    LeafNode (unsigned id, const MBoundingBox& bounds)
        : id(id), bounds(bounds) {}

    static void* create (RTCThreadLocalAllocator alloc, const RTCBuildPrimitive* prims, size_t numPrims, void* userPtr)
    {
        assert(numPrims == 1);
        void* ptr = rtcThreadLocalAlloc(alloc,sizeof(LeafNode),16);
        // return (void*) new (ptr) LeafNode(prims->primID,*(MBoundingBox*)prims);

        Node *node = new (ptr) LeafNode(prims->primID, *(MBoundingBox*)prims);
        return (void *)node;
    }

    LeafNode *leaf() { return this; }
};


using IDMapper = std::vector<std::pair<int, int>>;
using TriangleStorage = std::vector<TriangleData>;
class EmbreeKernel : public SpatialDivisionKernel
{
public:
    ~EmbreeKernel() override
    {
        if (this->bvh) {
            rtcReleaseBVH(this->bvh);
        }
        if (this->device) {
            rtcReleaseDevice(this->device);
        }
    }


                      MStatus build(const MObject& meshObject, const MBoundingBox& bbox, const MMatrix& offsetMatrix) override;
    std::vector<TriangleData> queryIntersected(const TriangleData& triangle) const override;
              K2KIntersection intersectKernelKernel(SpatialDivisionKernel& otherKernel) const;

private:
         RTCBVH bvh;
      RTCDevice device;
           Node *root;
            int bvhDepth;
TriangleStorage triangles;

};
