#include "EmbreeKernel.h"
#include "../utility.h"

#include <glm/glm.hpp>
#include <embree4/rtcore.h>
#include <embree4/rtcore_geometry.h>
#include <embree4/rtcore_scene.h>

#include <maya/MStatus.h>
#include <maya/MMatrix.h>
#include <maya/MFnMesh.h>
#include <maya/MFnDagNode.h>
#include <maya/MItMeshPolygon.h>
#include <maya/MBoundingBox.h>
#include <maya/MPoint.h>
#include <maya/MPointArray.h>
#include <maya/MIntArray.h>
#include <maya/MGlobal.h>
#include <maya/MStatus.h>

#include <stack>
#include <queue>
#include <vector>
#include <cstdint>
#include <functional>
#include <cassert>


/* This function is called by the builder to signal progress and to
 * report memory consumption. */
bool memoryMonitor(void* userPtr, ssize_t bytes, bool post) {
    return true;
}


bool buildProgress (void* userPtr, double f) {
    return true;
}


void splitPrimitive (const RTCBuildPrimitive* prim, unsigned int dim, float pos, RTCBounds* lprim, RTCBounds* rprim, void* userPtr)
{
    assert(dim < 3);
    assert(prim->geomID == 0);
    *(MBoundingBox*) lprim = *(MBoundingBox*) prim;
    *(MBoundingBox*) rprim = *(MBoundingBox*) prim;
    (&lprim->upper_x)[dim] = pos;
    (&rprim->lower_x)[dim] = pos;
}


void errorHandler(void* userPtr, enum RTCError code, const char* str)
{
    printf("Embree Error: ");
    MGlobal::displayError("Embree Error: ");
    switch (code) {
        case RTC_ERROR_NONE:
            MGlobal::displayError("No error has been recorded. str parameter is set to nullptr.\n");
            break;
        case RTC_ERROR_UNKNOWN:
            MGlobal::displayError("An unknown error has occurred. Typically, this error will occur when an error has been generated in the Embree kernel that did not correspond to any of the other error codes.\n");
            break;
        case RTC_ERROR_INVALID_ARGUMENT:
            MGlobal::displayError("An invalid argument was passed.\n");
            break;
        case RTC_ERROR_INVALID_OPERATION:
            MGlobal::displayError("The operation is not allowed for the current state.\n");
            break;
        case RTC_ERROR_OUT_OF_MEMORY:
            MGlobal::displayError("There is not enough memory left to complete the operation.\n");
            break;
        case RTC_ERROR_UNSUPPORTED_CPU:
            MGlobal::displayError("The user tried to run the library on an unsupported CPU. The CPU must support at least SSE2.\n");
            break;
        case RTC_ERROR_CANCELLED:
            MGlobal::displayError("The operation was cancelled by the user.\n");
            break;
    }
    if (str) {
        MGlobal::displayError(str);
    }
}

MStatus EmbreeKernel::build(const MObject& meshObject, const MBoundingBox& bbox, const MMatrix& offsetMatrix)
{
    MStatus status;

    this->device = rtcNewDevice(nullptr);
    if(!this->device)
    {
        MGlobal::displayError("Failed to create Embree device");
        return MStatus::kFailure;
    }

    rtcSetDeviceErrorFunction(this->device, errorHandler, nullptr);

    this->bvh = rtcNewBVH(this->device);
    if (!this->bvh) {
        MGlobal::displayError("Failed to create Embree BVH");
        return MStatus::kFailure;
    }

    // store the PrimID to face id and triangle id mapping
    this->triangles.clear();

    // collect all triangles
    std::vector<RTCBuildPrimitive> primitives;
    MItMeshPolygon itPoly(meshObject);
    int primId = 0;
    for(; !itPoly.isDone(); itPoly.next()) {

        int numTriangles;
        itPoly.numTriangles(numTriangles);

        for (int triangleId=0; triangleId < numTriangles; ++triangleId) {
            MPointArray points;
            MIntArray vertexList;
            itPoly.getTriangle(triangleId, points, vertexList, MSpace::kObject);
            TriangleData triangle(
                    itPoly.index(),
                    triangleId,
                    points[0] * offsetMatrix,
                    points[1] * offsetMatrix,
                    points[2] * offsetMatrix);

            RTCBuildPrimitive prim;
            prim.lower_x = (float)triangle.bbox.min().x;
            prim.lower_y = (float)triangle.bbox.min().y;
            prim.lower_z = (float)triangle.bbox.min().z;
            prim.geomID = 0;
            prim.upper_x = (float)triangle.bbox.max().x;
            prim.upper_y = (float)triangle.bbox.max().y;
            prim.upper_z = (float)triangle.bbox.max().z;
            prim.primID = primId;
            primitives.push_back(prim);

            this->triangles.push_back(triangle);

            primId++;
        }
    }

    // Build BVH
    RTCBuildArguments arguments = rtcDefaultBuildArguments();
    arguments.byteSize               = sizeof(arguments);
    // arguments.buildFlags             = RTC_BUILD_FLAG_NONE;
    arguments.buildFlags             = RTC_BUILD_FLAG_DYNAMIC;
    arguments.buildQuality           = RTC_BUILD_QUALITY_MEDIUM;
    arguments.maxBranchingFactor     = 2;
    arguments.maxDepth               = 1024;
    arguments.sahBlockSize           = 1;
    arguments.minLeafSize            = 1;
    arguments.maxLeafSize            = 8;
    arguments.traversalCost          = 1.0f;
    arguments.intersectionCost       = 1.0f;
    arguments.bvh                    = this->bvh;
    arguments.primitives             = primitives.data();
    arguments.primitiveCount         = primitives.size();
    arguments.primitiveArrayCapacity = primitives.capacity();
    arguments.createNode             = InnerNode::create;
    arguments.setNodeChildren        = InnerNode::setChildren;
    arguments.setNodeBounds          = InnerNode::setBounds;
    arguments.createLeaf             = LeafNode::create;
    arguments.splitPrimitive         = splitPrimitive;
    arguments.buildProgress          = buildProgress;
    arguments.userPtr                = nullptr;
    // arguments.userPtr                = userData.data();

    root = (Node *)rtcBuildBVH(&arguments);
    if (!root) {
        MGlobal::displayError("Failed to build Embree BVH");
        return MStatus::kFailure;
    }

    return MStatus::kSuccess;
}



std::vector<TriangleData> EmbreeKernel::queryIntersected(const TriangleData& triangle) const
{
    std::vector<TriangleData> intersectingA;

    std::stack<Node*> stack;
    stack.push(root);

    while (!stack.empty()) {

        Node* currentNode = stack.top();
        stack.pop();

        bool isAInner = currentNode->branch() != nullptr;
        bool  isALeaf = currentNode->leaf() != nullptr;

        if (isAInner) {

            MBoundingBox bboxL = currentNode->branch()->bounds[0];
            MBoundingBox bboxR = currentNode->branch()->bounds[1];

            if (intersectBoxBox(bboxL, triangle.bbox)) {
                stack.push(currentNode->branch()->children[0]);
            }
            if (intersectBoxBox(bboxR, triangle.bbox)) {
                stack.push(currentNode->branch()->children[1]);
            }
        }

        if (isALeaf) {

            // FIXME:
            // Ideally, we should use box-to-box checks for collisions. However,
            // due to the occurrence of extremely small boxes, collisions
            // might be missed. Therefore, we're opting to skip the check.
            // if (!intersectBoxBox(currentNode->leaf()->bounds, triangle.bbox)) {

            int index = currentNode->leaf()->id;
            TriangleData triangleData = this->triangles[index];
            if (intersectTriangleTriangle(triangle, triangleData)) {
                intersectingA.push_back(triangleData);
            }

            if (index > 0) {
                triangleData = this->triangles[index-1];
                if (intersectTriangleTriangle(triangle, triangleData)) {
                    intersectingA.push_back(triangleData);
                }
            }
            if (index < this->triangles.size()-1) {
                triangleData = this->triangles[index+1];
                if (intersectTriangleTriangle(triangle, triangleData)) {
                    intersectingA.push_back(triangleData);
                }
            }
        }

    };

    return intersectingA;
}


void intersectBvhNodesRecursive(
        Node* nodeA,
        Node* nodeB,
        std::vector<std::pair<Node*, Node*>>& intersectedNodes
) {
    if (!nodeA || !nodeB) {
        return;
    }

    bool isAInner = nodeA->branch() != nullptr;
    bool isBInner = nodeB->branch() != nullptr;
    bool isALeaf = nodeA->leaf() != nullptr;
    bool isBLeaf = nodeB->leaf() != nullptr;

    if (nodeA->isLeaf() && nodeB->isLeaf()) {
        intersectedNodes.push_back({nodeA, nodeB});
        return;
    }

    if (nodeA->isLeaf()) {  // A is leaf, B is inner
        if (intersectBoxBox(nodeA->leaf()->bounds, nodeB->branch()->bounds[0])) {
            intersectBvhNodesRecursive(nodeA, nodeB->branch()->children[0], intersectedNodes);
        }

        if (intersectBoxBox(nodeA->leaf()->bounds, nodeB->branch()->bounds[1])) {
            intersectBvhNodesRecursive(nodeA, nodeB->branch()->children[1], intersectedNodes);
        }
        return;
    }

    if (nodeB->isLeaf()) {  // A is inner, B is leaf
        if (intersectBoxBox(nodeB->leaf()->bounds, nodeA->branch()->bounds[0])) {
            intersectBvhNodesRecursive(nodeA->branch()->children[0], nodeB, intersectedNodes);
        }

        if (intersectBoxBox(nodeB->leaf()->bounds, nodeA->branch()->bounds[1])) {
            intersectBvhNodesRecursive(nodeA->branch()->children[1], nodeB, intersectedNodes);
        }
        return;
    }

    // Both are inner nodes
    if (intersectBoxBox(nodeA->branch()->bounds[0], nodeB->branch()->bounds[0])) {
        intersectBvhNodesRecursive(nodeA->branch()->children[0], nodeB->branch()->children[0], intersectedNodes);
    }

    if (intersectBoxBox(nodeA->branch()->bounds[0], nodeB->branch()->bounds[1])) {
        intersectBvhNodesRecursive(nodeA->branch()->children[0], nodeB->branch()->children[1], intersectedNodes);
    }

    if (intersectBoxBox(nodeA->branch()->bounds[1], nodeB->branch()->bounds[0])) {
        intersectBvhNodesRecursive(nodeA->branch()->children[1], nodeB->branch()->children[0], intersectedNodes);
    }

    if (intersectBoxBox(nodeA->branch()->bounds[1], nodeB->branch()->bounds[1])) {
        intersectBvhNodesRecursive(nodeA->branch()->children[1], nodeB->branch()->children[1], intersectedNodes);
    }
}


K2KIntersection EmbreeKernel::intersectKernelKernel(

    SpatialDivisionKernel& otherKernel

) const {
    // MGlobal::displayInfo(MString("Intersecting EmbreeKernel with "));

    std::vector<TriangleData> intersectedTrianglesA;
    std::vector<TriangleData> intersectedTrianglesB;

    EmbreeKernel* other = dynamic_cast<EmbreeKernel*>(&otherKernel);
    if (!other) {
        // MGlobal::displayError("Failed to cast SpatialDivisionKernel to EmbreeKernel");
        return std::make_pair(intersectedTrianglesA, intersectedTrianglesB);
    }

    std::vector<std::pair<Node*, Node*>> intersectedNodes;
    intersectBvhNodesRecursive(this->root, other->root, intersectedNodes);

    for (const auto& pair : intersectedNodes) {
        Node* nodeA = pair.first;
        Node* nodeB = pair.second;

        if (nodeA->isLeaf() && nodeB->isLeaf()) {
            for (TriangleData triA : nodeA->triangles) {
                for (TriangleData triB : nodeB->triangles) {
                    if (intersectTriangleTriangle(triA, triB)) {
                        intersectedTrianglesA.push_back(triA);
                        intersectedTrianglesB.push_back(triB);
                    }
                }
            }
        }
    }

    return std::make_pair(intersectedTrianglesA, intersectedTrianglesB);
}
