#include "EmbreeKernel.h"

#include <embree4/rtcore.h>
#include <embree4/rtcore_geometry.h>
#include <embree4/rtcore_scene.h>

#include <queue>
#include <vector>

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

MStatus EmbreeKernel::build(const MObject& meshObject, const MBoundingBox& bbox)
{
    MStatus status;

    rtcDevice = rtcNewDevice(nullptr);
    if(!rtcDevice)
    {
        MGlobal::displayError("Failed to create Embree device");
        return MStatus::kFailure;
    }

    rtcSetDeviceErrorFunction(rtcDevice, errorHandler, nullptr);

    // RTCScene scene = rtcNewScene(rtcDevice);

    rtcBVH = rtcNewBVH(rtcDevice);
    if (!rtcBVH) {
        MGlobal::displayError("Failed to create Embree BVH");
        return MStatus::kFailure;
    }

    // collect all triangles
    std::vector<RTCBuildPrimitive> primitives;
    MItMeshPolygon itPoly(meshObject);
    for(; !itPoly.isDone(); itPoly.next()) {

        int numTriangles;
        itPoly.numTriangles(numTriangles);

        for (int i=0; i < numTriangles; ++i) {
            MPointArray points;
            MIntArray vertexList;
            itPoly.getTriangle(i, points, vertexList, MSpace::kObject);

            RTCBuildPrimitive prim;
            prim.lower_x = static_cast<float>(points[0].x);
            prim.lower_y = static_cast<float>(points[0].y);
            prim.lower_z = static_cast<float>(points[0].z);
            prim.geomID = 0;
            prim.upper_x = static_cast<float>(points[2].x);
            prim.upper_y = static_cast<float>(points[2].y);
            prim.upper_z = static_cast<float>(points[2].z);
            prim.primID = i;
            primitives.push_back(prim);
        }
    }

    MGlobal::displayInfo(MString("Number of primitives: ") + std::to_wstring(primitives.size()).c_str());

    // Build BVH
    RTCBuildArguments arguments = rtcDefaultBuildArguments();
    arguments.byteSize               = sizeof(arguments);
    // arguments.buildFlags             = RTC_BUILD_FLAG_NONE;
    arguments.buildQuality           = RTC_BUILD_QUALITY_MEDIUM;
    // arguments.maxBranchingFactor     = 2;
    // arguments.maxDepth               = 1024;
    // arguments.sahBlockSize           = 1;
    // arguments.minLeafSize            = 1;
    // arguments.maxLeafSize            = 1;
    // arguments.traversalCost          = 1.0f;
    // arguments.intersectionCost       = 10.0f;
    arguments.bvh                    = rtcBVH;
    arguments.primitives             = primitives.data();
    arguments.primitiveCount         = primitives.size();
    arguments.primitiveArrayCapacity = primitives.size();
    // arguments.createNode             = InnerNode::create;
    // arguments.setNodeChildren        = InnerNode::setChildren;
    // arguments.setNodeBounds          = InnerNode::setBounds;
    // arguments.createLeaf             = AMRLeafNode::create;
    // arguments.splitPrimitive         = nullptr;
    // arguments.buildProgress          = nullptr;
    // arguments.userPtr                = userData.data();

    rtcRoot = (Node *)rtcBuildBVH(&arguments);
    if (!rtcRoot) {
        MGlobal::displayError("Failed to build Embree BVH");
        return MStatus::kFailure;
    }

    return MStatus::kNotImplemented;
}

std::vector<TriangleData> EmbreeKernel::queryIntersected(const TriangleData& triangle) const
{
    // TODO: Implement Embree querying
    return std::vector<TriangleData>();
}
