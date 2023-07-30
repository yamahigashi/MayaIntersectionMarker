#include "EmbreeKernel.h"

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


#include <queue>
#include <vector>

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

    // RTCScene scene = rtcNewScene(rtcDevice);

    this->bvh = rtcNewBVH(this->device);
    if (!this->bvh) {
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
            MBoundingBox bbox;
            bbox.expand(points[0]);
            bbox.expand(points[1]);
            bbox.expand(points[2]);

            RTCBuildPrimitive prim;
            prim.lower_x = bbox.min().x;
            prim.lower_y = bbox.min().y;
            prim.lower_z = bbox.min().z;
            prim.geomID = 0;
            prim.upper_x = bbox.max().x;
            prim.upper_y = bbox.max().y;
            prim.upper_z = bbox.max().z;
            prim.primID = i;
            primitives.push_back(prim);
        }
    }

    MGlobal::displayInfo(MString("Number of primitives: ") + std::to_wstring(primitives.size()).c_str());

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

    // for (size_t i=0; i<10; i++)
    // {
    //     /* we recreate the prims array here, as the builders modify this array */
    //     for (size_t j=0; j<prims.size(); j++) prims[j] = prims_i[j];
    // 
    //     std::cout << "iteration " << i << ": building BVH over " << prims.size() << " primitives, " << std::flush;
    //     double t0 = getSeconds();
    //     Node* root = (Node*) rtcBuildBVH(&arguments);
    //     double t1 = getSeconds();
    //     const float sah = root ? root->sah() : 0.0f;
    //     std::cout << 1000.0f*(t1-t0) << "ms, " << 1E-6*double(prims.size())/(t1-t0) << " Mprims/s, sah = " << sah << " [DONE]" << std::endl;
    // }
    // 
    // rtcReleaseBVH(bvh);

    return MStatus::kSuccess;
}




std::vector<TriangleData> EmbreeKernel::queryIntersected(const TriangleData& triangle) const
{
    // TODO: Implement Embree querying
    return std::vector<TriangleData>();
}

// bool EmbreeKernel::intersectTriangleTriangle(unsigned geomID0, unsigned primID0, unsigned geomID1, unsigned primID1)
// {
//   //CSTAT(bvh_collide_prim_intersections1++);
// 
//   /* special culling for scene intersection with itself */
//   if (geomID0 == geomID1 && primID0 == primID1) {
//     return false;
//   }
//   //CSTAT(bvh_collide_prim_intersections2++);
// 
//   auto mesh0 = meshes[geomID0].get ();
//   auto mesh1 = meshes[geomID1].get ();
//   auto const & tri0 = (Triangle&) mesh0->tris_[primID0];
//   auto const & tri1 = (Triangle&) mesh1->tris_[primID1];
// 
//   if (geomID0 == geomID1)
//   {
//     /* ignore intersection with topological neighbors */
//     const vint4 t0(tri0.v0,tri0.v1,tri0.v2,tri0.v2);
//     if (any(vint4(tri1.v0) == t0)) return false;
//     if (any(vint4(tri1.v1) == t0)) return false;
//     if (any(vint4(tri1.v2) == t0)) return false;
//   }
//   //CSTAT(bvh_collide_prim_intersections3++);
// 
//   const Vec3fa a0 = mesh0->x_[tri0.v0];
//   const Vec3fa a1 = mesh0->x_[tri0.v1];
//   const Vec3fa a2 = mesh0->x_[tri0.v2];
//   const Vec3fa b0 = mesh1->x_[tri1.v0];
//   const Vec3fa b1 = mesh1->x_[tri1.v1];
//   const Vec3fa b2 = mesh1->x_[tri1.v2];
// 
//   return isa::TriangleTriangleIntersector::intersect_triangle_triangle(a0,a1,a2,b0,b1,b2);
//   return false;
// }
