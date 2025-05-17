#include <optix.h>
#include <optix_device.h> // For device-side functions
#include <cuda_runtime.h>

// Include shared headers directly (assuming correct include paths during compilation)
#include "../cuda_utils/MochiKernelPODs.h"
#include "../cuda_utils/vec3_cu.hpp"
#include "../cuda_utils/point_cu.hpp"

// --- Constants ---
#define INTERSECTION_EPSILON 1e-5f // Tolerance for point-on-segment check

// --- Launch Parameters ---
// The C++ side defines this struct and copies data to constant memory symbol "params"
// The name "params" must match OptixPipelineCompileOptions::pipelineLaunchParamsVariableName
struct Params {
    OptixTraversableHandle targetGas;             // BVH of the mesh being hit
    MochiCollisionPairPOD* collisionBuffer;       // Output buffer for collision pairs
    unsigned int*          collisionCounter;      // Atomic counter for number of collisions
    Point_cu*              meshAVertices;         // Vertex buffer of the mesh shooting rays
    int3*                  meshAIndices;          // Original triangle indices of the mesh shooting rays
    unsigned int           meshANumOriginalTriangles; // Number of original triangles in the mesh shooting rays
    int*                   meshAOriginalFaceIndices; // Optional: Face indices for mesh A
    bool                   selfIntersectionCheck;   // Flag to enable self-hit checks
    unsigned int           maxCollisions;           // Size of collisionBuffer
};

extern "C" { __constant__ Params params; }

// --- SBT Data Structures ---
struct HitGroupData {
    alignas(OPTIX_SBT_RECORD_ALIGNMENT) char header[OPTIX_SBT_RECORD_HEADER_SIZE];
    enum GeoType { ORIGINAL, AUXILIARY };
    GeoType type;
    Point_cu* vertices;
    int3*     indices; // Original or Aux indices depending on type
    int*      originalFaceIndices; // Only valid for ORIGINAL type
    unsigned int numOriginalTriangles; // Needed by Aux hit to map back
};

// ---------------------------------------------------------------------------
// Helper Functions (Device-side)
// ---------------------------------------------------------------------------

// Checks if point P lies on the line segment defined by vA and vB (within epsilon)
// Considers projection and distance check.
__device__ __forceinline__ bool isPointOnSegment(const Point_cu& P, const Point_cu& vA, const Point_cu& vB) {
    Vec3_cu segVec = vB - vA;
    Vec3_cu pVec = P - vA;
    float segLenSq = segVec.norm_squared();

    // Handle zero-length segment case
    if (segLenSq < INTERSECTION_EPSILON * INTERSECTION_EPSILON) {
        return pVec.norm_squared() < INTERSECTION_EPSILON * INTERSECTION_EPSILON;
    }

    // Project pVec onto segVec (calculate t parameter)
    // t = dot(pVec, segVec)
    // We need t normalized by segment length squared to check bounds [0, 1]
    float t_num = pVec.dot(segVec);

    // Check if projection is within the segment bounds [0, segLenSq] for numerical stability
    if (t_num < -INTERSECTION_EPSILON || t_num > segLenSq + INTERSECTION_EPSILON) {
        return false; // Projection lies outside the segment endpoints
    }

    // Check if the point P is actually close to the line defined by the segment.
    // Calculate the projection point: Proj = vA + segVec * (t_num / segLenSq)
    // Calculate squared distance from P to the projection point.
    Vec3_cu projection_vec = segVec * (t_num / segLenSq);
    float distSq = (pVec - projection_vec).norm_squared();

    // Allow a small deviation from the line
    return distSq < (INTERSECTION_EPSILON * INTERSECTION_EPSILON);
}


// --- OptiX Programs ---

extern "C" __global__ void __raygen__shootEdgeRays()
{
    // Get launch indices: x = source triangle index, y = edge index (0, 1, 2)
    const uint3 launchIdx = optixGetLaunchIndex();
    const unsigned int sourceTriIdx = launchIdx.x;
    const unsigned int edgeIdx = launchIdx.y;

    // Bounds check for safety
    if (sourceTriIdx >= params.meshANumOriginalTriangles) {
        return;
    }

    // Fetch source triangle vertices (Mesh A)
    const int3 vIdxA = params.meshAIndices[sourceTriIdx];
    const Point_cu vA0 = params.meshAVertices[vIdxA.x];
    const Point_cu vA1 = params.meshAVertices[vIdxA.y];
    const Point_cu vA2 = params.meshAVertices[vIdxA.z];

    // Determine ray properties based on edge index
    Point_cu rayOrigin;
    Vec3_cu rayDirection;
    float rayTmax;

    switch (edgeIdx) {
        case 0: // Edge vA0 -> vA1
            rayOrigin = vA0;
            rayDirection = vA1 - vA0;
            break;
        case 1: // Edge vA1 -> vA2
            rayOrigin = vA1;
            rayDirection = vA2 - vA1;
            break;
        case 2: // Edge vA2 -> vA0
            rayOrigin = vA2;
            rayDirection = vA0 - vA2;
            break;
        default:
            return; // Should not happen
    }

    rayTmax = rayDirection.norm();

    // Avoid tracing zero-length or tiny rays, normalize direction
    if (rayTmax < INTERSECTION_EPSILON) {
        return;
    }
    rayDirection = rayDirection * (1.0f / rayTmax);

    // Payload: Pass the source triangle index (Mesh A)
    unsigned int payload = sourceTriIdx;

    // Trace the ray
    optixTrace(
        params.targetGas,                             // Handle of the GAS to trace against (Mesh B)
        make_float3(rayOrigin.x, rayOrigin.y, rayOrigin.z), // Ray origin
        make_float3(rayDirection.x, rayDirection.y, rayDirection.z), // Ray direction
        0.0f,                                         // tmin
        rayTmax - INTERSECTION_EPSILON,               // tmax (subtract epsilon to avoid self-intersection at end point)
        0.0f,                                         // rayTime
        OptixVisibilityMask(1),                       // Visibility mask
        OPTIX_RAY_FLAG_NONE,                          // Flags - Important: No TERMINATE_ON_FIRST_HIT
        0,                                            // SBT offset for this ray type
        2,                                            // SBT stride (Original=0, Aux=1)
        0,                                            // Miss SBT Index
        payload                                       // Pass source triangle index in payload[0]
    );
}

extern "C" __global__ void __anyhit__processHit()
{
    // Get SBT data pointer for the hit geometry (Mesh B)
    const HitGroupData* sbtData = reinterpret_cast<const HitGroupData*>(optixGetSbtDataPointer());

    // Get primitive index within the target GAS (Mesh B)
    const unsigned int hitPrimIdx = optixGetPrimitiveIndex();

    // Get source triangle index (Mesh A) from payload
    const unsigned int rayOriginTriIdxA = optixGetPayload_0();

    unsigned int hitOrigTriIdxB = 0;       // Index of the original triangle in Mesh B this hit corresponds to
    int hitFaceIndexA = -1; // Placeholder, needs data passed
    int hitFaceIndexB = -1;

    // --- Determine hit type and corresponding original triangle index in Mesh B ---
    if (sbtData->type == HitGroupData::ORIGINAL) {
        hitOrigTriIdxB = hitPrimIdx;

        // Optional: Get face index if buffer provided
        if (sbtData->originalFaceIndices) {
            hitFaceIndexB = sbtData->originalFaceIndices[hitOrigTriIdxB];
        }

        // Self-intersection check
        if (params.selfIntersectionCheck && rayOriginTriIdxA == hitOrigTriIdxB) {
            return; // Ignore self-hit
        }

        // --- Collision Confirmed (Original Hit) ---
        // Atomically get the next available slot in the buffer
        unsigned int writeIndex = atomicAdd(params.collisionCounter, 1);
        if (writeIndex < params.maxCollisions) {
             // Need face index for mesh A
             if (params.meshAOriginalFaceIndices){
                 // Assuming params.meshAOriginalFaceIndices points to device memory
                 // Need bounds check if this pointer could be null
                 // hitFaceIndexA = params.meshAOriginalFaceIndices[rayOriginTriIdxA];
             }

            params.collisionBuffer[writeIndex].triangleIndexA = rayOriginTriIdxA;
            params.collisionBuffer[writeIndex].faceIndexA = hitFaceIndexA; // Placeholder
            params.collisionBuffer[writeIndex].triangleIndexB = hitOrigTriIdxB;
            params.collisionBuffer[writeIndex].faceIndexB = hitFaceIndexB;
        } else {
            // Buffer overflow - optionally report or handle
            // atomicSub(params.collisionCounter, 1); // careful with this
             printf("Collision buffer overflow!\n");
        }

    } else if (sbtData->type == HitGroupData::AUXILIARY) {
        // --- Hit an Auxiliary Triangle ---
        const unsigned int numOriginalTrisB = sbtData->numOriginalTriangles;
        if (hitPrimIdx < numOriginalTrisB) {
             // This indicates an SBT setup error
             printf("AH Error: Hit Aux SBT but PrimIdx %u < numOrig %u\n", hitPrimIdx, numOriginalTrisB);
             return;
        }
        const unsigned int auxPrimOffset = hitPrimIdx - numOriginalTrisB;
        hitOrigTriIdxB = auxPrimOffset / 3; // Original triangle index in Mesh B
        const unsigned int edgeInOrigB = auxPrimOffset % 3; // Original edge (0, 1, or 2)

         // Optional: Get face index if buffer provided
        if (sbtData->originalFaceIndices) {
            hitFaceIndexB = sbtData->originalFaceIndices[hitOrigTriIdxB];
        }

        // Self-intersection check
        if (params.selfIntersectionCheck && rayOriginTriIdxA == hitOrigTriIdxB) {
            return; // Ignore self-hit
        }

        // Get vertices of the *original* Mesh B triangle
        // **CRITICAL:** sbtData->indices must point to the *original* indices buffer here!
        //             This might require passing both index buffers in the SBT data or params.
        //             Assuming sbtData->indices points to the correct buffer for the type...
        //             If sbtData->indices points to aux_indices, we need another pointer.
        //             Let's assume sbtData->indices always points to the *original* indices
        //             and Aux hits use this pointer based on calculated hitOrigTriIdxB.
         if (!sbtData->indices || !sbtData->vertices) {
              printf("AH Error: Missing vertex/index data for aux hit check\n");
              return;
         }
        const int3 vIdxB = sbtData->indices[hitOrigTriIdxB]; // Fetch original indices
        const Point_cu vB0 = sbtData->vertices[vIdxB.x];
        const Point_cu vB1 = sbtData->vertices[vIdxB.y];
        const Point_cu vB2 = sbtData->vertices[vIdxB.z];

        // Identify the original edge segment E'
        Point_cu edgeB_Start, edgeB_End;
        switch (edgeInOrigB) {
            case 0: { edgeB_Start = vB0; edgeB_End = vB1; break; }
            case 1: { edgeB_Start = vB1; edgeB_End = vB2; break; }
            case 2: { edgeB_Start = vB2; edgeB_End = vB0; break; }
            default: { return; }
        }

        // Calculate intersection point P
        const float t = optixGetRayTmax();
        const float3 rayO = optixGetWorldRayOrigin();
        const float3 rayD = optixGetWorldRayDirection();
        const Point_cu P = Point_cu(rayO.x, rayO.y, rayO.z) + Vec3_cu(rayD.x, rayD.y, rayD.z) * t;

        // Check if P lies on the original segment E'
        if (isPointOnSegment(P, edgeB_Start, edgeB_End)) {
            // --- Collision Confirmed (Auxiliary Hit on Edge) ---
            unsigned int writeIndex = atomicAdd(params.collisionCounter, 1);
            if (writeIndex < params.maxCollisions) {
                // Need face index for mesh A
                if (params.meshAOriginalFaceIndices){
                     // hitFaceIndexA = params.meshAOriginalFaceIndices[rayOriginTriIdxA];
                }

                params.collisionBuffer[writeIndex].triangleIndexA = rayOriginTriIdxA;
                params.collisionBuffer[writeIndex].faceIndexA = hitFaceIndexA; // Placeholder
                params.collisionBuffer[writeIndex].triangleIndexB = hitOrigTriIdxB;
                params.collisionBuffer[writeIndex].faceIndexB = hitFaceIndexB;
            } else {
                 // Buffer overflow
                 printf("Collision buffer overflow!\n");
            }
        }
    } else {
        // Should not happen with current setup
        printf("AH Error: Unknown SBT geometry type\n");
    }

    // Do NOT terminate or ignore, allow ray to continue for further potential hits.
}

// Basic miss shader
extern "C" __global__ void __miss__miss()
{
    // No action needed when a ray misses everything
}
