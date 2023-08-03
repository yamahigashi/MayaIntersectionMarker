#pragma once
/**
    Copyright (c) 2023 Takayoshi Matsumoto
    You may use, distribute, or modify this code under the terms of the MIT license.
*/

#include <glm/glm.hpp>

#include <vector>
#include <unordered_map>
#include <algorithm>

#include <maya/MGlobal.h>
#include <maya/MItMeshVertex.h>
#include <maya/MItMeshPolygon.h>
#include <maya/MIntArray.h>
#include <maya/MPointArray.h>
#include <maya/MFloatVector.h>
#include <maya/MFloatVectorArray.h>
#include <maya/MVector.h>
#include <maya/MFnMesh.h>
#include <maya/MStatus.h>
#include <maya/MPoint.h>
#include <maya/MMatrix.h>
#include <maya/MBoundingBox.h>

// https://github.com/embree/embree/blob/0fcb306c9176221219dd15e27fe0527ed334948f/common/sys/platform.h#L196
#if !defined(likely)
#if defined(_MSC_VER) && !defined(__INTEL_COMPILER) || defined(__SYCL_DEVICE_ONLY__)
#define   likely(expr) (expr)
#define unlikely(expr) (expr)
#else
#define   likely(expr) __builtin_expect((bool)(expr),true )
#define unlikely(expr) __builtin_expect((bool)(expr),false)
#endif
#endif

struct TriangleData
{
    int faceIndex;
    int triangleIndex;
    MPoint vertices[3];
    MBoundingBox bbox;

    TriangleData() = default;
    TriangleData(int faceIndex, int triangleIndex, MPoint v0, MPoint v1, MPoint v2)
        : faceIndex(faceIndex), triangleIndex(triangleIndex)
    {
        vertices[0] = v0;
        vertices[1] = v1;
        vertices[2] = v2;

        bbox.expand(v0);
        bbox.expand(v1);
        bbox.expand(v2);
    }
};

struct BoundingBox1D
{
    float min = FLT_MAX;
    float max = -FLT_MAX;

    void expand(float value)
    {
        if (value < min)
            min = value;
        if (value > max)
            max = value;
    }

    bool intersect(const BoundingBox1D& other) const
    {
        return (min <= other.max) && (max >= other.min);
    }
};


class PolyChecksum
{
public:
    PolyChecksum()
    {
        // for all possible byte values
        for (unsigned i = 0; i < 256; ++i)
        {
            unsigned long reg = i << 24;
            // for all bits in a byte
            for (int j = 0; j < 8; ++j)
            {
                bool topBit = (reg & 0x80000000) != 0;
                reg <<= 1;

                if (topBit)
                    reg ^= _key;
            }
            _table [i] = reg;
        }
    };

    virtual void        putBytes(void* bytes, size_t dataSize)
    {
        unsigned char* ptr = (unsigned char*) bytes;

        for (size_t i = 0; i < dataSize; i++)
        {
            unsigned byte = *(ptr + i);
            unsigned top = _register >> 24;
            top ^= byte;
            top &= 255;

            _register = (_register << 8) ^ _table [top];
        }
    }
    ;
    virtual int         getResult() const { return (int) _register; }

public:
    unsigned long       _table[256];
    unsigned long       _register = 0;
    unsigned long       _key = 0x04c11db7;
};


static inline int getVertexChecksum(MObject polyObject, MMatrix& offsetMatrix)
{
    PolyChecksum checksum;

    MItMeshVertex itVertex(polyObject);

    while (!itVertex.isDone())
    {
        int index = itVertex.index();
        checksum.putBytes(&index, sizeof(index));

        // topology
        MIntArray connectedVertices;
        itVertex.getConnectedVertices(connectedVertices);
        uint numConnectedVertices = connectedVertices.length();

        for (uint i = 0; i < numConnectedVertices; i++)
        {
            uint idx = connectedVertices[i];
            checksum.putBytes(&idx, sizeof(idx));
        }

        // position
        MPoint point = itVertex.position(MSpace::kObject);
        checksum.putBytes(&point, sizeof(point));

        itVertex.next();
    }

    for (int i = 0; i < 4; i++) {
        MFloatVector row = offsetMatrix[i];
        checksum.putBytes(&row, sizeof(row));
    }

    return checksum.getResult();
}


static inline MVector computePlaneNormal(const MPoint& p1, const MPoint& p2, const MPoint& p3)
{
    MVector v1 = p2 - p1;
    MVector v2 = p3 - p1;
    return v1 ^ v2; // cross product gives the normal
}


static inline MVector computePlaneOrigin(const MPoint& p1, const MPoint& p2, const MPoint& p3)
{
    return (p1 + p2 + p3) / 3.0; // average gives the center of the triangle
}


static inline bool isEdgeIntersectingPlane(const MVector& planeNormal, const MVector& planeOrigin, const MPoint& edgeStart, const MPoint& edgeEnd)
{
    // calculate the dot products
    double dotProduct1 = MVector(edgeStart - planeOrigin) * planeNormal;
    double dotProduct2 = MVector(edgeEnd - planeOrigin) * planeNormal;

    // check if the edge crosses the plane
    return dotProduct1 * dotProduct2 < 0;
}


static inline bool isPointInTriangle(const MPoint& point, const MPoint vertices[])
{
    MVector v0 = vertices[2] - vertices[0];
    MVector v1 = vertices[1] - vertices[0];
    MVector v2 = point - vertices[0];

    double d00 = v0 * v0;
    double d01 = v0 * v1;
    double d11 = v1 * v1;
    double d20 = v2 * v0;
    double d21 = v2 * v1;

    double denom = d00 * d11 - d01 * d01;

    double v = (d11 * d20 - d01 * d21) / denom;
    double w = (d00 * d21 - d01 * d20) / denom;
    double u = 1.0 - v - w;

    // Check if point is in triangle
    return (v >= 0) && (w >= 0) && (u >= 0);
}


static inline MStatus offsetPolygon(MFnMesh& meshFn, unsigned int polyIndex, float thickness)
{
    MStatus status;

    MIntArray vertexIndices;
    status = meshFn.getPolygonVertices(polyIndex, vertexIndices);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    for (unsigned int i = 0; i < vertexIndices.length(); ++i) {
        MPoint point;
        status = meshFn.getPoint(vertexIndices[i], point, MSpace::kObject);

        if (status != MS::kSuccess) {
            continue;
        }

        MVector normal;
        status = meshFn.getVertexNormal(vertexIndices[i], true, normal, MSpace::kObject);
        CHECK_MSTATUS_AND_RETURN_IT(status);

        point += normal * thickness;
        meshFn.setPoint(vertexIndices[i], point, MSpace::kObject);
    }

    return MS::kSuccess;
}


static MPoint computeEdgePlaneIntersection(const MVector& planeNormal, const MVector& planeOrigin, const MPoint& edgeStart, const MPoint& edgeEnd)
{
    MVector edgeDirection = edgeEnd - edgeStart;
    double t = ((planeOrigin - edgeStart) * planeNormal) / (edgeDirection * planeNormal);
    return edgeStart + t * edgeDirection;
}


static bool isPointInsideTriangle(const MPoint& point, const TriangleData& triangle)
{
    MVector v0 = triangle.vertices[2] - triangle.vertices[0];
    MVector v1 = triangle.vertices[1] - triangle.vertices[0];
    MVector v2 = point - triangle.vertices[0];

    double dot00 = v0 * v0;
    double dot01 = v0 * v1;
    double dot02 = v0 * v2;
    double dot11 = v1 * v1;
    double dot12 = v1 * v2;

    double invDenom = 1.0 / (dot00 * dot11 - dot01 * dot01);
    double u = (dot11 * dot02 - dot01 * dot12) * invDenom;
    double v = (dot00 * dot12 - dot01 * dot02) * invDenom;

    return (u >= 0) && (v >= 0) && (u + v < 1);
}


__forceinline const double size(const MBoundingBox& bbox)
{
    double width = bbox.width();
    double height = bbox.height();
    double depth = bbox.depth();

    return 2.0 * (width * height + width * depth + height * depth);
}


__forceinline const double halfarea(const MBoundingBox& bbox)
{
    double width = bbox.width();
    double height = bbox.height();
    double depth = bbox.depth();

    return (width * height + width * depth + height * depth);
}


__forceinline const double area(const MBoundingBox& bbox)
{
    return 2.0 * halfarea(bbox);
}


__forceinline const MBoundingBox merge(const MBoundingBox& bbox1, const MBoundingBox& bbox2)
{
    MBoundingBox bbox = MBoundingBox(bbox1);
    bbox.expand(bbox2);
    return bbox;
}



__forceinline int maxDim( const MFloatVector& a )
{
    const MFloatVector b = MFloatVector(fabsf(a.x), fabsf(a.y), fabsf(a.z));
    if (b.x > b.y) {
        if (b.x > b.z) return 0; else return 2;
    } else {
        if (b.y > b.z) return 1; else return 2;
    }
}


__forceinline int maxDim( const glm::vec3& a )
{
    const glm::vec3 b = glm::vec3(fabsf(a.x), fabsf(a.y), fabsf(a.z));
    if (b.x > b.y) {
        if (b.x > b.z) return 0; else return 2;
    } else {
        if (b.y > b.z) return 1; else return 2;
    }
}


__forceinline float det(const glm::vec2& a, const glm::vec2& b) {
    return a.x * b.y - a.y * b.x;
}

__forceinline float computePointOnSegment(
        float pointA,
        float pointB,
        float ratioA,
        float ratioB
) {
    if (std::fabs(ratioA - ratioB) < std::numeric_limits<float>::epsilon()) {
        throw std::invalid_argument("ratioA and ratioB are too close");
    }

    float ratio = ratioA / (ratioA - ratioB);
    return glm::mix(pointA, pointB, ratio);
}



static inline bool point_line_side(
        const glm::vec2& p,
        const glm::vec2& a0,
        const glm::vec2& a1
) {
    return det(p-a0,a0-a1) >= 0.0f;
}

static inline bool point_inside_triangle(
        const glm::vec2& p,
        const glm::vec2& a,
        const glm::vec2& b,
        const glm::vec2& c
) {
    const bool pab = point_line_side(p,a,b); 
    const bool pbc = point_line_side(p,b,c);
    const bool pca = point_line_side(p,c,a);
    return pab == pbc && pab == pca;
}


static inline bool intersect_line_line(
        const glm::vec2& a0,
        const glm::vec2& a1,
        const glm::vec2& b0,
        const glm::vec2& b1
) {
    const bool different_sides0 = point_line_side(b0,a0,a1) != point_line_side(b1,a0,a1);
    const bool different_sides1 = point_line_side(a0,b0,b1) != point_line_side(a1,b0,b1);
    return different_sides0 && different_sides1;
}


// ref: https://github.com/embree/embree/blob/0fcb306c9176221219dd15e27fe0527ed334948f/kernels/geometry/triangle_triangle_intersector.h#L66
static inline bool intersectTriangleTriangle (
    const glm::vec2& a0,
    const glm::vec2& a1,
    const glm::vec2& a2,

    const glm::vec2& b0,
    const glm::vec2& b1,
    const glm::vec2& b2
) {

    return true;

    const bool a01_b01 = intersect_line_line(a0,a1,b0,b1); 
    if (a01_b01) return true;

    const bool a01_b12 = intersect_line_line(a0,a1,b1,b2);
    if (a01_b12) return true;

    const bool a01_b20 = intersect_line_line(a0,a1,b2,b0);
    if (a01_b20) return true;

    const bool a12_b01 = intersect_line_line(a1,a2,b0,b1);
    if (a12_b01) return true;

    const bool a12_b12 = intersect_line_line(a1,a2,b1,b2);
    if (a12_b12) return true;

    const bool a12_b20 = intersect_line_line(a1,a2,b2,b0);
    if (a12_b20) return true;

    const bool a20_b01 = intersect_line_line(a2,a0,b0,b1);
    if (a20_b01) return true;

    const bool a20_b12 = intersect_line_line(a2,a0,b1,b2);
    if (a20_b12) return true;

    const bool a20_b20 = intersect_line_line(a2,a0,b2,b0);
    if (a20_b20) return true;

    bool a_in_b = (
            point_inside_triangle(a0,b0,b1,b2) &&
            point_inside_triangle(a1,b0,b1,b2) &&
            point_inside_triangle(a2,b0,b1,b2)
    );
    if (a_in_b) return true;

    bool b_in_a = (
            point_inside_triangle(b0,a0,a1,a2) &&
            point_inside_triangle(b1,a0,a1,a2) &&
            point_inside_triangle(b2,a0,a1,a2)
    );
    if (b_in_a) return true;

    return false;
}


static inline bool intersectTriangleTriangle (
    const glm::vec3& a0,
    const glm::vec3& a1,
    const glm::vec3& a2,

    const glm::vec3& b0,
    const glm::vec3& b1,
    const glm::vec3& b2
) {
    const float eps = 1E-5f;

    /* calculate triangle planes */
    const glm::vec3 Na = cross(a1-a0,a2-a0);
    const float     Ca = dot(Na,a0);
    const glm::vec3 Nb = cross(b1-b0,b2-b0);
    const float     Cb = dot(Nb,b0);

    /* project triangle A onto plane B */
    const float    da0 = dot(Nb,a0)-Cb;
    const float    da1 = dot(Nb,a1)-Cb;
    const float    da2 = dot(Nb,a2)-Cb;
    if (std::max({da0,da1,da2}) < -eps) return false;
    if (std::min({da0,da1,da2}) > +eps) return false;
    //CSTAT(bvh_collide_prim_intersections4++);

    /* project triangle B onto plane A */
    const float db0 = dot(Na,b0)-Ca;
    const float db1 = dot(Na,b1)-Ca;
    const float db2 = dot(Na,b2)-Ca;
    if (std::max({db0,db1,db2}) < -eps) return false;
    if (std::min({db0,db1,db2}) > +eps) return false;
    //CSTAT(bvh_collide_prim_intersections5++);

    if (unlikely
            // coplanar
            (
                (
                    std::fabs(da0) < eps &&
                    std::fabs(da1) < eps &&
                    std::fabs(da2) < eps
                ) || (
                    std::fabs(db0) < eps &&
                    std::fabs(db1) < eps &&
                    std::fabs(db2) < eps
                )
            )
    ) {
        const unsigned int dz = maxDim(Na);
        const unsigned int dx = (dz+1)%3;
        const unsigned int dy = (dx+1)%3;
        const glm::vec2 A0(a0[dx],a0[dy]);
        const glm::vec2 A1(a1[dx],a1[dy]);
        const glm::vec2 A2(a2[dx],a2[dy]);
        const glm::vec2 B0(b0[dx],b0[dy]);
        const glm::vec2 B1(b1[dx],b1[dy]);
        const glm::vec2 B2(b2[dx],b2[dy]);
        return intersectTriangleTriangle(A0,A1,A2, B0,B1,B2);
    }

    const glm::vec3 D = cross(Na,Nb);
    const float pa0 = dot(D,a0);
    const float pa1 = dot(D,a1);
    const float pa2 = dot(D,a2);
    const float pb0 = dot(D,b0);
    const float pb1 = dot(D,b1);
    const float pb2 = dot(D,b2);

    BoundingBox1D ba;
    if (std::min(da0,da1) <= 0.0f && std::max(da0,da1) >= 0.0f && abs(da0-da1) > 0.0f) ba.expand(computePointOnSegment(pa0,pa1,da0,da1));
    if (std::min(da1,da2) <= 0.0f && std::max(da1,da2) >= 0.0f && abs(da1-da2) > 0.0f) ba.expand(computePointOnSegment(pa1,pa2,da1,da2));
    if (std::min(da2,da0) <= 0.0f && std::max(da2,da0) >= 0.0f && abs(da2-da0) > 0.0f) ba.expand(computePointOnSegment(pa2,pa0,da2,da0));

    BoundingBox1D bb;
    if (std::min(db0,db1) <= 0.0f && std::max(db0,db1) >= 0.0f && abs(db0-db1) > 0.0f) bb.expand(computePointOnSegment(pb0,pb1,db0,db1));
    if (std::min(db1,db2) <= 0.0f && std::max(db1,db2) >= 0.0f && abs(db1-db2) > 0.0f) bb.expand(computePointOnSegment(pb1,pb2,db1,db2));
    if (std::min(db2,db0) <= 0.0f && std::max(db2,db0) >= 0.0f && abs(db2-db0) > 0.0f) bb.expand(computePointOnSegment(pb2,pb0,db2,db0));

    return ba.intersect(bb);
}


static inline bool intersectTriangleTriangle (
    const MFloatVector& a0,
    const MFloatVector& a1,
    const MFloatVector& a2,

    const MFloatVector& b0,
    const MFloatVector& b1,
    const MFloatVector& b2
) {
    return intersectTriangleTriangle(
        glm::vec3(a0.x,a0.y,a0.z),
        glm::vec3(a1.x,a1.y,a1.z),
        glm::vec3(a2.x,a2.y,a2.z),

        glm::vec3(b0.x,b0.y,b0.z),
        glm::vec3(b1.x,b1.y,b1.z),
        glm::vec3(b2.x,b2.y,b2.z)
    );
}


static inline bool intersectTriangleTriangle (
    const MVector& a0,
    const MVector& a1,
    const MVector& a2,

    const MVector& b0,
    const MVector& b1,
    const MVector& b2
) {
    return intersectTriangleTriangle(

        glm::vec3(a0.x,a0.y,a0.z),
        glm::vec3(a1.x,a1.y,a1.z),
        glm::vec3(a2.x,a2.y,a2.z),

        glm::vec3(b0.x,b0.y,b0.z),
        glm::vec3(b1.x,b1.y,b1.z),
        glm::vec3(b2.x,b2.y,b2.z)
    );
}

static inline bool intersectTriangleTriangle (
    const MPoint& a0,
    const MPoint& a1,
    const MPoint& a2,

    const MPoint& b0,
    const MPoint& b1,
    const MPoint& b2
) {
    return intersectTriangleTriangle(

        glm::vec3(a0.x,a0.y,a0.z),
        glm::vec3(a1.x,a1.y,a1.z),
        glm::vec3(a2.x,a2.y,a2.z),

        glm::vec3(b0.x,b0.y,b0.z),
        glm::vec3(b1.x,b1.y,b1.z),
        glm::vec3(b2.x,b2.y,b2.z)
    );
}

static inline bool intersectTriangleTriangle (
    const TriangleData& a,
    const TriangleData& b
) {
    return intersectTriangleTriangle(
        a.vertices[0],
        a.vertices[1],
        a.vertices[2],

        b.vertices[0],
        b.vertices[1],
        b.vertices[2]
    );
}


static inline bool intersectBoxBox (
    const MBoundingBox& a,
    const MBoundingBox& b
) {
    return a.intersects(b);
}

static inline bool boxContainsAnyVertices (
    const MBoundingBox& box,
    const TriangleData& triangle
) {
    for (const MPoint& vertex : triangle.vertices) {
        if (box.contains(vertex)) {
            return true;
        }
    }
    return false;
}


static inline bool boxContainsAllVertices (
    const MBoundingBox& box,
    const TriangleData& triangle
) {
    for (const MPoint& vertex : triangle.vertices) {
        if (!box.contains(vertex)) {
            return false;
        }
    }
    return true;
}


static inline bool intersectBoxTriangle (
    const MBoundingBox& box,
    const TriangleData& triangle
) {

    // if any vertex is inside the box, return true
    if (boxContainsAnyVertices(box, triangle)) { return true; }

    // if no bounding box intersection, return false
    MBoundingBox triBox;
    triBox.expand(triangle.vertices[0]);
    triBox.expand(triangle.vertices[1]);
    triBox.expand(triangle.vertices[2]);
    if(!intersectBoxBox(box, triBox)){  return false; }

    const glm::vec3& a0 = glm::vec3(triangle.vertices[0].x, triangle.vertices[0].y, triangle.vertices[0].z);
    const glm::vec3& a1 = glm::vec3(triangle.vertices[1].x, triangle.vertices[1].y, triangle.vertices[1].z);
    const glm::vec3& a2 = glm::vec3(triangle.vertices[2].x, triangle.vertices[2].y, triangle.vertices[2].z);

    const glm::vec3& min = glm::vec3(box.min().x, box.min().y, box.min().z);
    const glm::vec3& max = glm::vec3(box.max().x, box.max().y, box.max().z);

    const glm::vec3& b0 = glm::vec3(min.x, min.y, min.z);
    const glm::vec3& b1 = glm::vec3(min.x, min.y, max.z);
    const glm::vec3& b2 = glm::vec3(min.x, max.y, min.z);
    const glm::vec3& b3 = glm::vec3(min.x, max.y, max.z);
    const glm::vec3& b4 = glm::vec3(max.x, min.y, min.z);
    const glm::vec3& b5 = glm::vec3(max.x, min.y, max.z);
    const glm::vec3& b6 = glm::vec3(max.x, max.y, min.z);
    const glm::vec3& b7 = glm::vec3(max.x, max.y, max.z);


    return intersectTriangleTriangle(a0,a1,a2, b0,b2,b6) ||
           intersectTriangleTriangle(a0,a1,a2, b0,b6,b4) ||

           intersectTriangleTriangle(a0,a1,a2, b0,b1,b3) ||
           intersectTriangleTriangle(a0,a1,a2, b0,b3,b2) ||

           intersectTriangleTriangle(a0,a1,a2, b0,b4,b5) ||
           intersectTriangleTriangle(a0,a1,a2, b0,b5,b1) ||

           intersectTriangleTriangle(a0,a1,a2, b7,b3,b1) ||
           intersectTriangleTriangle(a0,a1,a2, b7,b1,b5) ||

           intersectTriangleTriangle(a0,a1,a2, b7,b6,b2) ||
           intersectTriangleTriangle(a0,a1,a2, b7,b2,b3) ||

           intersectTriangleTriangle(a0,a1,a2, b7,b4,b6) ||
           intersectTriangleTriangle(a0,a1,a2, b7,b5,b4);

}

