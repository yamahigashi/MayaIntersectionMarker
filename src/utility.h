/**
    Copyright (c) 2023 Takayoshi Matsumoto
    You may use, distribute, or modify this code under the terms of the MIT license.
*/
#pragma once

#include <vector>
#include <unordered_map>

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
#include <maya/MBoundingBox.h>


struct TriangleData {
    int faceIndex;
    MPoint vertices[3];
};



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
