/**
    Copyright (c) 2023 Takayoshi Matsumoto
    You may use, distribute, or modify this code under the terms of the MIT license.
*/
#ifndef UTILITY_H
#define UTILITY_H

#include "SpatialDivisionKernel.h"

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


class PolyChecksum
{
public:
                        PolyChecksum();
    virtual void        putBytes(void* bytes, size_t dataSize);
    virtual int         getResult();

public:
	unsigned long       _table[256];
	unsigned long       _register = 0;
	unsigned long       _key = 0x04c11db7;
};


#endif 


PolyChecksum::PolyChecksum()
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
}

void PolyChecksum::putBytes(void* bytes, size_t dataSize)
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


int PolyChecksum::getResult()
{
    return (int) this->_register;
}


int getVertexChecksum(MObject polyObject)
{
    PolyChecksum checksum;

    MItMeshVertex itVertex(polyObject);
    
    while (!itVertex.isDone())
    {
        int index = itVertex.index();
        checksum.putBytes(&index, sizeof(index));

        MIntArray connectedVertices;
        itVertex.getConnectedVertices(connectedVertices);
        uint numConnectedVertices = connectedVertices.length();

        for (uint i = 0; i < numConnectedVertices; i++)
        {
            uint idx = connectedVertices[i];
            checksum.putBytes(&idx, sizeof(idx));
        }

        itVertex.next();
    }

    return checksum.getResult();
}


inline MVector computePlaneNormal(const MPoint& p1, const MPoint& p2, const MPoint& p3)
{
    MVector v1 = p2 - p1;
    MVector v2 = p3 - p1;
    return v1 ^ v2; // cross product gives the normal
}


inline MVector computePlaneOrigin(const MPoint& p1, const MPoint& p2, const MPoint& p3)
{
    return (p1 + p2 + p3) / 3.0; // average gives the center of the triangle
}


inline bool isEdgeIntersectingPlane(const MVector& planeNormal, const MVector& planeOrigin, const MPoint& edgeStart, const MPoint& edgeEnd)
{
    // calculate the dot products
    double dotProduct1 = MVector(edgeStart - planeOrigin) * planeNormal;
    double dotProduct2 = MVector(edgeEnd - planeOrigin) * planeNormal;

    // check if the edge crosses the plane
    return dotProduct1 * dotProduct2 < 0;
}


inline bool isPointInTriangle(const MPoint& point, const MPoint vertices[])
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


inline MStatus offsetPolygon(MFnMesh& meshFn, unsigned int polyIndex, float thickness)
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


MPoint computeEdgePlaneIntersection(const MVector& planeNormal, const MVector& planeOrigin, const MPoint& edgeStart, const MPoint& edgeEnd) {
    MVector edgeDirection = edgeEnd - edgeStart;
    double t = ((planeOrigin - edgeStart) * planeNormal) / (edgeDirection * planeNormal);
    return edgeStart + t * edgeDirection;
}


bool isPointInsideTriangle(const MPoint& point, const TriangleData& triangle) {
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
