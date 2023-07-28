/**
    Copyright (c) 2023 Takayoshi Matsumoto
    You may use, distribute, or modify this code under the terms of the MIT license.
*/
#pragma once

#include "SpatialDivisionKernel.h"

#include <string>
#include <vector>
#include <unordered_set>

#include <maya/MDagPath.h>
#include <maya/MDataHandle.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MPxNode.h>
#include <maya/MPxLocatorNode.h>
#include <maya/MObject.h>
#include <maya/MPlug.h>
#include <maya/MString.h>
#include <maya/MTypeId.h>

#define MESH_A             "inMeshA"
#define MESH_B             "inMeshB"
#define OFFSET_MATRIX      "offsetMatrix"
#define REST_INTERSECTED   "restIntersected"

#define KERNEL             "kernel"
#define OUTPUT_INTERSECTED "outputIntersected"
#define OUT_MESH           "outMesh"


class IntersectionMarkerNode : public MPxLocatorNode
{
public:
                        IntersectionMarkerNode();
                        ~IntersectionMarkerNode() override;

    static  void*       creator();
    static  MStatus     initialize();

            MStatus     postEvaluation(const  MDGContext& context, const MEvaluationNode& evaluationNode, PostEvaluationType evalType) override;
            MStatus     compute(const MPlug &plug, MDataBlock &dataBlock);

    // static MStatus      onInitializePlugin();
    // static MStatus      onUninitializePlugin();

    static MStatus      getCacheKey(MObject &node, std::string &key);
    static MStatus      getCacheKeyFromMesh(MObject &meshObjA, MObject &meshObjB, std::string &key);

std::unique_ptr<SpatialDivisionKernel> getActiveKernel() const;
            MStatus     checkIntersections(MObject &meshAObject, MObject &meshBObject, std::unique_ptr<SpatialDivisionKernel> kernel, MMatrix offset);
            MStatus     getInputDagMesh(const MObject inputAttr, MFnMesh &outMesh) const;
      MBoundingBox      getBoundingBox(const MObject &meshObject) const;
           MStatus      createMeshFromTriangles(const MObject& meshAObject, const MIntArray& intersectedTriangleIDs, MFnMesh& outputMeshFn);
              bool      checkIntersectionsDetailed(const TriangleData triA, const TriangleData triB) const;

             MPlug      meshAPlug() const { return MPlug(thisMObject(), meshA); }
             MPlug      meshBPlug() const { return MPlug(thisMObject(), meshB); }

public:
    static MObject      meshA;
    static MObject      meshB;
    static MObject      offsetMatrix;
    static MObject      restIntersected;

    static MObject      vertexChecksumA;
    static MObject      vertexChecksumB;
    static MObject      kernelType;

    static MObject      outputIntersected;
    
    static MString      NODE_NAME;
    static MTypeId      NODE_ID;

    static MString      drawDbClassification;
    static MString      drawRegistrantId;

    std::unordered_set<int> intersectedFaceIdsA;
    std::unordered_set<int> intersectedFaceIdsB;
};
