/**
    Copyright (c) 2023 Takayoshi Matsumoto
    You may use, distribute, or modify this code under the terms of the MIT license.
*/

#include "meshData.h"
#include "intersectionMarkerNode.h"

#include <string>

#include <maya/MDataBlock.h>
#include <maya/MFnData.h>
#include <maya/MFnIntArrayData.h>
#include <maya/MFnMesh.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MFnNumericData.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MGlobal.h>
#include <maya/MIntArray.h>
#include <maya/MPlug.h>
#include <maya/MPxNode.h>
#include <maya/MObject.h>
#include <maya/MObjectHandle.h>

using namespace std; 

MObject IntersectionMarkerNode::meshA;
MObject IntersectionMarkerNode::meshB;

MObject IntersectionMarkerNode::kernel;
MObject IntersectionMarkerNode::vertexChecksumA;
MObject IntersectionMarkerNode::vertexChecksumB;

IntersectionMarkerNode::IntersectionMarkerNode() {}
IntersectionMarkerNode::~IntersectionMarkerNode() {}

void* IntersectionMarkerNode::creator()
{
    return new IntersectionMarkerNode();
}


MStatus IntersectionMarkerNode::initialize()
{
    MStatus status;

    MFnTypedAttribute tAttr;
    MFnNumericAttribute nAttr;
    MFnEnumAttribute eAttr;

    // Initialize Mesh A
    meshA = tAttr.create(MESH_A, MESH_A, MFnData::kMesh);
    tAttr.setStorable(true);
    tAttr.setKeyable(false);
    status = addAttribute(meshA);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    // Initialize Mesh B
    meshB = tAttr.create(MESH_B, MESH_B, MFnData::kMesh);
    tAttr.setStorable(true);
    tAttr.setKeyable(false);
    status = addAttribute(meshB);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    // Initialize Vertex Checksum
    vertexChecksumA = nAttr.create(VERTEX_CHECKSUM_A, VERTEX_CHECKSUM_A, MFnNumericData::kInt, 0);
    nAttr.setArray(true);
    nAttr.setUsesArrayDataBuilder(true);
    status = addAttribute(vertexChecksumA);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    // Initialize Vertex Checksum
    vertexChecksumB = nAttr.create(VERTEX_CHECKSUM_B, VERTEX_CHECKSUM_B, MFnNumericData::kInt, 0);
    nAttr.setArray(true);
    nAttr.setUsesArrayDataBuilder(true);
    status = addAttribute(vertexChecksumB);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    // Initialize Kernel
    kernel = eAttr.create(KERNEL, KERNEL, 0, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    eAttr.addField("Octree", 0);
    eAttr.addField("KDTree", 1);
    status = addAttribute(kernel);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    // Add dependencies
    status = attributeAffects(meshA, vertexChecksumA);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    status = attributeAffects(meshB, vertexChecksumB);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    // status = attributeAffects(kernel, result);
    // CHECK_MSTATUS_AND_RETURN_IT(status);

    return MS::kSuccess;
}


MStatus IntersectionMarkerNode::compute(const MPlug &plug, MDataBlock &dataBlock)
{
    return MStatus::kSuccess;
}


MStatus IntersectionMarkerNode::setValue(MFnDependencyNode &fnNode, const char* attributeName, int &value)
{
    MStatus status;

    MPlug plug = fnNode.findPlug(attributeName, false, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    status = plug.setValue(value);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    return MStatus::kSuccess;
}


MStatus IntersectionMarkerNode::getValue(MFnDependencyNode &fnNode, const char* attributeName, int &value)
{
    MStatus status;

    MPlug plug = fnNode.findPlug(attributeName, true, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    status = plug.getValue(value);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    return MStatus::kSuccess;
}


MStatus IntersectionMarkerNode::setValues(MFnDependencyNode &fnNode, const char* attributeName, vector<int> &values)
{
    MStatus status;

    MPlug plug = fnNode.findPlug(attributeName, false, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    MIntArray valueArray;

    for (int &v : values)
    { 
        valueArray.append(v);
    }

    MFnIntArrayData valueArrayData;

    MObject data = valueArrayData.create(valueArray, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    status = plug.setMObject(data);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    return MStatus::kSuccess;
}


MStatus IntersectionMarkerNode::getValues(MFnDependencyNode &fnNode, const char* attributeName, vector<int> &values)
{
    MStatus status;

    MPlug plug = fnNode.findPlug(attributeName, true, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    MObject data = plug.asMObject();

    MFnIntArrayData valueArrayData(data, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    MIntArray valueArray = valueArrayData.array(&status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    uint numberOfValues = valueArray.length();
    values.resize((int) numberOfValues);

    for (uint i = 0; i < numberOfValues; i++)
    {
        values[i] = valueArray[i];
    }

    return MStatus::kSuccess;
}


MStatus IntersectionMarkerNode::getCacheKey(MObject &node, string &key)
{
    MStatus status;
    
    int vertexChecksumA;
    int vertexChecksumB;

    MFnDependencyNode fnNode(node);

    IntersectionMarkerNode::getValue(fnNode, VERTEX_CHECKSUM_A, vertexChecksumA);
    IntersectionMarkerNode::getValue(fnNode, VERTEX_CHECKSUM_B, vertexChecksumB);

    if (vertexChecksumA == -1 || vertexChecksumB == -1)
    {
        MString cacheErrorMessage("Cannot cache ^1s because it has incomplete data. Delete it and try again.");
        cacheErrorMessage.format(cacheErrorMessage, fnNode.name());

        MGlobal::displayWarning(cacheErrorMessage);
    } else {
        key = to_string(vertexChecksum_A) + ":" + 
              to_string(vertexChecksum_B);
    }

    return MStatus::kSuccess;
}


MStatus IntersectionMarkerNode::getCacheKeyFromMesh(MDagPath &dagPathA, MDagPath &dagPathB, string &key)
{
    MStatus status;
    
    MFnMesh fnMeshA(dagPathA);
    MFnMesh fnMeshB(dagPathB);

    int vertexChecksumA = (int) MeshData::getVertexChecksum(dagPathA);
    int vertexChecksumB = (int) MeshData::getVertexChecksum(dagPathB);

    key = to_string(vertexChecksum_A) + ":" + 
          to_string(vertexChecksum_B);

    return MStatus::kSuccess; 
}
