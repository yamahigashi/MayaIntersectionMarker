/**
    Copyright (c) 2023 Takayoshi Matsumoto
    You may use, distribute, or modify this code under the terms of the MIT license.
*/

#include "utility.h"
#include "intersectionMarkerNode.h"
#include "KDTreeKernel.h"
#include "OctreeKernel.h"

#include <string>

#include <maya/MDagPath.h>
#include <maya/MDataBlock.h>
#include <maya/MFnData.h>
#include <maya/MIntArray.h>
#include <maya/MItMeshPolygon.h>
#include <maya/MPointArray.h>
#include <maya/MFnIntArrayData.h>
#include <maya/MFnArrayAttrsData.h>
#include <maya/MArrayDataBuilder.h>
#include <maya/MArrayDataHandle.h>
#include <maya/MFnMesh.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MFnNumericData.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MGlobal.h>
#include <maya/MPlug.h>
#include <maya/MPxNode.h>
#include <maya/MObject.h>
#include <maya/MObjectHandle.h>
#include <maya/MMatrix.h>

MObject IntersectionMarkerNode::meshA;
MObject IntersectionMarkerNode::meshB;

MObject IntersectionMarkerNode::vertexChecksumA;
MObject IntersectionMarkerNode::vertexChecksumB;
MObject IntersectionMarkerNode::kernelType;

MObject IntersectionMarkerNode::outputIntersected;

IntersectionMarkerNode::IntersectionMarkerNode() {}
IntersectionMarkerNode::~IntersectionMarkerNode() {}

void* IntersectionMarkerNode::creator()
{
    return new IntersectionMarkerNode();
}



MStatus IntersectionMarkerNode::initialize()
{
    MStatus status;

    MFnTypedAttribute tOputAttr;  // means writable output attribute
    MFnTypedAttribute tInputAttr;  // meanst read-only input attribute
    MFnNumericAttribute nAttr;
    MFnEnumAttribute eAttr;

    // Initialize Mesh A
    meshA = tInputAttr.create(MESH_A, MESH_A, MFnData::kMesh, MObject::kNullObj, &status);
    status = addAttribute(meshA);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    // Initialize Mesh B
    meshB = tInputAttr.create(MESH_B, MESH_B, MFnData::kMesh, MObject::kNullObj, &status);
    status = addAttribute(meshB);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    // Initialize Vertex Checksum of Mesh A
    vertexChecksumA = nAttr.create(VERTEX_CHECKSUM_A, VERTEX_CHECKSUM_A, MFnNumericData::kInt, -1);
    nAttr.setStorable(true);
    nAttr.setKeyable(false);
    nAttr.setWritable(false);
    status = addAttribute(vertexChecksumA);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    // Initialize Vertex Checksum of Mesh B
    vertexChecksumB = nAttr.create(VERTEX_CHECKSUM_B, VERTEX_CHECKSUM_B, MFnNumericData::kInt, -1);
    nAttr.setStorable(true);
    nAttr.setKeyable(false);
    nAttr.setWritable(false);
    status = addAttribute(vertexChecksumB);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    // Initialize Kernel
    kernelType = eAttr.create(KERNEL, KERNEL, 0, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    eAttr.addField("Octree", 0);
    eAttr.addField("KDTree", 1);
    status = addAttribute(kernelType);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    // Initialize Output Intersected
    outputIntersected = tOputAttr.create(OUTPUT_INTERSECTED, OUTPUT_INTERSECTED, MFnData::kIntArray);
    tOputAttr.setStorable(false);
    tOputAttr.setKeyable(false);
    tOputAttr.setWritable(false);
    tOputAttr.setReadable(true);
    status = addAttribute(outputIntersected);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    // Add dependencies
    status = attributeAffects(meshA, vertexChecksumA);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    status = attributeAffects(meshB, vertexChecksumB);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    status = attributeAffects(kernelType, outputIntersected);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    status = attributeAffects(vertexChecksumA, outputIntersected);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    status = attributeAffects(vertexChecksumB, outputIntersected);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    return MS::kSuccess;
}


// The compute function is called when the node is marked dirty in Maya, 
// meaning it has been modified in some way. In this function, we need to
// perform intersection tests and determine where to draw markers based on the results.
MStatus IntersectionMarkerNode::compute(const MPlug &plug, MDataBlock &dataBlock)
{
    MStatus status;
    MGlobal::displayInfo("Computing...");

    // Check if the plug parameter corresponds to the attribute that this node
    // is supposed to handle
    if (plug != outputIntersected) {
        // return MStatus::kUnknownParameter;
    }


    // Get necessary input data from the dataBlock. This is usually data from
    // the input attributes of the node.
    MGlobal::displayInfo("get input data...");
    MDataHandle meshAHandle = dataBlock.inputValue(meshA);
    MDataHandle meshBHandle = dataBlock.inputValue(meshB);

    // Get the MObject of the meshes
    MObject meshAObject = meshAHandle.asMesh();
    MObject meshBObject = meshBHandle.asMesh();
    // -------------------------------------------------------------------------------------------

    // Get the dag path of the meshes
    MGlobal::displayInfo("get dag path of the meshes...");
    // MFnMesh fnMeshA(meshAObject);
    // MFnMesh fnMeshB(meshBObject);
    // MDagPath dagPathA;
    // MDagPath dagPathB;
    // fnMeshA.getPath(dagPathA);
    // fnMeshB.getPath(dagPathB);

    // update checksums
    MGlobal::displayInfo("update checksums...");
    int newCheckA = getVertexChecksum(meshAObject);
    int newCheckB = getVertexChecksum(meshBObject);

    MDataHandle vertexChecksumAHandle = dataBlock.outputValue(vertexChecksumA, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    MDataHandle vertexChecksumBHandle = dataBlock.outputValue(vertexChecksumB, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    int checkA = vertexChecksumAHandle.asInt();
    int checkB = vertexChecksumBHandle.asInt();

    // If the checksums are the same, then we don't need to do anything
    // because the meshes have not changed.
    if (checkA == newCheckA && checkB == newCheckB) {
        MGlobal::displayInfo("no change in meshes...");
        // return MS::kSuccess;
    }

    vertexChecksumAHandle.set(newCheckA);
    vertexChecksumAHandle.setClean();
    vertexChecksumBHandle.set(newCheckB);
    vertexChecksumBHandle.setClean();

    // Build kernel
    MGlobal::displayInfo("build kernel...");
    // MDataHandle kernelTypeHandle = dataBlock.inputValue(kernelType, &status);
    // CHECK_MSTATUS_AND_RETURN_IT(status);
    // int kernelType = kernelTypeHandle.asShort();

    std::unique_ptr<SpatialDivisionKernel> kernel = getActiveKernel();
    bbox = getBoundingBox(meshA);
    status = kernel->build(meshAObject, bbox);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    MIntArray intersectedTriangleIDs = checkIntersections(meshBObject, std::move(kernel));

    // Get output data handle
    MDataHandle outputHandle = dataBlock.outputValue(outputIntersected, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    MFnIntArrayData outputData;
    MObject outputDataObject = outputData.create(intersectedTriangleIDs, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    // Set the output data handle
    outputHandle.set(outputDataObject);

    // Mark the output data handle as clean
    // kernelTypeHandle.setClean();
    meshAHandle.setClean();
    meshBHandle.setClean();
    outputHandle.setClean();
    dataBlock.setClean(plug);

    return MStatus::kSuccess;
}


MIntArray IntersectionMarkerNode::checkIntersections(MObject meshObject, std::unique_ptr<SpatialDivisionKernel> kernel) const
{
    MStatus status;
    TriangleData triangle;
    MIntArray intersectedTriangleIDs;
    MGlobal::displayInfo("checkIntersections...");

    // Iterate through the triangles in meshB
    MItMeshPolygon itPoly(meshObject);
    int numTrianglesB = itPoly.count();
    for (int i = 0; i < numTrianglesB; i++) {
        int prevIndex;
        status = itPoly.setIndex(i, prevIndex);

        int numTriangles;
        itPoly.numTriangles(numTriangles);

        for (int j = 0; j < numTriangles; ++j) {
            // Get the vertices of the triangle
            MPointArray vertices;
            MIntArray vertexList;
            itPoly.getTriangle(j, vertices, vertexList, MSpace::kWorld);

            triangle.vertices[0] = vertices[0];
            triangle.vertices[1] = vertices[1];
            triangle.vertices[2] = vertices[2];

            // Check intersection between triangle and the octree (kernel)
            std::vector<TriangleData> intersectedTriangles = kernel->queryIntersected(triangle);

            // If there is any intersection, store the intersection data into outputDataHandle
            if (!intersectedTriangles.empty()) {
                intersectedTriangleIDs.append(i);
                continue;
            }
        }
    }

    MGlobal::displayInfo(MString("intersectedTriangleIDs is ") + std::to_wstring(intersectedTriangleIDs.length()).c_str());
    return intersectedTriangleIDs;
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


MStatus IntersectionMarkerNode::setValues(MFnDependencyNode &fnNode, const char* attributeName, std::vector<int> &values)
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


MStatus IntersectionMarkerNode::getValues(MFnDependencyNode &fnNode, const char* attributeName, std::vector<int> &values)
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


MStatus IntersectionMarkerNode::getCacheKey(MObject &node, std::string &key)
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
        key = std::to_string(vertexChecksumA) + ":" + 
              std::to_string(vertexChecksumB);
    }

    return MStatus::kSuccess;
}


MStatus IntersectionMarkerNode::getCacheKeyFromMesh(MObject &meshAObject, MObject &meshBObject, std::string &key)
{
    MStatus status;
    
    int vertexChecksumA = (int) getVertexChecksum(meshAObject);
    int vertexChecksumB = (int) getVertexChecksum(meshBObject);

    key = std::to_string(vertexChecksumA) + ":" + 
          std::to_string(vertexChecksumB);

    return MStatus::kSuccess; 
}


std::unique_ptr<SpatialDivisionKernel> IntersectionMarkerNode::getActiveKernel() const
{
    // Get the value of the 'kernel' attribute
    MPlug kernelPlug(thisMObject(), kernelType);
    short kernelValue;
    kernelPlug.getValue(kernelValue);

    // Create the appropriate kernel based on the attribute value
    switch (kernelValue) {
    case 0: // Octree
        return std::make_unique<OctreeKernel>();
    case 1: // KDTree
        return std::make_unique<KDTreeKernel>();
    default:
        return nullptr;
    }
}


MStatus IntersectionMarkerNode::getInputDagMesh(const MObject inputAttr, MFnMesh &outMesh) const
{
    MStatus status;
    MPlug inputMeshPlug(thisMObject(), inputAttr);

    // Get the connected source node of the input plug.
    MPlugArray connectedPlugs;
    inputMeshPlug.connectedTo(connectedPlugs, true, false);
    if (connectedPlugs.length() == 0) {
        return MStatus::kFailure;  // No source node is connected.
    }
    MObject sourceNode = connectedPlugs[0].node();

    // Now sourceNode is the MObject of the connected source node (the mesh node).
    // To get the MDagPath of the mesh node, use MFnDagNode.
    MFnDagNode sourceDagNode(sourceNode);
    MDagPath sourceDagPath;
    sourceDagNode.getPath(sourceDagPath);
    outMesh.setObject(sourceDagPath);
    MBoundingBox bbox = outMesh.boundingBox();
    MGlobal::displayInfo(MString("bbox: ") + bbox.min().x + " " + bbox.min().y + " " + bbox.min().z + " " + bbox.max().x + " " + bbox.max().y + " " + bbox.max().z);
    
    return MStatus::kSuccess; 
}


// Returns the bounding box of the mesh,
// due to the fact that the MObject have no DAG information,
// no dag path can returns no bounding box.
// we need to get the DAG path of the mesh node first by traversing the input plug.
MBoundingBox IntersectionMarkerNode::getBoundingBox(const MObject &meshObject) const
{
    MFnMesh fnMesh;

    // TODO: check the MSatus
    getInputDagMesh(meshObject, fnMesh);
    MBoundingBox bbox = fnMesh.boundingBox();
    MGlobal::displayInfo(MString("bbox: ") + bbox.min().x + " " + bbox.min().y + " " + bbox.min().z + " " + bbox.max().x + " " + bbox.max().y + " " + bbox.max().z);

    return bbox;
}
