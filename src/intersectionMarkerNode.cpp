/**
    Copyright (c) 2023 Takayoshi Matsumoto
    You may use, distribute, or modify this code under the terms of the MIT license.
*/

#include "utility.h"
#include "intersectionMarkerNode.h"
#include "KDTreeKernel.h"
#include "OctreeKernel.h"

#include <string>
#include <unordered_set>

#include <maya/MDagPath.h>
#include <maya/MDataBlock.h>
#include <maya/MFnData.h>
#include <maya/MIntArray.h>
#include <maya/MItMeshPolygon.h>
#include <maya/MPointArray.h>
#include <maya/MFnIntArrayData.h>
// #include <maya/MFnArrayAttrsData.h>
// #include <maya/MArrayDataBuilder.h>
// #include <maya/MArrayDataHandle.h>
#include <maya/MFnMesh.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MFnNumericData.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MGlobal.h>
#include <maya/MPlug.h>
#include <maya/MPxNode.h>
#include <maya/MObject.h>
#include <maya/MMatrix.h>
#include <maya/MVector.h>
#include <maya/MDGModifier.h>

MObject IntersectionMarkerNode::meshA;
MObject IntersectionMarkerNode::meshB;
MObject IntersectionMarkerNode::offsetMatrix;
MObject IntersectionMarkerNode::restIntersected;

MObject IntersectionMarkerNode::vertexChecksumA;
MObject IntersectionMarkerNode::vertexChecksumB;
MObject IntersectionMarkerNode::kernelType;

MObject IntersectionMarkerNode::outputIntersected;
MObject IntersectionMarkerNode::outMesh;

IntersectionMarkerNode::IntersectionMarkerNode() {}
IntersectionMarkerNode::~IntersectionMarkerNode() {}

void* IntersectionMarkerNode::creator()
{
    return new IntersectionMarkerNode();
}



MStatus IntersectionMarkerNode::initialize()
{
    MStatus status;

    MFnTypedAttribute tOutputAttr;  // means writable output attribute
    MFnTypedAttribute tInputAttr;  // meanst read-only input attribute
    MFnMatrixAttribute mAttr;
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

    // Initialize Offset Matrix
    offsetMatrix = mAttr.create(OFFSET_MATRIX, OFFSET_MATRIX);
    status = addAttribute(offsetMatrix);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    // Initialize Rest Intersected
    restIntersected = nAttr.create(REST_INTERSECTED, REST_INTERSECTED, MFnNumericData::kInt, -1);
    nAttr.setStorable(true);
    nAttr.setKeyable(false);
    nAttr.setWritable(false);
    nAttr.setReadable(true);
    status = addAttribute(restIntersected);
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
    outputIntersected = nAttr.create(OUTPUT_INTERSECTED, OUTPUT_INTERSECTED, MFnNumericData::kBoolean, 0);
    nAttr.setStorable(true);
    nAttr.setKeyable(false);
    nAttr.setWritable(false);
    nAttr.setReadable(true);
    status = addAttribute(outputIntersected);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    // Initialize Output Mesh
    outMesh = tOutputAttr.create(OUT_MESH, OUT_MESH, MFnData::kMesh);
    tOutputAttr.setStorable(false);
    tOutputAttr.setKeyable(false);
    tOutputAttr.setWritable(false);
    tOutputAttr.setReadable(true);
    status = addAttribute(outMesh);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    // Add dependencies
    status = attributeAffects(meshA, vertexChecksumA);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    status = attributeAffects(meshB, vertexChecksumB);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    status = attributeAffects(offsetMatrix, vertexChecksumA);
    status = attributeAffects(offsetMatrix, vertexChecksumB);
    status = attributeAffects(offsetMatrix, outputIntersected);
    status = attributeAffects(meshA, outputIntersected);
    status = attributeAffects(meshB, outputIntersected);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    status = attributeAffects(kernelType, outputIntersected);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    status = attributeAffects(vertexChecksumA, outputIntersected);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    status = attributeAffects(vertexChecksumB, outputIntersected);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    status = attributeAffects(offsetMatrix, outMesh);
    status = attributeAffects(meshA, outMesh);
    status = attributeAffects(meshB, outMesh);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    status = attributeAffects(kernelType, outMesh);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    status = attributeAffects(vertexChecksumA, outMesh);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    status = attributeAffects(vertexChecksumB, outMesh);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    return MS::kSuccess;
}


// The compute function is called when the node is marked dirty in Maya, 
// meaning it has been modified in some way. In this function, we need to
// perform intersection tests and determine where to draw markers based on the results.
MStatus IntersectionMarkerNode::compute(const MPlug &plug, MDataBlock &dataBlock)
{
    MStatus status;
    // MGlobal::displayInfo("Computing...");

    // Check if the plug parameter corresponds to the attribute that this node
    // is supposed to handle
    if (plug != outMesh) {
        return MStatus::kUnknownParameter;
    }

    // Get necessary input data from the dataBlock. This is usually data from
    // the input attributes of the node.
    MDataHandle meshAHandle = dataBlock.inputValue(meshA);
    MDataHandle meshBHandle = dataBlock.inputValue(meshB);

    // Get the MObject of the meshes
    MObject meshAObject = meshAHandle.asMesh();
    MObject meshBObject = meshBHandle.asMesh();
    MMatrix offset = dataBlock.inputValue(offsetMatrix).asMatrix();
    // -------------------------------------------------------------------------------------------

    // update checksums
    // MGlobal::displayInfo("update checksums...");
    int newCheckA = getVertexChecksum(meshAObject);
    int newCheckB = getVertexChecksum(meshBObject);

    MDataHandle vertexChecksumAHandle = dataBlock.outputValue(vertexChecksumA);
    MDataHandle vertexChecksumBHandle = dataBlock.outputValue(vertexChecksumB);

    int checkA = vertexChecksumAHandle.asInt();
    int checkB = vertexChecksumBHandle.asInt();

    // If the checksums are the same, then we don't need to do anything
    // because the meshes have not changed.
    if (checkA == newCheckA && checkB == newCheckB) {
        // return MS::kSuccess;
    }

    vertexChecksumAHandle.set(newCheckA);
    vertexChecksumAHandle.setClean();
    vertexChecksumBHandle.set(newCheckB);
    vertexChecksumBHandle.setClean();

    // Build kernel
    std::unique_ptr<SpatialDivisionKernel> kernel = getActiveKernel();
    MBoundingBox bbox = getBoundingBox(meshA);
    bbox.transformUsing(offset);
    // MGlobal::displayInfo(MString("bbox: ") + bbox.min().x + " " + bbox.min().y + " " + bbox.min().z + " " + bbox.max().x + " " + bbox.max().y + " " + bbox.max().z);
    status = kernel->build(meshAObject, bbox);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    std::unordered_set<int> intersectedVertexIds;
    std::unordered_set<int> intersectedFaceIds;
    status = checkIntersections(meshAObject, meshBObject, std::move(kernel), offset, intersectedVertexIds, intersectedFaceIds);

    // Get output data handle
    MDataHandle outputIntersectedHandle = dataBlock.outputValue(outputIntersected, &status);
    outputIntersectedHandle.set(intersectedFaceIds.size() > 0);
    outputIntersectedHandle.setClean();

    // -------------------------------------------------------------------------------------------
    MDataHandle outputMeshHandle = dataBlock.outputValue(outMesh, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    outputMeshHandle.set(meshAObject);
    MFnMesh outputMeshFn(outputMeshHandle.asMesh());

    // -------------------------------------------------------------------------------------------
    // Output mesh
    // extrude the intersected faces
    MIntArray faceList;
    for (const auto &id : intersectedFaceIds) {
        offsetPolygon(outputMeshFn, id, 0.01f);
    }

    // MGlobal::displayInfo(MString("Size of intersectedFaceIds is ") + std::to_wstring(intersectedFaceIds.size()).c_str());
    // MGlobal::displayInfo(MString("Size of intersectedVertexIds is ") + std::to_wstring(intersectedVertexIds.size()).c_str());
    // for (const auto &vertexId : intersectedVertexIds) {
    //     MGlobal::displayInfo(MString("intersected vertex: ") + std::to_wstring(vertexId).c_str());
    // }

    // Create a set of all polygon indices
    int numVertices = outputMeshFn.numVertices();
    std::unordered_set<int> vertexIds;
    for (int i = 0; i < numVertices; ++i) {
        vertexIds.insert(i);
    }

    for (const auto &id : intersectedVertexIds) {
        vertexIds.erase(id);
    }

    MPoint center;  // center of the intersected vertices
    MPoint point;
    for (const auto &id : intersectedVertexIds) {
        outputMeshFn.getPoint(id, point);
        center += point;
    }
    center = center / static_cast<double>(intersectedVertexIds.size());

    // For each non-intersected polygon
    for (const auto &index : vertexIds) {
        outputMeshFn.setPoint(index, center);
    }
    // -------------------------------------------------------------------------------------------

    // Mark the output data handle as clean
    outputMeshHandle.setClean();

    meshAHandle.setClean();
    meshBHandle.setClean();
    outputIntersectedHandle.setClean();
    dataBlock.setClean(plug);

    return MStatus::kSuccess;
}


MStatus IntersectionMarkerNode::checkIntersections(MObject &meshAObject, MObject &meshBObject, std::unique_ptr<SpatialDivisionKernel> kernel, MMatrix offset, std::unordered_set<int> &intersectedVertexIds, std::unordered_set<int> &intersectedFaceIds) const
{
    MStatus status;
    TriangleData triangle;
    // MGlobal::displayInfo("checkIntersections...");

    // Iterate through the polygons in meshB
    MItMeshPolygon itPolyA(meshAObject);
    MItMeshPolygon itPolyB(meshBObject);
    MPointArray vertices;
    MIntArray vertexIndices;
    int numPolygons = itPolyB.count();
    for (int i = 0; i < numPolygons; i++) {
        int prevIndex;
        status = itPolyB.setIndex(i, prevIndex);
        CHECK_MSTATUS_AND_RETURN_IT(status);

        int numTriangles;
        itPolyB.numTriangles(numTriangles);

        for (int j = 0; j < numTriangles; ++j) {
            // Get the vertices of the triangle
            itPolyB.getTriangle(j, vertices, vertexIndices, MSpace::kWorld);
            // MGlobal::displayInfo(MString("check triangle: ") + std::to_wstring(i).c_str() + " " + std::to_wstring(j).c_str());

            triangle.vertices[0] = vertices[0];
            triangle.vertices[1] = vertices[1];
            triangle.vertices[2] = vertices[2];

            // Check intersection between triangle and the octree (kernel)
            std::vector<TriangleData> intersectedTriangles = kernel->queryIntersected(triangle);

            // If there is any intersection, store the intersection data into intersectedVertexIds
            bool hit = false;
            if (!intersectedTriangles.empty()) {
                // MGlobal::displayInfo(MString("need to check intersection more closely...") + std::to_wstring(intersectedTriangles.size()).c_str());
                TriangleData intersectedTriangle;
                for(int k = 0; k < intersectedTriangles.size(); ++k) {
                    intersectedTriangle = intersectedTriangles[k];
                    hit = checkIntersectionsDetailed(intersectedTriangle, triangle);
                    if (hit) {
                        intersectedFaceIds.insert(intersectedTriangle.faceIndex);
                        // break;  // To gather all intersected triangles, don't break here
                    }
                }
            }
        }
    }

    for (const auto &faceId : intersectedFaceIds) {
        int prevIndex;
        status = itPolyA.setIndex(faceId, prevIndex);
        CHECK_MSTATUS_AND_RETURN_IT(status);
        itPolyA.getVertices(vertexIndices);
        for (unsigned int i = 0; i < vertexIndices.length(); ++i) {
            intersectedVertexIds.insert(vertexIndices[i]);
        }
    }
    return MStatus::kSuccess;
}


/// Check if the triangle intersects with the plane
bool IntersectionMarkerNode::checkIntersectionsDetailed(const TriangleData triA, const TriangleData triB) const
{

    MVector planeNormal = computePlaneNormal(triA.vertices[0], triA.vertices[1], triA.vertices[2]);
    MVector planeOrigin = computePlaneOrigin(triA.vertices[0], triA.vertices[1], triA.vertices[2]);

    bool resA = checkEdgePlaneIntersections(triB, planeNormal, planeOrigin);
    if (!resA) {
        return false;
    }

    planeNormal = computePlaneNormal(triB.vertices[0], triB.vertices[1], triB.vertices[2]);
    planeOrigin = computePlaneOrigin(triB.vertices[0], triB.vertices[1], triB.vertices[2]);

    bool resB = checkEdgePlaneIntersections(triA, planeNormal, planeOrigin);
    if (!resB) {
        return false;
    }

    return true;
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

    return bbox;
}
