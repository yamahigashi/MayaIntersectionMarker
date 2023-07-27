/**
    Copyright (c) 2023 Takayoshi Matsumoto
    You may use, distribute, or modify this code under the terms of the MIT license.
*/

#include "utility.h"
#include "intersectionMarkerNode.h"
#include "KDTreeKernel.h"
#include "EmbreeKernel.h"
#include "OctreeKernel.h"

#include <omp.h>
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
#include <maya/MPxLocatorNode.h>
#include <maya/MObject.h>
#include <maya/MMatrix.h>
#include <maya/MVector.h>
#include <maya/MDGModifier.h>
#include <maya/MEvaluationNode.h>


MObject IntersectionMarkerNode::meshA;
MObject IntersectionMarkerNode::meshB;
MObject IntersectionMarkerNode::offsetMatrix;
MObject IntersectionMarkerNode::restIntersected;

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
    eAttr.addField("Embree", 2);
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

    return MS::kSuccess;
}


MStatus IntersectionMarkerNode::postEvaluation( const  MDGContext& context, const MEvaluationNode& evaluationNode, PostEvaluationType evalType )
{
    MGlobal::displayInfo("Post Evaluation");
    if(!context.isNormal()) {
        return MStatus::kFailure;
    }

    MStatus status;
    if(evalType == kLeaveDirty)
    {
    }
    else if ( (evaluationNode.dirtyPlugExists(meshA, &status) && status ) || 
              ( evaluationNode.dirtyPlugExists(meshB, &status) && status ) )
    {
        MDataBlock block = forceCache();
        MDataHandle meshAHandle = block.inputValue(meshA, &status);
        MDataHandle meshBHandle = block.inputValue(meshB, &status);
    }
   return MStatus::kSuccess;
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
    if (plug != outputIntersected) {
        // return MStatus::kUnknownParameter;
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

    status = checkIntersections(meshAObject, meshBObject, std::move(kernel), offset);

    // Get output data handle
    MDataHandle outputIntersectedHandle = dataBlock.outputValue(outputIntersected, &status);
    outputIntersectedHandle.set((intersectedFaceIdsA.size() > 0) || (intersectedFaceIdsB.size() > 0));
    outputIntersectedHandle.setClean();

    // -------------------------------------------------------------------------------------------

    meshAHandle.setClean();
    meshBHandle.setClean();
    outputIntersectedHandle.setClean();
    dataBlock.setClean(plug);

    return MStatus::kSuccess;
}


MStatus IntersectionMarkerNode::checkIntersections(MObject &meshAObject, MObject &meshBObject, std::unique_ptr<SpatialDivisionKernel> kernel, MMatrix offset)
{
    MStatus status;
    // MGlobal::displayInfo("checkIntersections...");
    intersectedFaceIdsA.clear();
    intersectedFaceIdsB.clear();

    // Iterate through the polygons in meshB
    MFnMesh meshBFn(meshBObject);
    int numPolygons = meshBFn.numPolygons();

    MIntArray triangleCounts;   // number of triangles in each face
    MIntArray triangleVertices; // The triangle vertex Ids for each triangle
    MIntArray triangleIndices;  // The index array for each triangle in face vertex space
    MPointArray vertexPositions;

    meshBFn.getTriangles(triangleCounts, triangleVertices);
    meshBFn.getTriangleOffsets(triangleCounts, triangleIndices);
    meshBFn.getPoints(vertexPositions, MSpace::kObject);

    // Calculate the offset into the triangleVertices array for each polygon
    MIntArray polygonTriangleOffsets(numPolygons, 0);
    for (int polygonIndex = 1; polygonIndex < numPolygons; polygonIndex++) {
        polygonTriangleOffsets[polygonIndex] = polygonTriangleOffsets[polygonIndex - 1] + triangleCounts[polygonIndex - 1] * 3;
    }

    // Similarly, calculate the offset into the triangleIndices array for each polygon
    MIntArray polygonIndexOffsets(numPolygons, 0);
    for (int polygonIndex = 1; polygonIndex < numPolygons; polygonIndex++) {
        polygonIndexOffsets[polygonIndex] = polygonIndexOffsets[polygonIndex - 1] + triangleCounts[polygonIndex - 1];
    }

    #pragma omp parallel
    {
        std::unordered_set<int> intersectedFaceIdsLocalA;
        std::unordered_set<int> intersectedFaceIdsLocalB;

        #pragma omp for
        for (int polygonIndex = 0; polygonIndex < numPolygons; polygonIndex++) {

            TriangleData triangle;
            int numTrianglesInPolygon = triangleCounts[polygonIndex];
            int triangleVerticesOffset = polygonTriangleOffsets[polygonIndex];
            int triangleIndicesOffset = polygonIndexOffsets[polygonIndex];

            for (int triangleIndex = 0; triangleIndex < numTrianglesInPolygon; triangleIndex++) {
                // Get the vertex positions of each triangle
                for (int vertexIndex = 0; vertexIndex < 3; vertexIndex++) {
                    int vertexId = triangleVertices[triangleVerticesOffset + triangleIndex * 3 + vertexIndex];
                    triangle.vertices[vertexIndex] = vertexPositions[vertexId] * offset;
                }

                // Check intersection between triangle and the octree (kernel)
                std::vector<TriangleData> intersectedTriangles = kernel->queryIntersected(triangle);
                // 
                // If there is any intersection, store the intersection data into intersectedVertexIdsLocal
                bool hit = false;
                if (!intersectedTriangles.empty()) {
                    for(int k = 0; k < intersectedTriangles.size(); ++k) {
                        hit = checkIntersectionsDetailed(intersectedTriangles[k], triangle);
                        if (hit) {
                            int a = intersectedTriangles[k].faceIndex;
                            int b = polygonIndex;

                            // MGlobal::displayInfo(MString("Intersection: ") + a + " " + b);
                            intersectedFaceIdsLocalA.insert(a);
                            intersectedFaceIdsLocalB.insert(b);
                        }
                    }
                }
            }
        }

        #pragma omp critical
        {
            intersectedFaceIdsA.insert(intersectedFaceIdsLocalA.begin(), intersectedFaceIdsLocalA.end());
            intersectedFaceIdsB.insert(intersectedFaceIdsLocalB.begin(), intersectedFaceIdsLocalB.end());
        }
    }

    return MStatus::kSuccess;
}


/// Check if the triangle intersects with the plane
bool IntersectionMarkerNode::checkIntersectionsDetailed(const TriangleData triA, const TriangleData triB) const
{

    MVector planeNormal = computePlaneNormal(triA.vertices[0], triA.vertices[1], triA.vertices[2]);
    MVector planeOrigin = computePlaneOrigin(triA.vertices[0], triA.vertices[1], triA.vertices[2]);

    for (int i = 0; i < 3; ++i) {
        MPoint edgeStart = triB.vertices[i];
        MPoint edgeEnd = triB.vertices[(i+1)%3];

        if (isEdgeIntersectingPlane(planeNormal, planeOrigin, edgeStart, edgeEnd)) {
            MPoint intersection = computeEdgePlaneIntersection(planeNormal, planeOrigin, edgeStart, edgeEnd);
            if (isPointInsideTriangle(intersection, triA)) {
                return true;
            }
        }
    }

    planeNormal = computePlaneNormal(triB.vertices[0], triB.vertices[1], triB.vertices[2]);
    planeOrigin = computePlaneOrigin(triB.vertices[0], triB.vertices[1], triB.vertices[2]);

    for (int i = 0; i < 3; ++i) {
        MPoint edgeStart = triA.vertices[i];
        MPoint edgeEnd = triA.vertices[(i+1)%3];

        if (isEdgeIntersectingPlane(planeNormal, planeOrigin, edgeStart, edgeEnd)) {
            MPoint intersection = computeEdgePlaneIntersection(planeNormal, planeOrigin, edgeStart, edgeEnd);
            if (isPointInsideTriangle(intersection, triB)) {
                return true;
            }
        }
    }

    return false;
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
    case 2: // Embree
        return std::make_unique<EmbreeKernel>();
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
