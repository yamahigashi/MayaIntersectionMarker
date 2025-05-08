/**
    Copyright (c) 2023 Takayoshi Matsumoto
    You may use, distribute, or modify this code under the terms of the MIT license.
*/
#define NO_CUDA

#include "IntersectionMarkerNode.h"
#include "IntersectionMarkerData.h"
#include "IntersectionMarkerDrawOverride.h"

#include <omp.h>
#include <string>
#include <unordered_set>

#include <maya/MGlobal.h>
#include <maya/MObjectHandle.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MFnMesh.h>
#include <maya/MDagPath.h>
#include <maya/MItMeshPolygon.h>
#include <maya/MEventMessage.h>


#define CHECK_MSTATUS_AND_RETURN_DATA(errorMessage) \
    do { \
        if (!status) { \
            MGlobal::displayInfo(errorMessage); \
            return data; \
        } \
    } while (0)


IntersectionMarkerDrawOverride::IntersectionMarkerDrawOverride(const MObject& obj)
    : MHWRender::MPxDrawOverride(obj, NULL, /* isAlwaysDirty=*/true)
    , fNode(obj)
{
    fModelEditorChangedCbId = MEventMessage::addEventCallback(
        "modelEditorChanged",
        OnModelEditorChanged,
        this
    );
}

IntersectionMarkerDrawOverride::~IntersectionMarkerDrawOverride()
{
    if (fModelEditorChangedCbId != 0)
    {
        MMessage::removeCallback(fModelEditorChangedCbId);
        fModelEditorChangedCbId = 0;
    }
}


void IntersectionMarkerDrawOverride::OnModelEditorChanged(void *clientData)
{
    // Mark the node as being dirty so that it can update on display mode switch,
    // e.g. between wireframe and shaded.
    IntersectionMarkerDrawOverride *ovr = static_cast<IntersectionMarkerDrawOverride*>(clientData);
    if (ovr) {
        MHWRender::MRenderer::setGeometryDrawDirty(ovr->fNode);
    }
}


MHWRender::DrawAPI IntersectionMarkerDrawOverride::supportedDrawAPIs() const
{
    // return (MHWRender::kOpenGL | MHWRender::kDirectX11 | MHWRender::kOpenGLCoreProfile);
    return MHWRender::kAllDevices;
}

MUserData* IntersectionMarkerDrawOverride::prepareForDraw(
    const MDagPath& objPath,
    const MDagPath& cameraPath,
    const MHWRender::MFrameContext& frameContext,
    MUserData* oldData)
{
    MStatus status;

    IntersectionMarkerData* data = dynamic_cast<IntersectionMarkerData*>(oldData);
    if (!data) {
        data = new IntersectionMarkerData();
    }

    MObject drawNode = objPath.node(&status);
    CHECK_MSTATUS_AND_RETURN_DATA("prepareForDraw: objPath.node is null");

    // Get the IntersectionMarkerNode
    MFnDependencyNode depNodeFn(drawNode, &status);
    CHECK_MSTATUS_AND_RETURN_DATA("prepareForDraw: depNodeFn is null");

    // Get the node and mesh data
    const IntersectionMarkerNode* node = (const IntersectionMarkerNode*)depNodeFn.userNode();
    if (!node) {
        return data;
    }

    MObjectHandle handle(drawNode);
    unsigned int nodeId = handle.hashCode();
    int prevChecksum = prevChecksums[nodeId];

    int checkSumA;
    int checkSumB;
    status = node->getChecksumA(checkSumA);
    CHECK_MSTATUS_AND_RETURN_DATA("prepareForDraw: getChecksumA failed");
    status = node->getChecksumB(checkSumB);
    CHECK_MSTATUS_AND_RETURN_DATA("prepareForDraw: getChecksumB failed");
    int newChecksum = checkSumA ^ checkSumB;
    if (newChecksum > 0 && newChecksum == prevChecksum) {
        return data;
    }

    prevChecksums[nodeId] = newChecksum;

    // Reset face data
    data->faces.clear();

    prevChecksum = newChecksum;

    MFnMesh meshAFn;
    int smoothModeA;
    status = node->getSmoothMode(node->smoothModeA, smoothModeA);
    if( smoothModeA == 0 ) {
        status = node->getInputDagMesh(node->meshA, meshAFn);
    } else {
        status = node->getInputDagMesh(node->smoothMeshA, meshAFn);
    }
    CHECK_MSTATUS_AND_RETURN_DATA("prepareForDraw: meshAFn is null");

    MFnMesh meshBFn;
    int smoothModeB;
    status = node->getSmoothMode(node->smoothModeB, smoothModeB);
    if( smoothModeB == 0 ) {
        status = node->getInputDagMesh(node->meshB, meshBFn);
    } else {
        status = node->getInputDagMesh(node->smoothMeshB, meshBFn);
    }
    CHECK_MSTATUS_AND_RETURN_DATA("prepareForDraw: meshBFn is null");

    // Get the offset matrix
    MMatrix outMatrixA;
    MMatrix outMatrixB;
    node->getOffsetMatrix(node->offsetMatrixA, outMatrixA);
    node->getOffsetMatrix(node->offsetMatrixB, outMatrixB);

    MPlug showMeshAPlug = depNodeFn.findPlug("showMeshA", false, &status);
    bool showMeshA = showMeshAPlug.asBool();
    MPlug showMeshBPlug = depNodeFn.findPlug("showMeshB", false, &status);
    bool showMeshB = showMeshBPlug.asBool();

    if (showMeshA) {
        addIntersectedVertices(meshAFn, data, node->intersectedFaceIdsA, outMatrixA);
        // addIntersectedVertices(meshAFn, data, node->intersectedFacesA, outMatrixA);
    }

    if (showMeshB) {
        addIntersectedVertices(meshBFn, data, node->intersectedFaceIdsB, outMatrixB);
        // addIntersectedVertices(meshBFn, data, node->intersectedFacesB, outMatrixB);
    }

    return data;
}


MStatus IntersectionMarkerDrawOverride::addIntersectedVertices(
        const MFnMesh& meshFn,
        IntersectionMarkerData* data,
        const std::unordered_set<int> &intersectedFaceIds,
        const MMatrix &offsetMatrix
) {
    MStatus status;

    CHECK_MSTATUS_AND_RETURN_IT(status);

    MIntArray vertexIndices;
    MPointArray vertices;


    MIntArray triangleCounts;   // number of triangles in each face
    MIntArray triangleVertices; // The triangle vertex Ids for each triangle
    MIntArray triangleIndices;  // The index array for each triangle in face vertex space
    MPointArray vertexPositions;
    MVector normal;

    int numPolygons = meshFn.numPolygons();
    meshFn.getTriangles(triangleCounts, triangleVertices);
    meshFn.getTriangleOffsets(triangleCounts, triangleIndices);
    meshFn.getPoints(vertexPositions, MSpace::kObject);  // not having DAG path, so use object space

    // Calculate the offset into the triangleVertices array for each polygon
    MIntArray polygonTriangleOffsets(numPolygons, 0);
    MIntArray polygonIndexOffsets(numPolygons, 0);
    for (int polygonIndex = 1; polygonIndex < numPolygons; polygonIndex++) {
        int previousTriangles = triangleCounts[polygonIndex - 1];
        polygonTriangleOffsets[polygonIndex] = polygonTriangleOffsets[polygonIndex - 1] + previousTriangles * 3;
        polygonIndexOffsets[polygonIndex] = polygonIndexOffsets[polygonIndex - 1] + previousTriangles;
    }

    for (const auto &faceId : intersectedFaceIds) {
        if( faceId < 0 || faceId >= numPolygons ) {
            MString message = "Face ID out of bounds: " + MString(std::to_string(faceId).c_str());
            MGlobal::displayInfo(message);
            continue;
        }
        TriangleData triangle;
        int numTrianglesInPolygon = triangleCounts[faceId];
        int triangleVerticesOffset = polygonTriangleOffsets[faceId];
        int triangleIndicesOffset = polygonIndexOffsets[faceId];

        CHECK_MSTATUS_AND_RETURN_IT(status);
        IntersectionMarkerData::FaceData faceData;
        meshFn.getPolygonNormal(faceId, normal);
        for (int triangleIndex = 0; triangleIndex < numTrianglesInPolygon; triangleIndex++) {
            // Get the vertex positions of each triangle
            int vertexId0 = triangleVertices[triangleVerticesOffset + triangleIndex * 3 + 0];
            int vertexId1 = triangleVertices[triangleVerticesOffset + triangleIndex * 3 + 1];
            int vertexId2 = triangleVertices[triangleVerticesOffset + triangleIndex * 3 + 2];
            MPoint p0 = vertexPositions[vertexId0] * offsetMatrix;
            MPoint p1 = vertexPositions[vertexId1] * offsetMatrix;
            MPoint p2 = vertexPositions[vertexId2] * offsetMatrix;
            TriangleData triangle(faceId, triangleIndex, p0, p1, p2);

            faceData.vertices.append(p0 + normal * 0.001);
            faceData.vertices.append(p1 + normal * 0.001);
            faceData.vertices.append(p2 + normal * 0.001);
        }
        data->faces.push_back(faceData);
    }

    return status;
}


// MStatus IntersectionMarkerDrawOverride::addIntersectedVertices(
//         const MFnMesh& meshFn,
//         IntersectionMarkerData* data,
//         const std::vector<IntersectionMarkerData::FaceData> &intersectedFaces,
//         const MMatrix &offsetMatrix
// ) {
//     MStatus status;
// 
//     for (const auto &faceData : intersectedFaces) {
//         data->faces.push_back(faceData);
//     }
// 
//     return status;
// }

void IntersectionMarkerDrawOverride::addUIDrawables(
    const MDagPath& objPath,
    MHWRender::MUIDrawManager& drawManager,
    const MHWRender::MFrameContext& frameContext,
    const MUserData* data)
{
    // cast the user data back to our to our struct
    const IntersectionMarkerData* markerData = dynamic_cast<const IntersectionMarkerData*>(data);
    if (!markerData) {
        // MGlobal::displayInfo("addUIDrawables: markerData is null");
        return;
    }

    drawManager.beginDrawable(MHWRender::MUIDrawManager::kNonSelectable);
    {
        // drawManager.setLineWidth(2.0f);
        drawManager.setLineStyle(MUIDrawManager::kSolid);

        // for each face
        for (const IntersectionMarkerData::FaceData& face : markerData->faces) {
            // draw the face
            drawManager.setColor(MColor(1.0f, 0.0f, 0.0f));
            drawManager.mesh(MHWRender::MUIDrawManager::kTriangles, face.vertices);

            // draw the edges
            drawManager.setColor(MColor(0.0f, 0.015f, 0.3764f));
            drawManager.mesh(MHWRender::MUIDrawManager::kLines, face.edges);
        }
    }
    drawManager.endDrawable();

}
