/**
    Copyright (c) 2023 Takayoshi Matsumoto
    You may use, distribute, or modify this code under the terms of the MIT license.
*/

#include "IntersectionMarkerNode.h"
#include "IntersectionMarkerData.h"
#include "IntersectionMarkerDrawOverride.h"

#include <omp.h>
#include <string>
#include <unordered_set>

#include <maya/MGlobal.h>
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

    // Reset face data
    data->faces.clear();

    prevChecksum = newChecksum;

    MFnMesh meshAFn;
    status = node->getInputDagMesh(node->meshA, meshAFn);
    CHECK_MSTATUS_AND_RETURN_DATA("prepareForDraw: meshAFn is null");

    MFnMesh meshBFn;
    status = node->getInputDagMesh(node->meshB, meshBFn);
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
    }

    if (showMeshB) {
        addIntersectedVertices(meshBFn, data, node->intersectedFaceIdsB, outMatrixB);
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

    MItMeshPolygon itPoly(meshFn.object(), &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    MIntArray vertexIndices;
    MPointArray vertices;
    int prevIndex;

    int numPolygons = meshFn.numPolygons();
    for (const auto &faceId : intersectedFaceIds) {
        status = itPoly.setIndex(faceId, prevIndex);
        CHECK_MSTATUS_AND_RETURN_IT(status);
        IntersectionMarkerData::FaceData faceData;

        MVector normal;
        meshFn.getPolygonNormal(faceId, normal);
        int numTriangles;
        itPoly.numTriangles(numTriangles);
        for (int j = 0; j < numTriangles; ++j) {
            itPoly.getTriangle(j, vertices, vertexIndices, MSpace::kObject);
            for (unsigned int k = 0; k < vertices.length(); ++k) {
                // To avoid z-fighting, move the vertex a little bit along the normal
                vertices[k] += normal * 0.001;
                vertices[k] *= offsetMatrix;
                faceData.vertices.append(vertices[k]);
                faceData.edges.append(vertices[k]);
                faceData.normals.append(normal);
            }
            faceData.edges.append(vertices[0]);  // close the loop of the triangle
        }

        // Get normal
        data->faces.push_back(faceData);
    }

    return status;
}

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
