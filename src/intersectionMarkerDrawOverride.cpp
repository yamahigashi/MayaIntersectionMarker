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


IntersectionMarkerDrawOverride::IntersectionMarkerDrawOverride(const MObject& obj)
    : MHWRender::MPxDrawOverride(obj, NULL, /* isAlwaysDirty=*/false)
    , fNode(obj)
{
    // MGlobal::displayInfo("IntersectionMarkerDrawOverride::IntersectionMarkerDrawOverride");
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

    // MGlobal::displayInfo("prepareForDraw");
    IntersectionMarkerData* data = dynamic_cast<IntersectionMarkerData*>(oldData);
    if (!data) {
        data = new IntersectionMarkerData();
    }

    // Reset face data
    data->faces.clear();

    MObject drawNode = objPath.node(&status);
    if (!status) 
    {
        MGlobal::displayInfo("prepareForDraw: drawNode is null");
        return data;
    }

    // Get the IntersectionMarkerNode
    MFnDependencyNode depNodeFn(drawNode, &status);
    if (!status) 
    {
        MGlobal::displayInfo("prepareForDraw: depNodeFn is null");
        return data;
    }

    // Get the node and mesh data
    const IntersectionMarkerNode* node = (const IntersectionMarkerNode*)depNodeFn.userNode();
    if (!node) {
        MGlobal::displayInfo(MString("prepareForDraw: locnode is null: real type: ") + depNodeFn.typeName());
        return data;
    }

    MFnMesh meshAFn;
    MFnMesh meshBFn;
    status = node->getInputDagMesh(node->meshA, meshAFn);
    if (!status) {
        MGlobal::displayInfo("prepareForDraw: meshAFn is null");
        return data;
    }

    status = node->getInputDagMesh(node->meshB, meshBFn);
    if (!status) {
        MGlobal::displayInfo("prepareForDraw: meshBFn is null");
        return data;
    }

    addIntersectedVertices(meshAFn, data, node->intersectedFaceIdsA);
    addIntersectedVertices(meshBFn, data, node->intersectedFaceIdsB);

    return data;
}


MStatus IntersectionMarkerDrawOverride::addIntersectedVertices(
        const MFnMesh& meshFn,
        IntersectionMarkerData* data,
        const std::unordered_set<int> &intersectedFaceIds
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

        int numTriangles;
        itPoly.numTriangles(numTriangles);
        for (int j = 0; j < numTriangles; ++j) {
            itPoly.getTriangle(j, vertices, vertexIndices, MSpace::kObject);
            for (int k = 0; k < vertices.length(); ++k) {
                faceData.vertices.append(vertices[k]);
            }
        }

        // Get normal
        meshFn.getPolygonNormal(faceId, faceData.normal);
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
    // MGlobal::displayInfo("addUIDrawables");
    // cast the user data back to our to our struct
    const IntersectionMarkerData* markerData = dynamic_cast<const IntersectionMarkerData*>(data);
    if (!markerData) {
        MGlobal::displayInfo("addUIDrawables: markerData is null");
        return;
    }

    drawManager.beginDrawable();
    {
        drawManager.setColor(MColor(1.0f, 0.0f, 0.0f));
        drawManager.setLineWidth(2.0f);
        drawManager.setLineStyle(MUIDrawManager::kSolid);
        double xpos = rand()/RAND_MAX*10.0;
        MPoint position(xpos, 0.0, 0.5 );
        MVector normal(0.0, 0.0, 1.0);
        MVector rectUp(0.0, 1.0, 0.0);
        drawManager.rect(position, rectUp, normal, 5 * 1, 5, false );

        // for each face
        for (const IntersectionMarkerData::FaceData& face : markerData->faces) {
            // draw the face
            drawManager.mesh(MHWRender::MUIDrawManager::kTriangles, face.vertices);
        }
    }
    drawManager.endDrawable();

}
