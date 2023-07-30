/**
    Copyright (c) 2023 Takayoshi Matsumoto
    You may use, distribute, or modify this code under the terms of the MIT license.
*/
#pragma once

#include "SpatialDivisionKernel.h"
#include "IntersectionMarkerData.h"

#include <string>
#include <vector>
#include <unordered_set>

#include <maya/MGlobal.h>
#include <maya/MDagPath.h>
#include <maya/MDataHandle.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MObject.h>
#include <maya/MPlug.h>
#include <maya/MString.h>
#include <maya/MTypeId.h>
#include <maya/MMessage.h>

// Viewport 2.0
#include <maya/M3dView.h>
#include <maya/MDrawRegistry.h>
#include <maya/MPxDrawOverride.h>
#include <maya/MUserData.h>
#include <maya/MDrawContext.h>
#include <maya/MHWGeometryUtilities.h>



class IntersectionMarkerDrawOverride : public MHWRender::MPxDrawOverride
{
public:

    static MHWRender::MPxDrawOverride* Creator(const MObject& obj)
    {
        return new IntersectionMarkerDrawOverride(obj);
    }
    ~IntersectionMarkerDrawOverride() override;

    MHWRender::DrawAPI supportedDrawAPIs() const override;

    MUserData* prepareForDraw(
        const MDagPath& objPath,
        const MDagPath& cameraPath,
        const MHWRender::MFrameContext& frameContext,
        MUserData* oldData) override;

    void addUIDrawables(
        const MDagPath& objPath,
        MHWRender::MUIDrawManager& drawManager,
        const MHWRender::MFrameContext& frameContext,
        const MUserData* data) override;

    bool hasUIDrawables() const override { return true; }
    // virtual bool isBounded(const MDagPath& objPath, const MDagPath& cameraPath) const;
    // virtual MBoundingBox boundingBox(const MDagPath& objPath, const MDagPath& cameraPath) const;

public:
    MObject     fNode;
    MCallbackId fModelEditorChangedCbId;

protected:
private:
    IntersectionMarkerDrawOverride(const MObject& obj);
    // class IntersectionMarkerData : public MUserData {...};
    static void OnModelEditorChanged(void *clientData);
    MStatus addIntersectedVertices(
            const MFnMesh& meshFn,
            IntersectionMarkerData* data,
            const std::unordered_set<int> &intersectedFaceIds,
            const MMatrix& offsetMatrix
    );

    int prevChecksum = -1;
};
