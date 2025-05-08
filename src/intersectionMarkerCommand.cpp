/**
    Copyright (c) 2023 Takayoshi Matsumoto
    You may use, distribute, or modify this code under the terms of the MIT license.
*/
#pragma once
#define NO_CUDA

#include "intersectionMarkerNode.h"
#include "intersectionMarkerCommand.h"

#include <string>

#include <maya/MDagPath.h>
#include <maya/MString.h>
#include <maya/MSyntax.h>
#include <maya/MArgDatabase.h>
#include <maya/MArgList.h>
#include <maya/MSelectionList.h>
#include <maya/MDGModifier.h>
#include <maya/MDagModifier.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MFnDagNode.h>
#include <maya/MFnAttribute.h>
#include <maya/MGlobal.h>

#include <maya/MFnMesh.h>
#include <maya/MPlug.h>


IntersectionMarkerCommand::IntersectionMarkerCommand()  {
    markerNode = MObject::kNullObj;
}

IntersectionMarkerCommand::~IntersectionMarkerCommand() {}


void* IntersectionMarkerCommand::creator()
{
    return new IntersectionMarkerCommand();
}


MSyntax IntersectionMarkerCommand::getSyntax()
{
    MSyntax syntax;

    syntax.setObjectType(MSyntax::kSelectionList, 2, 2);
    syntax.useSelectionAsDefault(true);

    syntax.enableQuery(false);
    syntax.enableEdit(false);

    return syntax;
}


MStatus IntersectionMarkerCommand::doIt(const MArgList& argList)
{
    // MGlobal::displayInfo("IntersectionMarkerCommand::doIt");
    MStatus status;
    MArgDatabase argsData(syntax(), argList);

    MSelectionList selection;
    argsData.getObjects(selection);

    status = selection.getDagPath(0, this->meshA);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    status = selection.getDagPath(1, this->meshB);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    if (!meshA.hasFn(MFn::Type::kMesh) || !meshB.hasFn(MFn::Type::kMesh))
    {
        MGlobal::displayError("Must select a mesh.");
        return MStatus::kFailure;
    }

    MDGModifier dgMod;
    MDagModifier dagMod;

    this->xformNode = dagMod.createNode(IntersectionMarkerNode::NODE_ID, MObject::kNullObj, &status);
    if (!status)
    {
        MString errorMsg = status.errorString();
        MGlobal::displayError(errorMsg);
        return MStatus::kFailure;
    }
    MFnDependencyNode(this->xformNode).setName("intersectionMarkerXform");
    status = dagMod.doIt();
    this->markerNode = MFnDagNode(this->xformNode).child(0, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    // input meshes
    MFnMesh meshFnA(meshA);
    MFnMesh meshFnB(meshB);
    MPlug meshAPlug = MFnDependencyNode(meshFnA.object()).findPlug("outMesh", false, &status);
    MPlug meshBPlug = MFnDependencyNode(meshFnB.object()).findPlug("outMesh", false, &status);

    MFnDependencyNode markerNodeFn(this->markerNode);
    MPlug markerInMeshAPlug = markerNodeFn.findPlug("inMeshA", false, &status);
    MPlug markerInMeshBPlug = markerNodeFn.findPlug("inMeshB", false, &status);

    dgMod.connect(meshAPlug, markerInMeshAPlug);
    dgMod.connect(meshBPlug, markerInMeshBPlug);

    // smooth meshes Subdivision
    MPlug smoothMeshAPlug = MFnDependencyNode(meshFnA.object()).findPlug("outSmoothMesh", false, &status);
    MPlug smoothMeshBPlug = MFnDependencyNode(meshFnB.object()).findPlug("outSmoothMesh", false, &status);
    MPlug smoothModeAPlug = MFnDependencyNode(meshFnA.object()).findPlug("displaySmoothMesh", false, &status);
    MPlug smoothModeBPlug = MFnDependencyNode(meshFnB.object()).findPlug("displaySmoothMesh", false, &status);
    MPlug smoothLevelAPlug = MFnDependencyNode(meshFnA.object()).findPlug("smoothLevel", false, &status);
    MPlug smoothLevelBPlug = MFnDependencyNode(meshFnB.object()).findPlug("smoothLevel", false, &status);

    MPlug markerInSmoothMeshAPlug = markerNodeFn.findPlug("inSmoothMeshA", false, &status);
    MPlug markerInSmoothMeshBPlug = markerNodeFn.findPlug("inSmoothMeshB", false, &status);
    MPlug markerSmoothModeAPlug = markerNodeFn.findPlug("smoothModeA", false, &status);
    MPlug markerSmoothModeBPlug = markerNodeFn.findPlug("smoothModeB", false, &status);
    MPlug markerSmoothLevelAPlug = markerNodeFn.findPlug("smoothLevelA", false, &status);
    MPlug markerSmoothLevelBPlug = markerNodeFn.findPlug("smoothLevelB", false, &status);

    dgMod.connect(smoothMeshAPlug, markerInSmoothMeshAPlug);
    dgMod.connect(smoothMeshBPlug, markerInSmoothMeshBPlug);
    dgMod.connect(smoothModeAPlug, markerSmoothModeAPlug);
    dgMod.connect(smoothModeBPlug, markerSmoothModeBPlug);
    dgMod.connect(smoothLevelAPlug, markerSmoothLevelAPlug);
    dgMod.connect(smoothLevelBPlug, markerSmoothLevelBPlug);
    status = dgMod.doIt();
    CHECK_MSTATUS_AND_RETURN_IT(status);

    // offset matrix
    MPlug meshAMatrixPlug = MFnDependencyNode(meshFnA.parent(0)).findPlug("matrix", false, &status);
    MPlug meshBMatrixPlug = MFnDependencyNode(meshFnB.parent(0)).findPlug("matrix", false, &status);

    MPlug markerOffsetAPlug = markerNodeFn.findPlug("offsetMatrixA", false, &status);
    MPlug markerOffsetBPlug = markerNodeFn.findPlug("offsetMatrixB", false, &status);

    dgMod.connect(meshAMatrixPlug, markerOffsetAPlug);
    dgMod.connect(meshBMatrixPlug, markerOffsetBPlug);
    status = dgMod.doIt();

    CHECK_MSTATUS_AND_RETURN_IT(status);

    return status;
}

MStatus IntersectionMarkerCommand::redoIt()
{
    // TODO: Implement this function.
    // unsigned long checksum = MeshData::getVertexChecksum(this->meshA);
    // this->setResult((int) checksum);

    return MStatus::kSuccess;
}


MStatus IntersectionMarkerCommand::undoIt()
{
    MStatus status;

    MDGModifier dgModifier;

    dgModifier.deleteNode(this->markerNode);
    dgModifier.deleteNode(this->xformNode);

    status = dgModifier.doIt();
    CHECK_MSTATUS_AND_RETURN_IT(status);

    return MStatus::kSuccess;
}


bool IntersectionMarkerCommand::isUndoable() const
{
    return !this->markerNode.isNull();
}
