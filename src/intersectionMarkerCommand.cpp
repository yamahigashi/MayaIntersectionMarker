/**
    Copyright (c) 2023 Takayoshi Matsumoto
    You may use, distribute, or modify this code under the terms of the MIT license.
*/
#pragma once

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
    MGlobal::displayInfo("IntersectionMarkerCommand::doIt");
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
    status = dagMod.doIt();
    this->markerNode = MFnDagNode(this->xformNode).child(0, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    // input meshes
    MFnMesh meshFnA(meshA);
    MFnMesh meshFnB(meshB);
    MPlug meshAPlug = MFnDependencyNode(meshFnA.object()).findPlug("outMesh", false, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    MPlug meshBPlug = MFnDependencyNode(meshFnB.object()).findPlug("outMesh", false, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    MFnDependencyNode markerNodeFn(this->markerNode);
    MPlug markerInMeshAPlug = markerNodeFn.findPlug("inMeshA", false, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    MPlug markerInMeshBPlug = markerNodeFn.findPlug("inMeshB", false, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    dgMod.connect(meshAPlug, markerInMeshAPlug);
    status = dgMod.doIt();
    CHECK_MSTATUS_AND_RETURN_IT(status);
    dgMod.connect(meshBPlug, markerInMeshBPlug);
    status = dgMod.doIt();
    CHECK_MSTATUS_AND_RETURN_IT(status);
    // MGlobal::displayInfo("plug connected");

    // offset matrix
    MPlug meshAMatrixPlug = MFnDependencyNode(meshFnA.parent(0)).findPlug("matrix", false, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    MPlug meshBMatrixPlug = MFnDependencyNode(meshFnB.parent(0)).findPlug("matrix", false, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    MPlug markerOffsetAPlug = markerNodeFn.findPlug("offsetMatrixA", false, &status);
    MPlug markerOffsetBPlug = markerNodeFn.findPlug("offsetMatrixB", false, &status);

    dgMod.connect(meshAMatrixPlug, markerOffsetAPlug);
    status = dgMod.doIt();
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
