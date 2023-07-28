/**
    Copyright (c) 2023 Takayoshi Matsumoto
    You may use, distribute, or modify this code under the terms of the MIT license.
*/
#pragma once

#include "intersectionMarkerCommand.h"

#include <string>

#include <maya/MDagPath.h>
#include <maya/MString.h>
#include <maya/MSyntax.h>
#include <maya/MArgDatabase.h>
#include <maya/MArgList.h>
#include <maya/MSelectionList.h>
#include <maya/MGlobal.h>


IntersectionMarkerCommand::IntersectionMarkerCommand()  {}
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

    return this->redoIt();
}

MStatus IntersectionMarkerCommand::redoIt()
{
    // TODO: Implement this function.
    // unsigned long checksum = MeshData::getVertexChecksum(this->meshA);
    // this->setResult((int) checksum);

    return MStatus::kSuccess;
}
