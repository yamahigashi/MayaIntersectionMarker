/**
    Copyright (c) 2023 Takayoshi Matsumoto
    You may use, distribute, or modify this code under the terms of the MIT license.
*/
#pragma once

#include <maya/MDagPath.h>
#include <maya/MPxCommand.h>
#include <maya/MSyntax.h>
#include <maya/MStatus.h>


class IntersectionMarkerCommand : public MPxCommand
{
public:
                        IntersectionMarkerCommand();
                        ~IntersectionMarkerCommand() override;

    static void*        creator();
    static MSyntax      getSyntax();

    virtual MStatus     doIt(const MArgList& argList);
    virtual MStatus     redoIt();

    virtual bool        isUndoable() const { return true; }
    virtual bool        hasSyntax()  const { return true; }
    
public:
    static MString      COMMAND_NAME;

private:    
    MDagPath            meshA;
    MDagPath            meshB;
};
