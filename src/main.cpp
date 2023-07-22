/**
    Copyright (c) 2023 Takayoshi Matsumoto
    You may use, distribute, or modify this code under the terms of the MIT license.
*/
#include <maya/MFnPlugin.h>
#include <maya/MObject.h>


#include <maya/MFnPlugin.h>
#include <maya/MGlobal.h>
#include <maya/MTypeId.h>
#include <maya/MString.h>
#include <maya/MStatus.h>

#include "intersectionMarkerNode.h"


const char* kAUTHOR = "Takayoshi Matsumoto";
const char* kVERSION = "0.0.1";
const char* kREQUIRED_API_VERSION = "Any";

MString IntersectionMarkerContextCmd::COMMAND_NAME = "polySymmetryCtx";
MString IntersectionMarkerCommand::COMMAND_NAME    = "polySymmetry";

MString IntersectionMarkerNode::NODE_NAME          = "polySymmetryData";
MTypeId IntersectionMarkerNode::NODE_ID            = 0x0012a540;

#define REGISTER_COMMAND(CMD) CHECK_MSTATUS_AND_RETURN_IT(fnPlugin.registerCommand(CMD::COMMAND_NAME, CMD::creator, CMD::getSyntax));
#define DEREGISTER_COMMAND(CMD) CHECK_MSTATUS_AND_RETURN_IT(fnPlugin.deregisterCommand(CMD::COMMAND_NAME))

#define REGISTER_NODE(NODE) \
    do { \
        MStatus status = fnPlugin.registerNode(NODE::NODE_NAME, NODE::NODE_ID, NODE::creator, NODE::initialize, MPxNode::kDependNode); \
        CHECK_MSTATUS_AND_RETURN_IT(status); \
    } while (0)

#define DEREGISTER_NODE(NODE) \
   do { \
       MStatus status = fnPlugin.deregisterNode(NODE::NODE_ID);
        CHECK_MSTATUS_AND_RETURN_IT(status); \
    } while (0)
                        


bool menuCreated = false;

MStatus initializePlugin(MObject obj)
{
    MStatus status;
    MFnPlugin fnPlugin(obj, kAUTHOR, kVERSION, kREQUIRED_API_VERSION);
	  REGISTER_NODE(IntersectionMarkerNode)

    return MS::kSuccess;
}


MStatus uninitializePlugin(MObject obj)
{
    MStatus status;
    MFnPlugin fnPlugin(obj, kAUTHOR, kVERSION, kREQUIRED_API_VERSION);

	  DEREGISTER_NODE(IntersectionMarkerNode)

    return MS::kSuccess;
}
