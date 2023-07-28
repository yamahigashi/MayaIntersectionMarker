/**
    Copyright (c) 2023 Takayoshi Matsumoto
    You may use, distribute, or modify this code under the terms of the MIT license.
*/
#include <maya/MFnPlugin.h>
#include <maya/MObject.h>


#include <maya/MFnPlugin.h>
#include <maya/MTypeId.h>
#include <maya/MString.h>
#include <maya/MStatus.h>
#include <maya/MDrawRegistry.h>
#include <maya/MEventMessage.h>

#include "intersectionMarkerNode.h"
#include "intersectionMarkerCommand.h"
#include "intersectionMarkerDrawOverride.h"


const char* kAUTHOR = "Takayoshi Matsumoto";
const char* kVERSION = "0.0.1";
const char* kREQUIRED_API_VERSION = "Any";

MString IntersectionMarkerCommand::COMMAND_NAME      = "intersectionMarker";
MString IntersectionMarkerNode::NODE_NAME            = "intersectionMarker";
MTypeId IntersectionMarkerNode::NODE_ID              = 0x0012a540;
MString IntersectionMarkerNode::drawDbClassification = "drawdb/geometry/intersectionMarkerDrawOverrider";
MString IntersectionMarkerNode::drawRegistrantId     = "intersectionMarkerDrawOverrider";

#define REGISTER_COMMAND(CMD) \
    do { \
        MStatus status = fnPlugin.registerCommand( \
                CMD::COMMAND_NAME, \
                CMD::creator, \
                CMD::getSyntax); \
        CHECK_MSTATUS_AND_RETURN_IT(status); \
    } while (0)

#define DEREGISTER_COMMAND(CMD) \
    do { \
        MStatus status = fnPlugin.deregisterCommand(CMD::COMMAND_NAME); \
        CHECK_MSTATUS_AND_RETURN_IT(status); \
    } while (0)


#define REGISTER_CONTEXT_COMMAND(CTX, CMD) \
    do { \
        MStatus status = fnPlugin.registerContextCommand( \
                CTX::COMMAND_NAME, \
                CTX::creator, \
                CMD::COMMAND_NAME, \
                CMD::creator, \
                CMD::getSyntax); \
        CHECK_MSTATUS_AND_RETURN_IT(status); \
    } while (0)

#define DEREGISTER_CONTEXT_COMMAND(CTX, CMD) \
    do { \
        MStatus = fnPlugin.deregisterCommand( \
            CTX::COMMAND_NAME, \
            CMD::COMMAND_NAME); \
        CHECK_MSTATUS_AND_RETURN_IT(status); \
    } while (0)

#define REGISTER_NODE(NODE) \
    do { \
        MStatus status = fnPlugin.registerNode( \
            NODE::NODE_NAME, \
            NODE::NODE_ID, \
            NODE::creator, \
            NODE::initialize, \
            MPxNode::kDependNode, \
            &NODE::drawDbClassification); \
        CHECK_MSTATUS_AND_RETURN_IT(status); \
    } while (0)

#define REGISTER_LOCATOR_NODE(NODE) \
    do { \
        MStatus status = fnPlugin.registerNode( \
            NODE::NODE_NAME, \
            NODE::NODE_ID, \
            &NODE::creator, \
            &NODE::initialize, \
            MPxNode::kLocatorNode, \
            &NODE::drawDbClassification); \
        CHECK_MSTATUS_AND_RETURN_IT(status); \
    } while (0)

#define DEREGISTER_NODE(NODE) \
   do { \
       MStatus status = fnPlugin.deregisterNode(NODE::NODE_ID); \
        CHECK_MSTATUS_AND_RETURN_IT(status); \
    } while (0)

#define REGISTER_DRAW_OVERRIDE(NODE, OVERRIDE) \
    do { \
        MStatus status = MHWRender::MDrawRegistry::registerDrawOverrideCreator( \
            NODE::drawDbClassification, \
            NODE::drawRegistrantId, \
            OVERRIDE::Creator); \
        CHECK_MSTATUS_AND_RETURN_IT(status); \
    } while (0)

#define DEREGISTER_DRAW_OVERRIDE(NODE, OVERRIDE) \
    do { \
        MStatus status = MHWRender::MDrawRegistry::deregisterDrawOverrideCreator( \
            NODE::drawDbClassification, \
            NODE::drawRegistrantId); \
        CHECK_MSTATUS_AND_RETURN_IT(status); \
    } while (0)                        


bool menuCreated = false;

MStatus initializePlugin(MObject obj)
{
    MStatus status;
    MFnPlugin fnPlugin(obj, kAUTHOR, kVERSION, kREQUIRED_API_VERSION);
	  REGISTER_LOCATOR_NODE(IntersectionMarkerNode);
    REGISTER_DRAW_OVERRIDE(IntersectionMarkerNode, IntersectionMarkerDrawOverride);
    REGISTER_COMMAND(IntersectionMarkerCommand);

    return MS::kSuccess;
}


MStatus uninitializePlugin(MObject obj)
{
    MStatus status;
    MFnPlugin fnPlugin(obj, kAUTHOR, kVERSION, kREQUIRED_API_VERSION);
    DEREGISTER_COMMAND(IntersectionMarkerCommand);
    DEREGISTER_DRAW_OVERRIDE(IntersectionMarkerNode, IntersectionMarkerDrawOverride);
	  DEREGISTER_NODE(IntersectionMarkerNode);

    return MS::kSuccess;
}
