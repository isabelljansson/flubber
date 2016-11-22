#include "../include/CustomDeformerPlugin.h"

MStatus initializePlugin(MObject obj) {
  MStatus status;
  MFnPlugin plugin(obj, "Ronja, Jonathan, Isabell", "0.1", "Any");
 
  // Make a deformer node
  status = plugin.registerNode("flubber", PushDeformerNode::id, PushDeformerNode::creator,
                               PushDeformerNode::initialize, MPxNode::kDeformerNode);
  CHECK_MSTATUS_AND_RETURN_IT(status);
 
  return status;
}
 
MStatus uninitializePlugin(MObject obj) {
  MStatus     status;
  MFnPlugin plugin(obj);
 
  status = plugin.deregisterNode(PushDeformerNode::id);
  CHECK_MSTATUS_AND_RETURN_IT(status);

  return status;
}