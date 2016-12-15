#include "../include/PushDeformerNode.h"

#define SIGN(i) (i < 0 ? -1 : 1)

MTypeId PushDeformerNode::id(0x00000002);

MObject PushDeformerNode::GravityMagnitude;
MObject PushDeformerNode::GravityDirection;
MObject PushDeformerNode::InitialVelocity;
MObject PushDeformerNode::Mode;

MObject PushDeformerNode::CurrentTime;
MObject PushDeformerNode::Mass;
MObject PushDeformerNode::Stiffness; // stiffness
MObject PushDeformerNode::Bounciness;
MObject PushDeformerNode::Friction;
MObject PushDeformerNode::Deformation; // deformation
MObject PushDeformerNode::Elasticity;

bool PushDeformerNode::initFrame;
ParticleSystem* PushDeformerNode::mesh;
MTime PushDeformerNode::tPrev;
 
void* PushDeformerNode::creator() { return new PushDeformerNode; }

MStatus PushDeformerNode::deform(MDataBlock& data, MItGeometry& it_geo, 
    const MMatrix &local_to_world_matrix, unsigned int m_index) {

    // create ParticleSystem first frame
    MTime tNow = data.inputValue(CurrentTime).asTime();
    if (initFrame || tNow.value() == 1) {
        if (mesh) // clean up, unneeded?
            delete mesh;

        tPrev = data.inputValue(CurrentTime).asTime();
        
        std::vector<glm::dvec3> p0;
        MVector temp = data.inputValue(InitialVelocity).asVector(); // ugly?
        glm::dvec3 v0 = glm::dvec3(temp[0], temp[1], temp[2]);
        for (; !it_geo.isDone(); it_geo.next()) {
            MPoint vertexPos = it_geo.position() * local_to_world_matrix;
            p0.push_back(glm::dvec3(vertexPos.x, vertexPos.y, vertexPos.z));
        }
        mesh = new ParticleSystem(p0, v0);
        
        initFrame = false;
        return MS::kSuccess;
    }
    // Update
    else {
        MStatus status;

        tNow = data.inputValue(CurrentTime).asTime();
        MTime tDiff = tNow - tPrev;
        tPrev = tNow;

        // physics arguments
        MVector temp = data.inputValue(GravityMagnitude).asDouble()
            * data.inputValue(GravityDirection).asVector();
        mesh->gravity = glm::dvec3(temp[0], temp[1], temp[2]);
        temp = data.inputValue(InitialVelocity).asVector();
        mesh->initVel = glm::dvec3(temp[0], temp[1], temp[2]);
        
        mesh->mass = data.inputValue(Mass).asDouble();
        mesh->stiffness = data.inputValue(Stiffness).asDouble();
        mesh->bounciness = data.inputValue(Bounciness).asDouble();
        mesh->friction = data.inputValue(Friction).asDouble();
        mesh->beta = data.inputValue(Deformation).asDouble();
        mesh->elasticity = data.inputValue(Elasticity).asDouble();
        mesh->mode = data.inputValue(Mode).asInt();
        
        // Update the particle systems positions with dynamics simulation and
        // Shape matching
        int updates = tDiff.value();
        int updatesPerTimeStep = 2;
        mesh->dt = 1 / 24.0 / updatesPerTimeStep * SIGN(updates);
        if (mesh) {
          for (int i = 0; i < abs(updates) * updatesPerTimeStep; ++i) {
            mesh->applyForces();
            mesh->deform();
          }
        }
        else
          MGlobal::displayInfo("Mesh isnt created");
        
        // Update output positions
        MMatrix local_to_world_matrix_inv = local_to_world_matrix.inverse();
        for (; !it_geo.isDone(); it_geo.next()) {
          int idx = it_geo.index();

          glm::dvec3 p = mesh->getPosition(idx);
          //glm::dvec3 p = glm::dvec3(0.0, 1.0, 0.0);

          // Transform back to model coordinates
          it_geo.setPosition(MPoint(p.x, p.y, p.z) * local_to_world_matrix_inv);
        }
        
        return MS::kSuccess;
    }
}
 
MStatus PushDeformerNode::initialize() {
    initFrame = true;

    MFnTypedAttribute tAttr;
    MFnNumericAttribute nAttr;
    MFnUnitAttribute uAttr;
    MFnEnumAttribute eAttr;

    // Create attributes
    Mode = eAttr.create("Mode", "me", 0);
    eAttr.setStorable(true);
    eAttr.setKeyable(true);
    eAttr.addField("Rigid", 0);
    eAttr.addField("Linear", 1);
    eAttr.addField("Quadratic", 2);


    
    GravityMagnitude = nAttr.create("GravityMagnitude", "gm", MFnNumericData::kDouble, 0.0);
    nAttr.setDefault(9.8);
    nAttr.setMin(0.0);
    nAttr.setMax(10.0);
    nAttr.setChannelBox(true);
    
    GravityDirection = nAttr.create("GravityDirection", "gd", MFnNumericData::k3Double, 0.0);
    nAttr.setDefault(0.0);
    nAttr.setMin(-1.0);
    nAttr.setMax(1.0);
    nAttr.setChannelBox(true);

    CurrentTime = uAttr.create("CurrentTime", "ct", MFnUnitAttribute::kTime, 0.0);
    uAttr.setDefault(MAnimControl::currentTime().as(MTime::kFilm));
    uAttr.setChannelBox(true);

    Mass = nAttr.create("Mass", "ma", MFnNumericData::kDouble, 0.0);
    nAttr.setDefault(1.0);
    nAttr.setMin(0.0);
    nAttr.setMax(10.0);
    nAttr.setChannelBox(true);

    Stiffness = nAttr.create("Stiffness", "st", MFnNumericData::kDouble, 0.0);
    nAttr.setDefault(0.5);
    nAttr.setMin(0.0);
    nAttr.setMax(1.0);
    nAttr.setChannelBox(true);

    Bounciness = nAttr.create("Bounciness", "bo", MFnNumericData::kDouble, 0.0);
    nAttr.setDefault(0.5);
    nAttr.setMin(0.0);
    nAttr.setMax(1.0);
    nAttr.setChannelBox(true);

    Friction = nAttr.create("Friction", "fr", MFnNumericData::kDouble, 0.0);
    nAttr.setDefault(0.1);
    nAttr.setMin(0.0);
    nAttr.setMax(1.0);
    nAttr.setChannelBox(true);

    Deformation = nAttr.create("Deformation", "de", MFnNumericData::kDouble, 0.0);
    nAttr.setDefault(0.5);
    nAttr.setMin(0.0);
    nAttr.setMax(1.0);
    nAttr.setChannelBox(true);

    Elasticity = nAttr.create("Elasticity", "el", MFnNumericData::kDouble, 0.0);
    nAttr.setDefault(0.5);
    nAttr.setMin(0.0);
    nAttr.setMax(1.0);
    nAttr.setChannelBox(true);
    
    InitialVelocity = nAttr.create("InitialVelocity", "iv", MFnNumericData::k3Double, 0.0);
    nAttr.setDefault(0.0);
    nAttr.setMin(-10.0);
    nAttr.setMax(10.0);
    nAttr.setChannelBox(true);


    // Add the attributes
    addAttribute(CurrentTime);
    addAttribute(GravityMagnitude);
    addAttribute(GravityDirection);
    addAttribute(Mass);
    addAttribute(Stiffness);
    addAttribute(Bounciness);
    addAttribute(Friction);
    addAttribute(Deformation);
    addAttribute(Elasticity);
    addAttribute(Mode);
    addAttribute(InitialVelocity);
    
    attributeAffects(CurrentTime, outputGeom);  // does this do anything?
    attributeAffects(GravityMagnitude, outputGeom);
    attributeAffects(GravityDirection, outputGeom);
    attributeAffects(Mass, outputGeom);
    attributeAffects(Stiffness, outputGeom);
    attributeAffects(Bounciness, outputGeom);
    attributeAffects(Friction, outputGeom);
    attributeAffects(Deformation, outputGeom);
    attributeAffects(Elasticity, outputGeom);
    attributeAffects(Mode, outputGeom);
    attributeAffects(InitialVelocity, outputGeom);
    
    // Make the deformer weights paintable (maybe wait with this)
    // MGlobal::executeCommand("makePaintable -attrType multiFloat -sm deformer PushDeformerNode weights;");

    return MS::kSuccess;
}
