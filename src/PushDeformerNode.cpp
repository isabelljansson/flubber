#include "../include/PushDeformerNode.h"

MTypeId PushDeformerNode::id(0x00000002);

MObject PushDeformerNode::GravityMagnitude;
MObject PushDeformerNode::GravityDirection;

MObject PushDeformerNode::CurrentTime;
MObject PushDeformerNode::Mass;
MObject PushDeformerNode::Flubbiness;

MObject PushDeformerNode::InitialVelocity;

bool PushDeformerNode::initFrame;
ParticleSystem* PushDeformerNode::shape;
MTime PushDeformerNode::tPrev;


MObject PushDeformerNode::Menu;
 
void* PushDeformerNode::creator() { return new PushDeformerNode; }

MStatus PushDeformerNode::deform(MDataBlock& data, MItGeometry& it_geo, 
    const MMatrix &local_to_world_matrix, unsigned int m_index) {

    // create ParticleSystem first frame
    MTime tNow = data.inputValue(CurrentTime).asTime();
    if (initFrame || tNow.value() == 1) {
        if (shape) // clean up, unneeded?
            delete shape;

        tPrev = data.inputValue(CurrentTime).asTime();
        std::vector<glm::vec3> *p0;
        MVector temp = data.inputValue(InitialVelocity).asVector(); // ugly?
        glm::vec3 v0 = glm::vec3(temp[0], temp[1], temp[2]);
        for (; !it_geo.isDone(); it_geo.next()) {
            MPoint vertexPos = it_geo.position() * local_to_world_matrix;
            glm::vec3 pi0(vertexPos.x, vertexPos.y, vertexPos.z);
            p0->push_back(pi0);
        }
        shape = new ParticleSystem(p0, v0);

        initFrame = false;
        return MS::kSuccess;
    }
    // Update
    else {
        MStatus status;

        // Fetch the envelope and the inflation input value
        float env = data.inputValue(envelope).asFloat(); // env is not used (as we use particle system instead)

        tNow = data.inputValue(CurrentTime).asTime();
        MTime tDiff = tNow - tPrev;
        tPrev = tNow;

        // physics arguments
        MVector temp = data.inputValue(GravityMagnitude).asDouble() 
            * data.inputValue(GravityDirection).asVector(); // ugly?
        /*shape->gravity*/ glm::vec3 gravity = glm::vec3(temp[0], temp[1], temp[2]);
        /*shape->mass*/ double mass = data.inputValue(Mass).asDouble();
        /*shape->flubbiness*/ double flub = data.inputValue(Flubbiness).asDouble();
        // more later..



        // Get the input mesh (fn_input_mesh)
        MArrayDataHandle h_input = data.outputArrayValue( input, &status );
        CHECK_MSTATUS_AND_RETURN_IT( status )
        status = h_input.jumpToElement( m_index );
        CHECK_MSTATUS_AND_RETURN_IT( status )
        MObject o_input_geom = h_input.outputValue().child( inputGeom ).asMesh();
        MFnMesh fn_input_mesh( o_input_geom );

        // Get the normal array from the input mesh
        MFloatVectorArray normals = MFloatVectorArray();
        fn_input_mesh.getVertexNormals(true, normals, MSpace::kTransform);


        // Loop through the geometry and set vertex positions
        for (; !it_geo.isDone(); it_geo.next()) {
            int idx = it_geo.index();
            MVector nrm = MVector(normals[idx]);
            MPoint pos = it_geo.position();
            MPoint new_pos = pos + (nrm * env);
            it_geo.setPosition(new_pos);
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

    // Create a numeric attributes
    Menu = eAttr.create("Menu", "me", 0);
    eAttr.setStorable(true);
    eAttr.setKeyable(true);
    eAttr.addField("Rigid", 0);
    eAttr.addField("Linear", 1);
    eAttr.addField("Quadratic", 2);


    GravityMagnitude = nAttr.create("GravityMagnitude", "gm", MFnNumericData::kDouble, 0.0);
    nAttr.setDefault(0.0);
    nAttr.setMin(0.0);
    nAttr.setMax(10.0);
    nAttr.setChannelBox(true);

    GravityMagnitude = nAttr.create("GravityDirection", "gd", MFnNumericData::k3Double, 0.0);
    nAttr.setDefault(0.0);
    nAttr.setMin(-1.0);
    nAttr.setMax(1.0);
    nAttr.setChannelBox(true);

    CurrentTime = nAttr.create("CurrentTime", "ct", MFnNumericData::kDouble, 0.0);
    uAttr.setDefault(MAnimControl::currentTime().as(MTime::kFilm));
    uAttr.setChannelBox(true);

    Mass = nAttr.create("Mass", "ma", MFnNumericData::kDouble, 0.0);
    nAttr.setDefault(1.0);
    nAttr.setMin(0.0);
    nAttr.setMax(10.0);
    nAttr.setChannelBox(true);

    Flubbiness = nAttr.create("Flubbiness", "fb", MFnNumericData::kDouble, 0.0);
    nAttr.setDefault(0.5);
    nAttr.setMin(0.0);
    nAttr.setMax(1.0);
    nAttr.setChannelBox(true);



    // Add the attributes
    addAttribute(CurrentTime);
    addAttribute(GravityMagnitude);
    addAttribute(GravityDirection);
    addAttribute(Mass);
    addAttribute(Flubbiness);
    addAttribute(Menu);

    attributeAffects(CurrentTime, outputGeom);  // does this do anything?
    attributeAffects(GravityMagnitude, outputGeom);
    attributeAffects(GravityDirection, outputGeom);
    attributeAffects(Mass, outputGeom);
    attributeAffects(Flubbiness, outputGeom);

    // Make the deformer weights paintable (maybe wait with this)
    // MGlobal::executeCommand("makePaintable -attrType multiFloat -sm deformer PushDeformerNode weights;");

    return MS::kSuccess;
}