#ifndef PUSH_DEFORMER_NODE
#define PUSH_DEFORMER_NODE

#include <math.h>

#include <maya/MAnimControl.h>
#include <maya/MTime.h>
#include <maya/MDataBlock.h>
#include <maya/MDataHandle.h>
#include <maya/MGlobal.h>
#include <maya/MItGeometry.h>
#include <maya/MMatrix.h>
#include <maya/MPointArray.h>
#include <maya/MVector.h>
#include <maya/MStatus.h>
 
#include <maya/MFnMesh.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MFnEnumAttribute.h>
#include <maya/MFloatVectorArray.h>

#include <maya/MPxDeformerNode.h>

#include "../include/ParticleSystem.h"

// Simple deformer node
// It will deform the vertices along the normals of the object surface.
class PushDeformerNode : public MPxDeformerNode {
	public:
		PushDeformerNode() {};
		virtual MStatus deform(MDataBlock& data, MItGeometry& it_geo,
		                     const MMatrix &local_to_world_matrix, unsigned int m_index);
		static void* creator();
		static MStatus initialize();

		// Unique ID
		static MTypeId id;

		// Global Attributes
		static MObject GravityMagnitude;
		static MObject GravityDirection;

		// Object Attributes
		static MObject CurrentTime;
		static MObject Mass;
		static MObject Flubbiness;
		// ... stiffness, elasticity, staticfriction, dynamicfriction, something more?

		// Initial values
		static MObject InitialVelocity;

		static MObject Mode;

	private:
		static ParticleSystem* shape;
		static bool initFrame;
		static MTime tPrev;

};

#endif