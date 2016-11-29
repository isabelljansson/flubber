#include "../include/ParticleSystem.h"

ParticleSystem::ParticleSystem(vector< vec3 >* x, vec3 vel) {
    // Set initial position
    x0 = x;
    x1 = x;

    // Set initial velocity vector
    v = new vector< vec3 >();
    for (int i = 0; i < x0->size(); ++i) {
        v->push_back(vel);
    }

    // Calculate initial center of mass
    initCom = calcCom(x0);

}


ParticleSystem::~ParticleSystem() {
    delete v;

}

std::vector< vec3 >* ParticleSystem::getPos() {
	// Är det verkligen x0 som ska returnas?
	return x0;
}

std::vector< vec3 >* ParticleSystem::getVel() {
	return v;
}

void ParticleSystem::deform() {

	fmat orgPos; // Original positions
	fmat defPos; // Deformed positions
	fmat Apq; 	 // Covariance matrix containing information about rotation

	vec3 newCom; // New center of mass

	newCom = calcCom();

	// rigid bodies
	
}

void ParticleSystem::updatePos() {
	// Euler integration

}

vec3 ParticleSystem::calcCom(vector< vec3 >* x) {
    vec3 com = vec3(0, 0, 0);
    for(vector< vec3 >::iterator it = x->begin(); it != x->end(); ++it) {
        com += *it;
    }
    return com / x.size();
}
