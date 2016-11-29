#include "../include/ParticleSystem.h"

ParticleSystem::ParticleSystem(vector< glm::vec3 >* x, glm::vec3 vel) {
    // Set initial position
    x0 = x;
    x1 = x;

    // Set initial velocity vector
    v = new vector< glm::vec3 >();
    for (int i = 0; i < x0->size(); ++i) {
        v->push_back(vel);
    }

    // Calculate initial center of mass
    initCom = calcCom(x0);

}


ParticleSystem::~ParticleSystem() {
    delete v;

}

std::vector< glm::vec3 >* ParticleSystem::getPos() {
	// Är det verkligen x0 som ska returnas?
	return x0;
}

std::vector< glm::vec3 >* ParticleSystem::getVel() {
	return v;
}

void ParticleSystem::deform() {

	arma::fmat orgPos; // Original positions
	arma::fmat defPos; // Deformed positions
	arma::fmat Apq; 	 // Covariance matrix containing information about rotation

	glm::vec3 newCom; // New center of mass

	newCom = calcCom(x1);

	// rigid bodies
	
}

void ParticleSystem::updatePos() {
	// Euler integration

}

glm::vec3 ParticleSystem::calcCom(vector< glm::vec3 >* x) {
    glm::vec3 com = glm::vec3(0, 0, 0);
    for(vector< glm::vec3 >::iterator it = x->begin(); it != x->end(); ++it) {
        com += *it;
    }
    return com / (float)x->size();
}
