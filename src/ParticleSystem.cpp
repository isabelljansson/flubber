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

	arma::fmat q; 	// Original positions
	arma::fmat p; 	// Deformed positions
	arma::fmat Apq;	// Covariance matrix containing information about rotation
	arma::fmat R;	// Rotation matrix

	glm::vec3 newCom; // New center of mass

	newCom = calcCom(x1);

	/* --- RIGID BODIES --- */

	// Allocate
	p = arma::fmat(3, x1->size());
	q = arma::fmat(3, x1->size());

	// Init orgPos and defPos matrices
	for (int i = 0; i < x1->size(); ++i) {
		p(0,i) = x1->at(i).x - newCom.x;
		p(1,i) = x1->at(i).y - newCom.y;
		p(2,i) = x1->at(i).z - newCom.z; 

		q(0,i) = x0->at(i).x - initCom.x;
		q(1,i) = x0->at(i).y - initCom.y;
		q(2,i) = x0->at(i).z - initCom.z;
	}

	// Find covariance matrix Apq
	// should be multiplied with x1->size()*massPerParticle
	Apq = p * q.t();

	// Find rotational part in Apq through Singular Value Decomposition
	arma::fmat U, V; // matrices for svd
	//arma::fvec S; // vector for svd

	//arma::svd(U,S,V,Apq);
	//R = V * U.t();


	
}


void ParticleSystem::updateVel()
{
    // Euler integration
    for (int i = 0; i < v->size(); ++i) {
        v->at(i) += F->at(i) / mass * dt;
        F->at(i) = glm::vec3(0,0,0); // Reset all forces?
    }
}

void ParticleSystem::updatePos()
{
    // Euler integration
    for (int i = 0; i < x1->size(); ++i) {
        x1->at(i) += v->at(i) * dt;
    }   
}

glm::vec3 ParticleSystem::calcCom(vector< glm::vec3 >* x) {
    glm::vec3 com = glm::vec3(0, 0, 0);
    for(vector< glm::vec3 >::iterator it = x->begin(); it != x->end(); ++it) {
        com += *it;
    }
    return com / (float)x->size();
}
