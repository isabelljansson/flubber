#include "../include/ParticleSystem.h"

ParticleSystem::ParticleSystem(vector< glm::dvec3 >* x, glm::dvec3 vel) {
    // Set initial position
    x0 = x;
    x1 = x;

    // Set intitial mode to a rigid transformation
    mode = 0;

    // Set initial velocity vector
    v = new vector< glm::dvec3 >();
    for (int i = 0; i < x0->size(); ++i) {
        v->push_back(vel);
    }

    // Calculate initial center of mass
    initCom = calcCom(x0);

}


ParticleSystem::~ParticleSystem() {
    delete v;

}

std::vector< glm::dvec3 >* ParticleSystem::getPos() {
	// Är det verkligen x0 som ska returnas?
	return x0;
}

std::vector< glm::dvec3 >* ParticleSystem::getVel() {
	return v;
}

void ParticleSystem::deform() {

	arma::mat q; 	// Original positions
	arma::mat p; 	// Deformed positions
	arma::mat Apq;	// Covariance matrix containing information about rotation
	arma::mat R;	// Rotation matrix in armadillo format
	arma::mat U, V; // matrices for svd
	arma::vec S; // vector for svd

	glm::dvec3 newCom; // New center of mass
	vector< glm::dvec3 > *g = x1; // Goal positions	
	glm::dmat3 Rot;	  // Rotation matrix in glm format

	newCom = calcCom(x1);

	switch (mode) {
		case 0: // Rigid transformation
			// Allocate
			p = arma::mat(3, x1->size());
			q = arma::mat(3, x1->size());

			// Init orgPos and defPos matrices
			for ( int i = 0; i < x1->size(); ++i ) {
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
			arma::svd(U,S,V,Apq);
			R = V * U.t();

			// Check if R has a reflection?

			Rot = to_glm(R);

			// Compute goal positions for 
			

			for ( int i = 0; i < x1->size(); ++i )
				g->at(i) = Rot * (x0->at(i) - initCom) + newCom;

			break;
		case 1: // Linear deformation
			cout << "Linear transformation\n";
			break;
		case 2: // Quadratic deformation
			cout << "Quadratic transformation\n";
			break; 
		default:
			cout << "Error with mode\n";
	}

	// Update positions with the modified Euler integration schema
	for (int i = 0; i < p.size(); ++i) {
		//v->at(i) += ...
		//x1->at(i) += ...
	}	
}

void ParticleSystem::updateVel()
{
    // Euler integration
    for (int i = 0; i < v->size(); ++i) {
        v->at(i) += F->at(i) / mass * dt;
        F->at(i) = glm::dvec3(0,0,0); // Reset all forces?
    }
}

void ParticleSystem::updatePos()
{
    // Euler integration
    for (int i = 0; i < x1->size(); ++i) {
        x1->at(i) += v->at(i) * dt;
    }   
}

glm::dvec3 ParticleSystem::calcCom(vector< glm::dvec3 >* x) {
    glm::dvec3 com = glm::dvec3(0, 0, 0);
    for(vector< glm::dvec3 >::iterator it = x->begin(); it != x->end(); ++it) {
        com += *it;
    }
    return com / (double) x->size();
}

// Convert Armadillo matrix to glm matrix
glm::dmat3 ParticleSystem::to_glm(arma::mat M) {
	glm::dmat3 M_glm;
	for (int i = 0; i < 3; ++i)
		for (int j = 0; j < 3; ++j)
			M_glm[i][j] = M(i,j);
	return M_glm;
}
