#include "../include/ParticleSystem.h"

ParticleSystem::ParticleSystem(vector< glm::dvec3 > x, glm::dvec3 vel) {
    // Set initial position
    x0 = x;
    x1 = x;

    // Set intitial mode to a rigid transformation
    mode = 0;
    dt = 0.0;

    // Set initial velocity vector
    v = new vector< glm::dvec3 >();
    F = new vector< glm::dvec3 >();

    for (int i = 0; i < x.size(); ++i) {
        v->push_back(vel);
        F->push_back(glm::dvec3(0.0,0.0,0.0));
    }
    
    // Calculate initial center of mass
    initCom = calcCom(x0);
}


ParticleSystem::~ParticleSystem() {
    delete v;
    delete F;

}

void ParticleSystem::setDt(double deltaT) {
	dt = deltaT;
}

void ParticleSystem::applyForces() {
	// Update F
	updateForce();

	/* --- Calculate new velocities and update positions, this is the deformed shape --- */
	
	// Update velocities
	updateVel();

	// Update positions
	updatePos();
}

std::vector< glm::dvec3 >* ParticleSystem::getVel() {
	return v;
}

void ParticleSystem::deform() {
	arma::mat q; 	    // Original positions
	arma::mat qTilde; 	// Original positions for quadratic transformation
	arma::mat p; 	    // Deformed positions
	arma::mat Apq;	    // Covariance matrix containing information about rotation
	arma::mat ApqTilde; // Apq using qTilde
	arma::mat Aqq;      // Information about scaling
	arma::mat AqqTilde; // Aqq using qTilde
	arma::mat A;	    // Linear/quadratic transformation
    arma::mat Asquare;  // Squared A to calculate determinant
	arma::mat R;	    // Rotation matrix in armadillo format
	arma::mat RTilde;	// Bigger rotation matrix in armadillo format, for quadratic deformations
    arma::mat T;        // Combinaton of A and R
    arma::mat TTilde;   // Combination of ATilde and RTilde
	arma::mat U, V;     // matrices for svd
	arma::vec S;        // vector for svd
	arma::mat gTmp;     // Temporary goal solution

	glm::dvec3 newCom; // New center of mass
	vector< glm::dvec3 > g = x1; // Goal positions	
	glm::dmat3 Rot;	  // Rotation matrix in glm format

    // Calculate center of mass for the deformed positions
    newCom = calcCom(x1);

    // Allocate
    p = arma::mat(3, x1.size());
    q = arma::mat(3, x1.size());

    // Init orgPos and defPos matrices
    for ( int i = 0; i < x1.size(); ++i ) {
        p(0,i) = x1.at(i).x - newCom.x;
        p(1,i) = x1.at(i).y - newCom.y;
        p(2,i) = x1.at(i).z - newCom.z; 

        q(0,i) = x0.at(i).x - initCom.x;
        q(1,i) = x0.at(i).y - initCom.y;
        q(2,i) = x0.at(i).z - initCom.z;
    }

    // Find covariance matrix Apq
    Apq = mass * p * q.t();

    // Find rotational part in Apq through Singular Value Decomposition
    arma::svd(U,S,V,Apq);
    R = V * U.t();

    // Check if R has a reflection
    if (det(R) < 0) {
		R(0,2) = -R(0,2);
		R(1,2) = -R(1,2);
		R(2,2) = -R(2,2);
	}
    
	switch (mode) {
		case 0: // Rigid transformation

            // Convert to glm
			Rot = to_glm(R);

			// Compute goal positions 
			for ( int i = 0; i < x1.size(); ++i )
				g.at(i) = Rot * (x0.at(i) - initCom) + newCom;
	
			break;
		case 1: // Linear deformation

			// Compute Aqq
			Aqq = (mass * q * q.t()).i(); 

			A = Apq * Aqq;

			// Scale A to ensure that det(A)=1
			A /= pow(arma::det(A), 1/3);

			T = beta * A + (1.0 - beta) * R;

			// Check if R has a reflection?

			Rot = to_glm(T);

			// Compute goal positions 
			for ( int i = 0; i < x1.size(); ++i )
				g.at(i) = Rot * (x0.at(i) - initCom) + newCom;

			break;
		case 2: // Quadratic deformation

            // [R 0 0]
			RTilde = arma::mat(3,9);
			RTilde 	<< R(0,0) << R(0,1) << R(0,2) << 0 << 0 << 0 << 0 << 0 << 0 << arma::endr
					<< R(1,0) << R(1,1) << R(1,2) << 0 << 0 << 0 << 0 << 0 << 0 << arma::endr
					<< R(2,0) << R(2,1) << R(2,2) << 0 << 0 << 0 << 0 << 0 << 0 << arma::endr;

            // Calculate ~q = [qx, qy, qz, qx^2, qy^2, qz^2, qxqy, qyqz, qzqx]
			qTilde = arma::mat(9, x1.size());
			for ( int i = 0; i < x1.size(); ++i ) {
				qTilde(0,i) = x0.at(i).x - initCom.x;	// qx
				qTilde(1,i) = x0.at(i).y - initCom.y;	// qy
				qTilde(2,i) = x0.at(i).z - initCom.z;	// qz
				qTilde(3,i) = qTilde(0,i)*qTilde(0,i);	// qx^2
				qTilde(4,i) = qTilde(1,i)*qTilde(1,i);	// qy^2
				qTilde(5,i) = qTilde(2,i)*qTilde(2,i);	// qz^2
				qTilde(6,i) = qTilde(0,i)*qTilde(1,i);	// qx*qy
				qTilde(7,i) = qTilde(1,i)*qTilde(2,i);	// qy*qz
				qTilde(8,i) = qTilde(2,i)*qTilde(0,i);	// qz*qx
			}
            
            // Calculate Apq and Aqq using new ~q
			ApqTilde = mass * p * qTilde.t();
			AqqTilde = mass * (qTilde * qTilde.t()).i(); 

			// Ã
			A = ApqTilde * AqqTilde;

            // Scale ~A
            Asquare = arma::eye<arma::mat>(9,9);
            Asquare.row(0) = A.row(0);
            Asquare.row(1) = A.row(1);
            Asquare.row(2) = A.row(2);
            Asquare /= pow(arma::det(Asquare), 1/9); 

            A.row(0) = Asquare.row(0);
            A.row(1) = Asquare.row(1);
            A.row(2) = Asquare.row(2);
			
            TTilde = beta * A + (1.0 - beta) * RTilde;

			gTmp = TTilde * qTilde;

			// Compute goal positions
			for ( int i = 0; i < x1.size(); ++i ) {
				g.at(i).x = gTmp(0,i);
				g.at(i).y = gTmp(1,i);
				g.at(i).z = gTmp(2,i);

				g.at(i) += newCom;
			}

			break; 
		default:
			cout << "Error with mode\n";
	}

	// Update positions with the modified Euler integration schema
	for (int i = 0; i < x1.size(); ++i) {
		v->at(i) += bounciness * stiffness * (g.at(i) - x1.at(i)) / dt;
		x1.at(i) += stiffness* (g.at(i) - x1.at(i));
		if (x1.at(i).y <= 0) 
            x1.at(i).y = 0.01; // Set position to above object
	}	
}

void ParticleSystem::updateForce()
{
    // Should set forces according to input and collisions etc
    for (int i = 0; i < F->size(); ++i) {
        // Gravity
        F->at(i) = gravity * mass / (double) x1.size();

        // Add collision impulse and friction
        if (x1.at(i).y <= 0) {
            glm::dvec3 normal = glm::dvec3(0.0, 1.0, 0.0);
            glm::dvec3 deltaV = v->at(i) - glm::dvec3(0,0,0); // Floor is static

            glm::dvec3 composant = normal * glm::dot(normal, deltaV); // deltaV composant in normal direction

            glm::dvec3 collisionImpulse = -(elasticity + 1) * normal * glm::dot(normal, deltaV) * (mass / x1.size());
            glm::dvec3 frictionImpulse = -friction * (deltaV - composant) * (mass / x1.size());

            F->at(i) += (collisionImpulse + frictionImpulse) / dt;
            x1.at(i).y = 0.01; // Set position to above object
        }
    }
}

glm::dvec3 ParticleSystem::getPosition(int i) {
	return x1.at(i);
}

void ParticleSystem::updateVel() {
    // Euler integration
    for (int i = 0; i < v->size(); ++i) {
        v->at(i) += F->at(i) / mass * dt;
        F->at(i) = glm::dvec3(0,0,0); // Reset all forces?
    }
}

void ParticleSystem::updatePos() {
    // Euler integration
    for (int i = 0; i < x1.size(); ++i) {
        x1.at(i) += v->at(i) * dt;
        if (x1.at(i).y <= 0) 
            x1.at(i).y = 0.01; // Set position to above object
        cout << "x0: " << to_string(x0.at(i)) << endl;
    	cout << "x1: " << to_string(x1.at(i)) << endl;
    }
}

glm::dvec3 ParticleSystem::calcCom(vector< glm::dvec3 > x) {
    glm::dvec3 com = glm::dvec3(0, 0, 0);
    for(vector< glm::dvec3 >::iterator it = x.begin(); it != x.end(); ++it) {
        com += *it;
    }
    return com / (double) x.size();
}

// Convert Armadillo matrix to glm matrix
glm::dmat3 ParticleSystem::to_glm(arma::mat M) {
	glm::dmat3 M_glm;
	for (int i = 0; i < 3; ++i)
		for (int j = 0; j < 3; ++j)
			M_glm[i][j] = M(i,j);
	return M_glm;
}
