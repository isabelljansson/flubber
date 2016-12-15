#include <iostream>
#include <vector>

#ifdef __linux__
#include <armadillo> 
#include "../glm/glm/glm.hpp"
#include "../glm/glm/ext.hpp"
#include "../glm/glm/gtc/matrix_access.hpp"
#elif __APPLE__
#include <armadillo>
#include "glm/glm.hpp"
#include "glm/ext.hpp"
#include "glm/gtc/matrix_access.hpp"
#endif

using namespace std;

class ParticleSystem {
    public:
        // Attributes
        double dt;          // Time step
        int mode;           // Mode for rigid, linear or quadratic deformation
        double mass;        // Object parameter
        double bounciness;  // Object parameter
        double stiffness;   // Object parameter
        double elasticity;  // Object parameter
        double friction;    // Object parameter
        double beta;        // Deformation constant

        glm::dvec3 gravity;
        glm::dvec3 initVel;

        ParticleSystem(vector<glm::dvec3> x, glm::dvec3 vel);
        ~ParticleSystem();

        vector< glm::dvec3 >* getVel();

        void deform();
        void applyForces();
        void updateForce();
        void updateVel();
        void updatePos();

        void setDt(double);
        glm::dvec3 getPosition(int i);
        glm::dvec3 calcCom(vector< glm::dvec3 > x);

    private:
    	vector< glm::dvec3 > x0; // Initial position
    	vector< glm::dvec3 > x1; // Updated position

    	vector< glm::dvec3 > *v; // Velocity
        vector< glm::dvec3 > *F; // External forces

        glm::dvec3 initCom;
        glm::dmat3 to_glm(arma::mat M);
};
