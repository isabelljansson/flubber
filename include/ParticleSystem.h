#include <iostream>
#include <vector>

#ifdef __linux__
#include <../armadillo/include/armadillo> 
#include "../glm/glm/glm.hpp"
#include "../glm/glm/ext.hpp"
#include "../glm/gtc/matrix_access.hpp"
#elif __APPLE__
#include <armadillo>
#include "glm/glm.hpp"
#include "glm/ext.hpp"
#include "glm/gtc/matrix_access.hpp"
#endif

using namespace std;
//using namespace glm;
//using namespace arma; //may be conflicts between glm and arma

class ParticleSystem {
    public:

        // Attributes
        double dt; // Time step
        int mode;
        double mass;
        double flubbiness;
        double stiffness;
        double elasticity;
        double friction;
        //double staticFriction;
        //double dynamicFriction;

        glm::dvec3 gravity;
        glm::dvec3 initVel;

        ParticleSystem(vector<glm::dvec3>* x, glm::dvec3 vel);
        ~ParticleSystem();

        vector< glm::dvec3 >* getPos();
        vector< glm::dvec3 >* getVel();

        void deform();
        void applyForces();
        void updateForce();
        void updateVel();
        void updatePos();

        void setDt(double);
        glm::dvec3 getPosition(int i);
        glm::dvec3 calcCom(vector< glm::dvec3 >* x);



    private:
    	vector< glm::dvec3 > *x0; // Initial position
    	vector< glm::dvec3 > *x1; // Updated postion

    	vector< glm::dvec3 > *v; // Velocity
        vector< glm::dvec3 > *F; // External forces

        glm::dvec3 initCom;
        glm::dmat3 to_glm(arma::mat M);

};
