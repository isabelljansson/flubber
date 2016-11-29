#include <iostream>
#include <vector>

#ifdef __linux__
#include <../armadillo/include/armadillo> 
#include "../glm/glm/glm.hpp"
#include "../glm/glm/ext.hpp"
#elif __APPLE__
#include <armadillo>
#include "glm/glm.hpp"
#include "glm/ext.hpp"
#endif

using namespace std;
//using namespace glm;
//using namespace arma; //may be conflicts between glm and arma

class ParticleSystem {
    public:

        // Attributes
        int mode;
        double mass;
        double dt; // Time step
        double flubbiness;
        double stiffness;
        double elasticity;
        double staticFriction;
        double dynamicFriction;

        glm::vec3 gravity;
        glm::vec3 initVel;

        ParticleSystem(vector<glm::vec3>* x, glm::vec3 vel);
        ~ParticleSystem();

        vector< glm::vec3 >* getPos();
        vector< glm::vec3 >* getVel();

        void deform();
        void updateVel();
        void updatePos();
        glm::vec3 calcCom(vector< glm::vec3 >* x);

    private:
    	vector< glm::vec3 > *x0; // Initial position
    	vector< glm::vec3 > *x1; // Updated postion

    	vector< glm::vec3 > *v; // Velocity
        vector< glm::vec3 > *F; // Force

        glm::vec3 initCom;

        // Physics variables
        
};
