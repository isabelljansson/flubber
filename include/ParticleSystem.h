#include <iostream>
#include <vector>

#ifdef __linux__
#include <../armadillo/armadillo> 
#include "../glm/glm/glm.hpp"
#elif __APPLE__
#include <armadillo>
#include "glm/glm.hpp"
#endif

using namespace std;
using namespace glm;
using namespace arma; //may be conflicts between glm and arma

class ParticleSystem {
    public:
        ParticleSystem(vector<vec3>* x, vec3 vel);
        ~ParticleSystem();

        vector< vec3 >* getPos();
        vector< vec3 >* getVelocity();

        void deform();
        void updatePos();
        vec3 calcCom(vector< vec3 >* x);

    private:
    	vector< vec3 > *x0;	// initial position
    	vector< vec3 > *x1; 	// updated postion
    	vector< vec3 > *v;

        vec3 initCom;

};
