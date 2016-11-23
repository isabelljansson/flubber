#include <iostream>
#include <vector>

#ifdef __linux__
#include "../glm/glm/glm.hpp"
#elif __APPLE__
#include "glm/glm.hpp"
#endif

using namespace std;
using namespace glm;

class ParticleSystem {
    public:
        ParticleSystem(vector<vec3>* x, vec3 vel);
        ~ParticleSystem();

        vector< vec3 >* getPos();
        vector< vec3 >* getVelocity();

        void deform();
        vec3 calcCom(vector< vec3 >* x);

    private:
    	vector< vec3 > *x0;
    	vector< vec3 > *v;
        vec3 initCom;

};
