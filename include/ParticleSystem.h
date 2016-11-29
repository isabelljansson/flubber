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
        glm::mat3 to_glm(arma::fmat M);

        // Physics variables
        float mass;
        float dt; // Time step
        int mode;
};
