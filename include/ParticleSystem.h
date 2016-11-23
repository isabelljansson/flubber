#include "glm/glm.hpp"

using namespace std;
using namespace glm;

class ParticleSystem {
    public:
        ParticleSystem();
        ~ParticleSystem();

        std::vector<particle>* getx0();

        void deform();

    private:
    	std::vector<glm::vec3>* x0;

};
