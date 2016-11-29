#include "../include/ParticleSystem.h"

ParticleSystem::ParticleSystem(vector< vec3 >* x, vec3 vel) {
    cout << "hej\n";
    // Set initial position
    x0 = x;

    cout << "hej0\n";
    // Set initial velocity vector
    v->resize(x0->size());
    fill(v->begin(), v->end(), vel);
    cout << "hej1\n";

    // Calculate initial center of mass
    initCom = calcCom(x0);
    cout << "hej2\n";

}


ParticleSystem::~ParticleSystem() {

}

std::vector< vec3 >* ParticleSystem::getPos() {
	return x0;
}

void ParticleSystem::deform() {

}

vec3 ParticleSystem::calcCom(vector< vec3 >* x) {
    vec3 com = vec3(0, 0, 0);
    for(vector< vec3 >::iterator it = x->begin(); it != x->end(); ++it) {
        com += *it;
    }
    return com;
}
