// Main for test purposes before running maya

#include <iostream>
#include "../include/ParticleSystem.h"

using namespace std;

int main(int argc, char* argv[]) {
    vector< vec3 >* pos = new vector< vec3 >;
    pos->push_back(vec3(0.0, 0.0, 0.0));
    vec3 vel = vec3(0, 0, 0);
    ParticleSystem object = ParticleSystem(pos, vel);

    return 0;
}