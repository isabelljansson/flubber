// Main for test purposes before running maya

#include <iostream>
#include "../include/ParticleSystem.h"

using namespace std;

int main(int argc, char* argv[]) {
    vector< glm::dvec3 >* pos = new vector< glm::dvec3 >;
    pos->push_back(glm::dvec3(0.0, 0.0, 0.0));
    pos->push_back(glm::dvec3(1.0, 1.0, 0.0));
    glm::dvec3 vel = glm::dvec3(1, 0, 1);
    ParticleSystem object = ParticleSystem(pos, vel);
    object.gravity = glm::dvec3(0.0, -9.82, 0.0);
    object.initVel = glm::dvec3();
    object.mass = 1.0;
    object.flubbiness = 0.5;
    object.friction = 0.1;
    object.beta = 0.5;
    object.elasticity = 0.5;
    object.dt = 0.033;
    object.mode = 0;

    for (int i = 0; i < 3; i++) {
        object.applyForces();
        object.deform();
    }

    //cout << "size: " << pos->size() << endl;
    // Write out new positions
    for ( int i = 0; i < pos->size(); i++) {
        cout << "pos: " << glm::to_string(object.getPosition(i)) << endl;
    }

    cout << "Weeee" << endl;
    delete pos;
    return 0;
}
