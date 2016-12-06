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

    cout << "Weeee" << endl;
    delete pos;
    return 0;
}
