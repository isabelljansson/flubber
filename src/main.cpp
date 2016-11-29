// Main for test purposes before running maya

#include <iostream>
#include "../include/ParticleSystem.h"

using namespace std;

int main(int argc, char* argv[]) {
    vector< glm::vec3 >* pos = new vector< glm::vec3 >;
    pos->push_back(glm::vec3(0.0, 0.0, 0.0));
    pos->push_back(glm::vec3(1.0, 1.0, 0.0));
    glm::vec3 vel = glm::vec3(1, 0, 1);
    ParticleSystem object = ParticleSystem(pos, vel);

    cout << "Weeee" << endl;
    delete pos;
    return 0;
}
