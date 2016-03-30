#include "Particle.hpp"


static const GLfloat g_vertex_buffer_data_array_data[] = {
    -0.5f, -0.5f, 0.0f,
    0.5f, -0.5f, 0.0f,
    -0.5f,  0.5f, 0.0f,
    0.5f,  0.5f, 0.0f,
};

const GLfloat* Particle::g_vertex_buffer_data_array = g_vertex_buffer_data_array_data;


std::vector<GLfloat> Particle::getParticleBaseMeshBufferData() {
    
    std::vector<GLfloat> particle_vertex_buffer_data;
    
    for (float f : g_vertex_buffer_data_array_data) {
        particle_vertex_buffer_data.push_back(f);
    }
    return particle_vertex_buffer_data;
    
}

bool Particle::operator<(const Particle& that) const {
    // Sort in reverse order : far particles drawn first.
    return this->cameradistance > that.cameradistance;
}

//assuming the speed had already been given from outside
void Particle::updatePositionWithRK2() {
    std::cout << " WEIGHTED SPEED " << glm::to_string(speed) << std::endl;
    std::cout << " old pos " << glm::to_string(pos) << std::endl;

    
    glm::vec3 fwdEulerPos = pos + speed;
    glm::vec3 midpoint = glm::vec3((fwdEulerPos.x + pos.x)/2,
                                   (fwdEulerPos.y + pos.y)/2,
                                   (fwdEulerPos.z + pos.z)/2 );
    
    std::cout << " new pos " << glm::to_string(pos) << std::endl;

    //forward euler
    
    
    //midpoint
    
    //new velocity
    
    //new velocity
   /* double K1 = (H * f(x,y));
    double K2 = (H * f((x + 1 / 2 * H), (y + 1 / 2 * K1)));
    double K3 = (H * f((x + 1 / 2 * H), (y + 1 / 2 * K2)));
    double K4 = (H * f((x + H), (y + K3)));     double runge = (y + (1 / 6) * (K1 + 2 * K2 + 2 * K3 + K4));
    positio = runge;
    
    Reference https://www.physicsforums.com/threads/runge-kutta-in-c.1448/
*/
}

