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

