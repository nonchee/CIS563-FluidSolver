#ifndef PARTICLE_HPP
#define PARTICLE_HPP
//particle class

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/norm.hpp>
#include <iostream>
#include "glm/gtx/string_cast.hpp"
//using namespace glm;


class Particle {
    static const GLfloat* g_vertex_buffer_data_array;
    
    
public:
    int gridIndex;
    glm::vec3 pos, speed;
    unsigned char r,g,b,a; // Color
    float size, angle, weight;
    float life; // Remaining life of the particle. if <0 : dead and unused.
    float cameradistance; // *Squared* distance to the camera. if dead : -1.0f
    
    static std::vector<GLfloat> getParticleBaseMeshBufferData(); //the same for all particles !
    bool operator<(const Particle& that) const;
    glm::ivec3 gridIJK; 
    
    void updatePositionWithRK2();
};

#endif