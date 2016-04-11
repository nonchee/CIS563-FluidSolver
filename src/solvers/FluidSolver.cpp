#include "FluidSolver.hpp"
#include "Particle.hpp"


const int MaxParticles = 100000;
//Particle ParticlesContainer[MaxParticles];

int LastUsedParticle = 0;
int ParticlesCount;


GLuint billboard_vertex_buffer;
GLuint particles_position_buffer;
GLuint particles_color_buffer;

static GLfloat* g_particule_position_size_data = new GLfloat[MaxParticles * 4];
static GLubyte* g_particule_color_data         = new GLubyte[MaxParticles * 4];

void FluidSolver::drawParticles() {
    // 1rst attribute buffer : vertices
    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, billboard_vertex_buffer);
    glVertexAttribPointer(
                          0,                  // attribute 0
                          3,                  // size
                          GL_FLOAT,           // type
                          GL_FALSE,           // normalized?
                          0,                  // stride
                          (void*)0            // array buffer offset
                          );
    
    // 2nd attribute buffer : positions of particles' centers
    glEnableVertexAttribArray(1);
    glBindBuffer(GL_ARRAY_BUFFER, particles_position_buffer);
    glVertexAttribPointer(
                          1,                                // attribute 1
                          4,                                // size : x + y + z + size => 4
                          GL_FLOAT,                         // type
                          GL_FALSE,                         // normalized?
                          0,                                // stride
                          (void*)0                          // array buffer offset
                          );
    
    // 3rd attribute buffer : particles' colors
    glEnableVertexAttribArray(2);
    glBindBuffer(GL_ARRAY_BUFFER, particles_color_buffer);
    glVertexAttribPointer(
                          2,                                // attribute. No particular reason for 1, but must match the layout in the shader.
                          4,                                // size : r + g + b + a => 4
                          GL_UNSIGNED_BYTE,                 // type
                          GL_TRUE,                          // normalized?    *** YES, this means that the unsigned char[4] will be accessible with a vec4 (floats) in the shader ***
                          0,                                // stride
                          (void*)0                          // array buffer offset
                          );
    
    glVertexAttribDivisor(0, 0); // particles vertices : always reuse the same 4 vertices -> 0
    glVertexAttribDivisor(1, 1); // positions : one per quad (its center)                 -> 1
    glVertexAttribDivisor(2, 1); // color : one per quad                                  -> 1
    
    // Draw the particules ! This draws many times a small triangle_strip (which looks like a quad).
    // This is equivalent to : for(i in ParticlesCount) : glDrawArrays(GL_TRIANGLE_STRIP, 0, 4),
    // but faster.
    glDrawArraysInstanced(GL_TRIANGLE_STRIP, 0, 4, ParticlesCount);
    glDisableVertexAttribArray(0);
    glDisableVertexAttribArray(1);
    glDisableVertexAttribArray(2);
}


void FluidSolver::setParticleBounds(float x, float y, float z, float psep) {
    particleBoundX = x;
    particleBoundY = y;
    particleBoundZ = z;
    particleSeparation = psep;
    
}

// Finds a Particle in ParticlesContainer which isn't used yet.
// (i.e. life < 0);
int FluidSolver::FindUnusedParticle(){
    
    for(int i=LastUsedParticle; i<MaxParticles; i++){
        if (ParticlesContainer[i].life < 0){
            LastUsedParticle = i;
            return i;
        }
    }
    
    for(int i=0; i<LastUsedParticle; i++){
        if (ParticlesContainer[i].life < 0){
            LastUsedParticle = i;
            return i;
        }
    }
    
    return 0; // All particles are taken, override the first one
}

void FluidSolver::SortParticles(){
    std::sort(&ParticlesContainer[0], &ParticlesContainer[MaxParticles]);
}


void FluidSolver::update(float delta, float boxScaleX, float boxScaleY, float boxScaleZ, glm::vec3 CameraPosition) {
    
    ParticlesCount = 0;
    // Simulate all particles
    bool gravityEnabled = true;
    
       // std::cout << "ParticlesCount " << ParticlesContainer.size();
  
    for(int i=0; i<ParticlesContainer.size(); i++){
        
        Particle& p = ParticlesContainer[i]; // shortcut
        
        
        // Fill the GPU buffer
        g_particule_position_size_data[4*ParticlesCount+0] = p.pos.x;
        g_particule_position_size_data[4*ParticlesCount+1] = p.pos.y;
        g_particule_position_size_data[4*ParticlesCount+2] = p.pos.z;
        g_particule_position_size_data[4*ParticlesCount+3] = p.size;
        
        
        g_particule_color_data[4*ParticlesCount+0] = p.r;
        g_particule_color_data[4*ParticlesCount+1] = p.g;
        g_particule_color_data[4*ParticlesCount+2] = p.b;
        g_particule_color_data[4*ParticlesCount+3] = p.a;
        
        /*//collisiondetection
        if (p.pos.x > -boxScaleX && p.pos.x < boxScaleX && p.pos.y > -boxScaleY && p.pos.y < boxScaleY) {
            
        }
        
        else{
            g_particule_color_data[4*ParticlesCount+0] = p.r;
            g_particule_color_data[4*ParticlesCount+1] = 255;
            g_particule_color_data[4*ParticlesCount+2] = p.b;
            g_particule_color_data[4*ParticlesCount+3] = 200;
        }*/
        
        p.cameradistance = -1.0f;
        
        ParticlesCount++;
        
    }
    
    
}

void FluidSolver::setUpParticleBuffers() {
    // The VBO containing the 4 vertices of the particles.
    // Thanks to instancing, they will be shared by all particles.
    std::vector<GLfloat> g_vertex_buffer_data = Particle::getParticleBaseMeshBufferData();
    
    
    glGenBuffers(1, &billboard_vertex_buffer);
    glBindBuffer(GL_ARRAY_BUFFER, billboard_vertex_buffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(g_vertex_buffer_data), &g_vertex_buffer_data[0], GL_STATIC_DRAW);
    
    // The VBO containing the positions and sizes of the particle
    glGenBuffers(1, &particles_position_buffer);
    glBindBuffer(GL_ARRAY_BUFFER, particles_position_buffer);
    glBufferData(GL_ARRAY_BUFFER, MaxParticles * 4 * sizeof(GLfloat), NULL, GL_STREAM_DRAW);
    
    
    glGenBuffers(1, &particles_color_buffer);
    glBindBuffer(GL_ARRAY_BUFFER, particles_color_buffer);
    // Initialize with empty (NULL) buffer : it will be updated later, each frame.
    glBufferData(GL_ARRAY_BUFFER, MaxParticles * 4 * sizeof(GLubyte), NULL, GL_STREAM_DRAW);
}

void FluidSolver::updateParticleBuffers() {
    
    
    
    
    
    
    glBindBuffer(GL_ARRAY_BUFFER, particles_position_buffer);
    glBufferData(GL_ARRAY_BUFFER, MaxParticles * 4 * sizeof(GLfloat), NULL, GL_STREAM_DRAW);
    glBufferSubData(GL_ARRAY_BUFFER, 0, ParticlesCount * sizeof(GLfloat) * 4, g_particule_position_size_data);
    
    glBindBuffer(GL_ARRAY_BUFFER, particles_color_buffer);
    glBufferData(GL_ARRAY_BUFFER, MaxParticles * 4 * sizeof(GLubyte), NULL, GL_STREAM_DRAW);
    glBufferSubData(GL_ARRAY_BUFFER, 0, ParticlesCount * sizeof(GLubyte) * 4, g_particule_color_data);
    
    
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
}





void FluidSolver::spawnParticles() {
    
    
    for (float i = 0; i < particleBoundX; i+= particleSeparation) {
        for (float j = 0; j < particleBoundY; j += particleSeparation) {
            for (float k = 0; k < particleBoundZ; k += particleSeparation) {
                
                Particle particle;
                
                particle.pos = glm::vec3(i, j, k);
                
                particle.r = 0;
                particle.g = 0;
                particle.b = 255;
                particle.a = 255;
                
                particle.life = -1.0f;
                particle.cameradistance= -1.0f;
                particle.size = 0.1; //(rand()%1000)/2000.0f + 0.1f;
                
                
                ParticlesContainer.push_back(particle);
                
            }
        }
    }
    
    
}


void FluidSolver::deleteVBOS() {
    delete[] g_particule_position_size_data;
    
    // Cleanup VBO and shader
    glDeleteBuffers(1, &particles_color_buffer);
    glDeleteBuffers(1, &particles_position_buffer);
    glDeleteBuffers(1, &billboard_vertex_buffer);
}