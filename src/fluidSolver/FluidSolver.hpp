#include "Particle.hpp"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <vector>
#include <glm/gtx/norm.hpp>




class FluidSolver {
    
public:
    void setUpParticleBuffers();
    void updateParticleBuffers();
    void update(float delta, float boxScaleX, float boxScaleY, float boxScaleZ, glm::vec3 CameraPosition);;
    
    void setParticleBounds(float x, float y, float z, float psep);
    int FindUnusedParticle();
    void SortParticles();
    void spawnParticles();
    void drawParticles();
    void deleteVBOS();
    
};