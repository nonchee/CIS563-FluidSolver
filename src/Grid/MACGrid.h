#pragma once
#include "Grid.h"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/norm.hpp>
#include "glm/gtx/string_cast.hpp"

#include "../solvers/Particle.hpp"
#include <map>
#include <vector>

class MACGrid {
    
public:
    MACGrid(Grid* U, Grid* V, Grid* W, Grid* P);
    MACGrid(float fluidBoundX, float fluidBoundY, float fluidBoundZ, float gridCellSidelengths);
    
    int dimX ;
    int dimY ;
    int dimZ ;
    float cellSidelength;
    
    Grid* gridU;
    Grid* gridV;
    Grid* gridW;
    Grid* gridP;
    
    int getGridIndex(glm::vec3 position);
    
    //particle to grid
    void storeParticleVelocityToGrid(Particle p);
    
    //grid to particle
    glm::vec3 interpolateFromGrid(glm::vec3 pos) const;
    
    void colorSplattedFaces(Particle p); 

    void storeParticlesToGrid(std::map<int, std::vector<Particle>>* particlesByIndex);
   //void storeParticleToGrid(Particle p);
    void resetToZero(int size);
    
};