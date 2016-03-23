#include "MACGrid.h"
#include <math.h>
#include "../solvers/Particle.hpp"


MACGrid::MACGrid(Grid* U, Grid* V, Grid* W, Grid* P) {
    gridU = U;
    gridV = V;
    gridW = W;
    gridP = P;
}

MACGrid::MACGrid(float fluidBoundX, float fluidBoundY, float fluidBoundZ, float gridCellSidelength) {
    
    cellSidelength = gridCellSidelength;
    
    //set number of grid cells in each direction
    dimX = ceil(fluidBoundX / gridCellSidelength);
    dimY = ceil(fluidBoundY / gridCellSidelength);
    dimZ = ceil(fluidBoundZ / gridCellSidelength);
    
    gridU = new Grid(dimX + 1, dimY, dimZ, cellSidelength);
    gridV = new Grid(dimX, dimY + 1, dimZ, cellSidelength);
    gridW = new Grid(dimX, dimY, dimZ + 1, cellSidelength);
    gridP = new Grid(dimX, dimY, dimZ, cellSidelength);
}


int MACGrid::getGridIndex(glm::vec3 position) {

    //WE ALL LOVE FLOATING POINT ERRORS
    float epsilon = 0.001;
    
    //find closest multiple of cellSideLength
    int x = (position.x + epsilon)/cellSidelength;
    int y = (position.y + epsilon)/cellSidelength;
    int z = (position.z + epsilon)/cellSidelength;

    //give a 1D location for 3D index
    return (z * dimX * dimY) + (y * dimX) + x;
    
}

void MACGrid::resetToZero(int size) {
    gridU->resetToZero(size);
    gridV->resetToZero(size);
    gridW->resetToZero(size);
}

void MACGrid::storeParticlesToGrid(std::map<int, std::vector<Particle>>* particlesByIndex){
    
    gridU->storeParticlesToGrid(particlesByIndex, glm::vec3(0, 0.5, 0.5));
    gridV->storeParticlesToGrid(particlesByIndex, glm::vec3(0.5, 0, 0.5));
    gridW->storeParticlesToGrid(particlesByIndex, glm::vec3(0.5, 0.5, 0));
    gridP->storeParticlesToGrid(particlesByIndex, glm::vec3(0.5, 0.5, 0.5));

}

// for (i = 0; i < )
// (i - 1, j, z);
// (i, j, k
//take weighted average with kernel function
//assign that particle's velocity to the MAC grid
//mGrid->assignVelocityToCell(i, j, k, weightedVelocity);
//mGrid();
void MACGrid::storeParticleVelocityToGrid(Particle p){
    
    gridU->storeParticleVelocityToGrid(p, glm::vec3(0, 0.5, 0.5), glm::vec3(1, 0, 0));
    gridV->storeParticleVelocityToGrid(p, glm::vec3(0.5, 0, 0.5), glm::vec3(0, 1, 0));
    gridW->storeParticleVelocityToGrid(p, glm::vec3(0.5, 0.5, 0), glm::vec3(0, 0, 1));
    //gridP->storeParticleToGrid(p, glm::vec3(0.5, 0.5, 0.5), glm::vec3(1, 1, 1));
    
}


//call this from flipsolver
glm::vec3 MACGrid::interpolateFromGrid(glm::vec3 pos) const {
    
    float velX = gridU->trilinearlyInterpolate(pos, glm::vec3(0, 0.5, 0.5)).x;
    float velY = gridV->trilinearlyInterpolate(pos, glm::vec3(0.5, 0, 0.5)).y;
    float velZ = gridW->trilinearlyInterpolate(pos, glm::vec3(0, 0.5, 0)).z;
    
    return glm::vec3(velX, velY, velZ);
    
}

void MACGrid::colorSplattedFaces(Particle p) {
    
    gridU->colorSplattedFaces(p);
    gridV->colorSplattedFaces(p);
    gridW->colorSplattedFaces(p); //std::vector neighborhood = getNeighboring
    
}