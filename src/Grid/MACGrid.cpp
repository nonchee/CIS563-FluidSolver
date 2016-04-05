#include "MACGrid.h"
#include <math.h>
#include "../solvers/Particle.hpp"


MACGrid::MACGrid(Grid<float>* U, Grid<float>* V, Grid<float>* W, Grid<float>* P) {
    gridU = U;
    gridV = V;
    gridW = W;
    gridP = P;

}

MACGrid::MACGrid(float boxBoundX, float boxBoundY, float boxBoundZ, float gridCellSidelength) {
    
    cellSidelength = gridCellSidelength;
    
    //set number of grid cells in each direction
    dimX = ceil(boxBoundX / gridCellSidelength);
    dimY = ceil(boxBoundY / gridCellSidelength);
    dimZ = ceil(boxBoundZ / gridCellSidelength);
    
    gridU = new Grid<float>(dimX + 1, dimY, dimZ, cellSidelength);
    gridV = new Grid<float>(dimX, dimY + 1, dimZ, cellSidelength);
    gridW = new Grid<float>(dimX, dimY, dimZ + 1, cellSidelength);
    gridP = new Grid<float>(dimX, dimY, dimZ, cellSidelength);
    gridMarker = new Grid<int>(dimX, dimY, dimZ, cellSidelength);
    
    std::cout << "cell side " << gridCellSidelength << std::endl;
    std::cout << "grid dims " << dimX << " " << dimY << " " << dimZ << std::endl;
    std::cout << "box bounds: " << boxBoundX << " " << boxBoundY << " " << boxBoundZ << std::endl;
    
    
    
    /*//set gridmarker borders to solid
    for (int i = 0; i < dimX; i++) {
        for (int j = 0; j < dimY; j++) {
            for (int k = 0; k < dimZ; k++) {
                if (i == dimX - 1 || i == 0 || j == 0 || j == dimY -1 || k == 0 || k == dimZ - 1) {
                    int id = i + j * dimX + k * dimX * dimY;
                    std::cout << id << " lol solidddd" << std::endl;
                    gridMarker->setValueAt(-1, id);
                }

            }
        }
    }
    */
    
}
void MACGrid::setNumFluidCells(int num) {
    numFluidCells = num;
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

void MACGrid::resetToZero() {
    gridU->resetToZero();
    gridV->resetToZero();
    gridW->resetToZero();
}


void MACGrid::extrapolateVelocities() {

    gridV->extrapolateVelocities(gridMarker); //, <#int j#>, <#int k#>
}


void MACGrid::storeParticlesToGrid(std::map<int, std::vector<Particle>>* particlesByIndex){
    
    /*gridU->storeParticlesToGrid(particlesByIndex, glm::vec3(0, 0.5, 0.5));
    gridV->storeParticlesToGrid(particlesByIndex, glm::vec3(0.5, 0, 0.5));
    gridW->storeParticlesToGrid(particlesByIndex, glm::vec3(0.5, 0.5, 0));
    gridP->storeParticlesToGrid(particlesByIndex, glm::vec3(0.5, 0.5, 0.5));
*/
}


//take weighted average with kernel function
//assign that particle's velocity to the MAC grid
//mGrid->assignVelocityToCell(i, j, k, weightedVelocity);
//mGrid();
void MACGrid::storeParticleVelocityToGrid(Particle p){
    
    //gridU->storeParticleVelocityToGrid(p, glm::vec3(0, 0.5, 0.5), glm::vec3(1, 0, 0));
    gridV->storeParticleVelocityToGrid(p, glm::vec3(0.5, 0, 0.5), glm::vec3(0, 1, 0), W);
    //gridW->storeParticleVelocityToGrid(p, glm::vec3(0.5, 0.5, 0), glm::vec3(0, 0, 1));
    
}


//call this from flipsolver
glm::vec3 MACGrid::interpolateFromGrid(glm::vec3 pos) const {
    
    float velX = gridU->trilinearlyInterpolate(pos, glm::vec3(0, 0.5, 0.5));
    float velY = gridV->trilinearlyInterpolate(pos, glm::vec3(0.5, 0, 0.5));
    float velZ = gridW->trilinearlyInterpolate(pos, glm::vec3(0, 0.5, 0));
    
    return glm::vec3(velX, velY, velZ);
    
}

glm::vec3 MACGrid::giveNewVelocity(Particle p) {
    

    
    //for now testing with gridV only

    glm::vec3 PICvel = interpolateFromGrid(p.pos);
    
    //calculate change in velocties
    glm::vec3 FLIPvel = glm::vec3(); 
    
    //interpolate !
    
    //FLIPvel = glm::vec3(0.95 * FLIPvel.x, 0.95 * FLIPvel.y, 0.95 * FLIPvel.z);
    //PICvel = glm::vec3(0.05 * PICvel.x, 0.05 * PICvel.y, 0.05 * PICvel.z);
    return FLIPvel + PICvel;
    
}


void MACGrid::calculateAvgNumberOfParticlesPerGrid() {
    W = 0.0f;
    
    for (int numParticlesInCell : gridMarker->data) {
        W += numParticlesInCell;
    }
    W =  (float) W / (float)(gridMarker->data.size());
   // std::cout << W << std::endl;
}


void MACGrid::printMarker(std::string caption) {
    std::cout << caption << std::endl;
    
    for (int i = 0; i < gridMarker->data.size(); i++) {
        std::cout << i << " " << gridMarker->data.at(i) << " " << std::endl;
    }
    
}

void MACGrid::printDimensions() {
    std::cout << "--- MACGRID WITH CHEESE ---" << std::endl;
    
    std::cout << "particle bounds " << std::endl;
    
}