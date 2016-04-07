#include "MACGrid.h"
#include <math.h>
#include "../solvers/Particle.hpp"


MACGrid::MACGrid(float boxBoundX, float boxBoundY, float boxBoundZ, float gridCellSidelength) {
    
    cellSidelength = gridCellSidelength;
    
    //set number of grid cells in each direction
    dimX = ceil(2 * boxBoundX / gridCellSidelength);
    dimY = ceil(2 * boxBoundY / gridCellSidelength);
    dimZ = ceil(2 * boxBoundZ / gridCellSidelength);
    
    bbX = boxBoundX;
    bbY = boxBoundY;
    bbZ = boxBoundZ;
    
    gridU = new Grid<float>(dimX + 1, dimY, dimZ, cellSidelength);
    gridV = new Grid<float>(dimX, dimY + 1, dimZ, cellSidelength);
    gridW = new Grid<float>(dimX, dimY, dimZ + 1, cellSidelength);
    gridP = new Grid<float>(dimX, dimY, dimZ, cellSidelength);
    gridMarker = new Grid<int>(dimX, dimY, dimZ, cellSidelength);
    
    markSolidBoundaries();
 
    
}

void MACGrid::markSolidBoundaries() {
    //set gridmarker borders to solid
    for (int i = 0; i < dimX; i++) {
        for (int j = 0; j < dimY; j++) {
            for (int k = 0; k < dimZ; k++) {
                if (i == dimX - 1 || i == 0 || j == 0 || j == dimY -1 || k == 0 || k == dimZ - 1) {
                    //int id = getGridIndex(i, j, k);
                    int id = i + j * dimX + k * dimX * dimY;
                    gridMarker->setValueAt(-1, id);
                }
            }
        }
    }
}

void MACGrid::setNumFluidCells(int num) {
    numFluidCells = num;
}


int MACGrid::getGridIndex(int i , int j, int k) {
    return getGridIndex(glm::vec3(i, j, k));
}

int MACGrid::getGridIndex(glm::vec3 position) {
    
    //assumes that the grid starts at 000 into +x, +y, +z
    float epsilon = 0.001;
    
    glm::vec3 boxOrig(-bbX, -bbY, -bbZ);
    for(int i = 0; i < 3; i++) {
        if (position[i] != boxOrig[i])  {
            position[i] -= epsilon;
        }
    }
    
    position -= boxOrig;
    
    //find closest multiple of cellSideLength
    int x = position.x/cellSidelength;
    int y = position.y/cellSidelength;
    int z = position.z/cellSidelength;

    //give a 1D location for 3D index
    return (z * dimX * dimY) + (y * dimX) + x;
    
}

glm::ivec3 MACGrid::getGridIJK(glm::vec3 position) {
    //assumes that the grid starts at 000 into +x, +y, +z
    
    //WE ALL LOVE FLOATING POINT ERRORS
    float epsilon = 0.001;
    
    glm::vec3 boxOrig((-bbX, -bbY, -bbZ));
    for(int i = 0; i < 3; i++) {
        if (position[i] != boxOrig[i])  {
            position[i] -= epsilon;
        }
    }
    
    position -= boxOrig;
    
    //find closest multiple of cellSideLength
    int x = position.x/cellSidelength;
    int y = position.y/cellSidelength;
    int z = position.z/cellSidelength;
    
    return glm::ivec3(x, y, z);
    
}


void MACGrid::resetGrids() {
    
    // marker to zero, keeping solids
    for (int i = 0; i < dimX; i++) {
        for (int j = 0; j < dimY; j++) {
            for (int k = 0; k < dimZ; k++) {
                if ((*gridMarker)(i, j, k) >= 0) {
                    gridMarker->setValueAt(0, i, j, k);
                }
            }
        }
    }
    
    //set all velocities to zero
    gridU->resetToZero();
    gridV->resetToZero();
    gridW->resetToZero();
    
    //DON'T reset pressures to zero
}


void MACGrid::addForcesToGrids(glm::vec3 forces, float delta) {
    
    std::cout << "force to add " << forces[1] * delta << std::endl;
    printMarker("add force to these pls");
    
    for (int i = 0; i < dimX; i++) {
        for (int j =0; j < dimY; j++) {
            for (int k =0; k < dimZ; k++) {
                if(isFluid(i, j, k)) {
                  gridV->updateVel(i, j, k, forces[1] * delta);
                }
            }
        }
    }
}



bool MACGrid::isFluid(int i, int j, int k) {
    return (*gridMarker)(i, j, k) > 0;
}


void MACGrid::extrapolateVelocities() {
    gridV->extrapolateVelocities(gridMarker);
}



void MACGrid::storeParVelToGrids(Particle p){
    
    //gridU->storeParticleVelocityToGrid(p, glm::vec3(0, 0.5, 0.5), glm::vec3(1, 0, 0));
    gridV->storeParticleVelocityToGrid(p, glm::vec3(0.5, 0, 0.5), W);
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



void MACGrid::PressureUpdate(float delta) {
    
    float scale = delta/(float)cellSidelength;

    for (int i = 0; i < dimX; i++) {
        for (int j = 0; j < dimY; j++) {
            for (int k = 0; k < dimZ; k++) {
                
                //get position in data
                glm::vec3 cur(i, j, k);
                //glm::vec3 neighbV = cur - gridV->gridDir;
                glm::vec3 neighbV = cur - gridV->gridDir;
                
                
                int id = gridP->ijkToGridIndex(cur);
                int id2 = gridP->ijkToGridIndex(neighbV);
                float pressureChange = gridP->data.at(id) - gridP->data.at(id2);
                
                gridV->addValueAt(-scale * pressureChange, i, j, k);
                
            }
        }
    }
}




void MACGrid::calculateAvgNumberOfParticlesPerGrid() {
    
    W = 0.0f;
    
    //right? is it avg number of particles in FLUID cells only?
    //or is it over the entire grid
    
    for (int numParticlesInCell : gridMarker->data) {
        if(numParticlesInCell > 0) {
             W += numParticlesInCell;
        }
    }
    
    //ask about this one lol
    
    W =  (float) W / (float)(numFluidCells);
    //W = (float) W / (float) gridMarker->data.size();
   
}


void MACGrid::printMarker(std::string caption) {
    std::cout << caption << std::endl;
    
    for (int i = 0; i < gridMarker->data.size(); i++) {
        if (i > 0 && i % gridMarker->dimX == 0) {
            std::cout << std::endl;
        }
        
        if (i % ((gridMarker->dimY)*(gridMarker->dimX)) == 0) {
            std::cout << std::endl;
        }
        std::cout << i << ": " << gridMarker->data.at(i) << "  ";
    }
    std::cout << std::endl;
}

void MACGrid::printDimensions() {
    std::cout << "--- McGRID  ---" << std::endl;
    
    std::cout << "grid dimensions " << dimX << " x " << dimY << "  " << dimZ <<std::endl;
    std::cout << "grid size " << (dimX * dimY * dimZ) << std::endl;
    std::cout << "box bounds: " << bbX << " " << bbY << " " << bbZ << std::endl;
    std::cout << " cell side: " << cellSidelength << std::endl;
    std::cout << " particle sep " << cellSidelength/2 << std::endl;
    
    
    std::cout << "-------" << std::endl;
    
    //std::cout << "particle bounds " << std::endl;
    
}