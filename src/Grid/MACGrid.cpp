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

void MACGrid::storeParPosToMarker(Particle p) {
    
    int markerIndex = gridMarker->ijkToGridIndex(p.gridIJK);
    gridMarker->setValueAt(1, markerIndex); //cool so marker grid gets updated
    
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
    
    if (position.y < -bbY) {
        //position.y = 0 ; (-bbY- position.y); //set to zero
        //std::cout << "PARTICLE ABOUT TO BE OUT OF BOUNDS " << glm::to_string(position) << std::endl;
    }
    
    position -= boxOrig;
    
    if (position.y < 0) {
        //position.y = 0;
        //std::cout << "THIS IS THE BOX " << glm::to_string(boxOrig) << std::endl;
        //std::cout << "PARTICLE OUT OF BOUNDS " << glm::to_string(position) << std::endl;
    }
    
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
    markSolidBoundaries();
    
    //DON'T reset pressures to zero
}


void MACGrid::addExternalForcesToGrids(glm::vec3 forces, float dt) {
    
    std::vector<float> deltaV(gridV->data);
    
    std::cout << "force * dt = " << forces[1] * dt << std::endl;
    
    
    for (int i = 0; i < dimX; i++) {
        for (int j =0; j < dimY; j++) {
            for (int k =0; k < dimZ; k++) {
                if(isFluid(i, j, k)) {
                    gridU->updateVel(i, j, k, forces[0] * dt);
                    gridV->updateVel(i, j, k, forces[1] * dt);
                    gridW->updateVel(i, j, k, forces[2] * dt);
//                    std::cout << " whut come on " << (*gridV)(i, j, k);
                }
            }
        }
    }
}



bool MACGrid::isFluid(int i, int j, int k) {
    return (*gridMarker)(i, j, k) > 0;
}


void MACGrid::extrapolateVelocities() {
    gridU->extrapolateVelocities(gridMarker);
    gridV->extrapolateVelocities(gridMarker);
    gridW->extrapolateVelocities(gridMarker); 
}



void MACGrid::storeParVelToGrids(Particle p){
    
    gridU->storeParticleVelocityToGrid(p, glm::vec3(0, 0.5, 0.5), W);
    gridV->storeParticleVelocityToGrid(p, glm::vec3(0.5, 0, 0.5), W);
    gridW->storeParticleVelocityToGrid(p, glm::vec3(0.5, 0.5, 0), W);
    
}


//call this from flipsolver
glm::vec3 MACGrid::interpolateFromGrid(glm::vec3 pos,
                                       std::vector<float> deltaU,
                                       std::vector<float> deltaV,
                                       std::vector<float> deltaW) const {
        
    glm::vec3 boxOrig = glm::vec3(-bbX, -bbY, -bbZ);
    
    glm::vec3 poffset = boxOrig + glm::vec3(0, 0.5, 0.5);
    
    float picVelX = gridU->getInterpedVelocity(pos, poffset, gridU->data); //
    float flipVelX = gridU->getInterpedVelocity(pos, poffset, deltaU);
    
    
    poffset = boxOrig + glm::vec3(0.5, 0, 0.5);
    float picVelY = gridV->getInterpedVelocity(pos, poffset, gridV->data); //
    float flipVelY = gridV->getInterpedVelocity(pos, poffset, deltaV);
    
    poffset = boxOrig + glm::vec3(0.5, 0.5, 0);
    float picVelZ = gridW->getInterpedVelocity(pos, poffset, gridW->data); //
    float flipVelZ = gridW->getInterpedVelocity(pos, poffset, deltaW);
    
    glm::vec3 picVel(0.05 * picVelX, 0.05 * picVelY, 0.05 * picVelZ);
    glm::vec3 flipVel(0.95 * flipVelX, 0.95 * flipVelY, 0.95 * flipVelZ);
    
    return picVel + flipVel;
    
}


void MACGrid::UpdatePressureGrid(Eigen::SparseMatrix<float> &A, Eigen::VectorXf &p, float dt) {
    
    float dx = cellSidelength;
    float scale = dt/(dx * dx);
    
    
    for (int k=0; k<A.outerSize(); ++k) {
        
        // int prevPressure = 0;
        for (Eigen::SparseMatrix<float>::InnerIterator it(A,k); it; ++it)
        {
            int count = it.value();
            int id = it.index();
            float pressure = p[id];
            gridP->setValueAt(pressure, id);
            
        }
    }
    
    printMarker("these have fluids");
    
    //gridP->printContents("GRID P PRESSURE UPDATED");
}





void MACGrid::UpdateVelocityGridsByPressure(float delta) {
    
    
    float scale = delta/(float)(cellSidelength * cellSidelength);

    for (int i = 0; i < dimX; i++) {
        for (int j = 0; j < dimY; j++) {
            for (int k = 0; k < dimZ; k++) {
                
                int id;
                int id2;
                float pressureChange;
                
                //get P
                glm::vec3 cur(i, j, k);
                
                //std::cout << " u: previously this " << (*gridU)(i, j, k) << std::endl;
                glm::vec3 neighbU = cur - gridU->gridDir;
                id = gridP->ijkToGridIndex(cur);
                id2 = gridP->ijkToGridIndex(neighbU);
                pressureChange = (*gridP)((glm::ivec3) cur) - (*gridP)((glm::ivec3) neighbU);
            
                gridU->addValueAt(-scale * pressureChange, i, j, k);
                //std::cout << " u: does this change " << (*gridU)(i, j, k) << std::endl;
                

                //get P - 1

                //std::cout << " v: previously this: does this change " << (*gridV)(i, j, k) << std::endl;
                glm::vec3 neighbV = cur - gridV->gridDir;
                id = gridP->ijkToGridIndex(cur);
                id2 = gridP->ijkToGridIndex(neighbV);
                pressureChange = (*gridP)((glm::ivec3) cur) - (*gridP)((glm::ivec3) neighbV);
                gridV->addValueAt(-scale * pressureChange, i, j, k);
                //std::cout << " v: does this change " << (*gridV)(i, j, k) << std::endl;
                
               
                /*if ((*gridW)(i, j, k) > 0 ) {
                    std::cout << " w: previously this " << (*gridW)(i, j, k) << std::endl;
                }*/
         
                glm::vec3 neighbW = cur - gridW->gridDir;
                id = gridP->ijkToGridIndex(cur);
                id2 = gridP->ijkToGridIndex(neighbW);
                pressureChange = (*gridP)((glm::ivec3) cur) - (*gridP)((glm::ivec3) neighbW);
                gridW->addValueAt(-scale * pressureChange, i, j, k); //)
                
                /*if ((*gridW)(i, j, k) > 0 ) {
                    std::cout << "w: does this change "<< (*gridW)(i, j, k) << std::endl;
                }*/
                //std::cout << "  << (*gridW)(i, j, k) << std::endl;
                
            }
        }
    }
   
   /* gridU->printContents("YOLO U");
    gridV->printContents("YOLO V");
    gridW->printContents("YOLO W");
    */

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
        
        if (gridMarker->data.at(i) > 0) {
            std::cout << "[" <<i << ": " << gridMarker->data.at(i) << "]  ";
        }
        else {
          std::cout << i << ": " << gridMarker->data.at(i) << "  ";
        }
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