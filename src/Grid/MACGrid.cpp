#include "MACGrid.h"
#include <math.h>



MACGrid::MACGrid(float boxBoundX, float boxBoundY, float boxBoundZ) {
    
    
    //set number of grid cells in each direction
    dimX = ceil(2 * boxBoundX  / DX );
    dimY = ceil(2 * boxBoundY  / DX );
    dimZ = ceil(2 * boxBoundZ  / DX );
    
    bbX = boxBoundX;
    bbY = boxBoundY;
    bbZ = boxBoundZ;
    
    gridU = new Grid<float>(dimX + 1, dimY, dimZ);
    gridV = new Grid<float>(dimX, dimY + 1, dimZ);
    gridW = new Grid<float>(dimX, dimY, dimZ + 1);
    gridP = new Grid<float>(dimX, dimY, dimZ);
    gridMarker = new Grid<int>(dimX, dimY, dimZ);
 
    
}

int MACGrid::ijkToIndex(glm::ivec3 ijk) {
    return (ijk.z * dimX * dimY) + (ijk.y * dimX) + ijk.x;
}

int MACGrid::ijkToIndex(int i, int j, int k) {
    return (k * dimX * dimY) + (j * dimX) + i;
}

int MACGrid::posToIndex(float i , float j, float k) {
    return posToIndex(glm::vec3(i, j, k));
}

int MACGrid::posToIndex(glm::vec3 position) {
    //std::cout <<"    pos " << glm::to_string(position) << std::endl;
  
    
    //assumes that the grid starts at 000 into +x, +y, +z
    float epsilon = 0.001;

    glm::vec3 boxOrig(-bbX, -bbY, -bbZ);
    //std::cout << "   box things " << glm::to_string(boxOrig) << std::endl;
    
    position -= boxOrig;
    //std::cout <<"    pos after box orig " << glm::to_string(position) << std::endl;
    
    //find closest multiple of cellSideLength
    int x = position.x/DX;
    int y = position.y/DX;
    int z = position.z/DX;
    
    //std::cout <<"    after inting " << x << " " << y << " " << z << std::endl;
    //std::cout <<"    therefore index is " <<  (z * dimX * dimY) + (y * dimX) + x << std::endl;

    //give a 1D location for 3D index
    return (z * dimX * dimY) + (y * dimX) + x;
    
}

int i ;
void MACGrid::storeParticleToMarker(Particle p) {
    

    
    int markerIndex = gridMarker->ijkToIndex(p.gridIJK);
    // << i++ << std::endl;
    gridMarker->setValueAt(1, markerIndex);
    //std::cout << glm::to_string((*gridMarker)(p.gridIJK)) << std::endl;
    //printf("%i store particle to marker[%i] = ", i++, markerIndex);
   
    //cool so marker grid gets updated
    
}

glm::ivec3 MACGrid::posToIJK(glm::vec3 position) {
    

    glm::vec3 boxOrig(-bbX, -bbY, -bbZ);
    
    position -= boxOrig;
    

    int x = position.x/DX;
    int y = position.y/DX;
    int z = position.z/DX;
    
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
    gridP->resetToZero(); 

    

}


void MACGrid::addExternalForcesToGrids(glm::vec3 forces) {
    
    std::vector<float> deltaU; //(mGrid->gridU->data);
    std::vector<float> deltaV;//(mGrid->gridV->data);
    std::vector<float> deltaW;//(mGrid->gridW->data);
    
    deltaU = gridU->data;
    deltaV = gridV->data;
    deltaW = gridW->data;
    
    
    //caluclate
    for (int i = 0; i < gridU->dimX; i++) {
        for (int j =0; j < gridU->dimY; j++) {
            for (int k =0; k < gridU->dimZ; k++) {
                
                gridU->addValueAt(forces[0],i, j, k);
        
            }
        }
    }
    
    for (int i = 0; i < gridV->dimX; i++) {
        for (int j =0; j < gridV->dimY; j++) {
            for (int k =0; k < gridV->dimZ; k++) {
                
                gridV->addValueAt(forces[1],i, j, k);
                
            }
        }
    }
    
    for (int i = 0; i < gridW->dimX; i++) {
        for (int j =0; j < gridW->dimY; j++) {
            for (int k =0; k < gridW->dimZ; k++) {
                
                gridW->addValueAt(forces[2],i, j, k);
                
            }
        }
    }


    ///calculate delta velocity
    for (int i = 0; i < gridV->data.size(); i++) {
        deltaU[i] = gridU->data[i] - deltaV[i];
        deltaV[i] = gridV->data[i] - deltaV[i];
        deltaW[i] = gridW->data[i] - deltaW[i];
    }
    
    gridU->setDeltas(deltaU);
    gridV->setDeltas(deltaV);
    gridW->setDeltas(deltaW);
    
    
}


void MACGrid::extrapolateVelocities() {
  
    
    
    //fuids and near_fluids == FLUID
    // PER GRID
    // if me or behind was fluid in original, then i am fluid
    // otherwise i am air (even if solid was original)
    
    // iterate through TEMPORARY GRID OF JUST AIR AND FLUIDS
    // if current temp cell is NOT FLUID,
    // check each of 6 neighbors, take each fluid neighbor and add its velocity
    // divide by total number of fluid cells;
    
    
    /*std::vector<int> pseudoMarkers;
     
     std::vector<T> oldData(data);
     
     for (int i = 0; i < dimX; i++) {
     for (int j = 0; j < dimY; j++) {
     for (int k = 0; k < dimZ; k++) {
     
     float totalVel = 0;
     
     int gridIndex = ijkToIndex(glm::vec3(i, j, k));
     
     switch (axis) {
     case 0:
     if (i == 0 || i == dimX - 1) {
     data.at(gridIndex) = 0;
     }
     break;
     case 1: //std::cout << " I AM A V GRID " << std::endl;
     if (j == 0 || j == dimY - 1) {
     data.at(gridIndex) = 0;
     }
     break;
     case 2: //std::cout << " I AM A W GRID " << std::endl;
     if (k == 0 || k == dimZ - 1) {
     data.at(gridIndex) = 0;
     }
     break;
     default: " LOL WHUT I HAVE NO AXIS ";
     }
     
     
     //if empty or solid
     if ((*marker)(i, j, k) <= 0) {
     
     
     //std::vector<glm::ivec3> neighbors = getTrilinNeighbors(glm::ivec3(i, j, k));
     int numNeighbors = 0;
     
     for (glm::ivec3 neighbor : neighbors) {
     //printVector((glm::vec3)neighbor, "  was neighbor");
     //if neighbor was fluid, get its vel and add to neighbcount
     if ((*marker)(neighbor.x, neighbor.y, neighbor.z) >=0) {
     totalVel+= oldData.at(ijkToIndex(neighbor));
     numNeighbors++;
     }
     }
     //can't just use neighbrs.size() because trilin is quite general
     if (numNeighbors > 0) {
     data.at(gridIndex) = totalVel/(float)numNeighbors;
     }
     
     else {
     data.at(gridIndex) = 0;
     }
     }
     }
     }
     }
     
     */
    
    
    gridU->extrapolateVelocities(gridMarker);
    gridV->extrapolateVelocities(gridMarker);
    gridW->extrapolateVelocities(gridMarker);
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




void MACGrid::UpdatePressureGrid(Eigen::SparseMatrix<float> &A, Eigen::VectorXf &p) {
    
    
    for (int id = 0; id < p.size(); id++) {
        if (gridMarker->data[id] > 0) { //if is fluid
            float pressure = p[id];
            gridP->setValueAt(pressure, id);
        }
    }
    
}


bool MACGrid::isSolid(glm::ivec3 cellIJK) {
    return (*gridMarker)(cellIJK.x, cellIJK.y, cellIJK.z) < 0;
}

bool MACGrid::isFluidOrAir(glm::ivec3 cellIJK) {
    return (*gridMarker)(cellIJK.x, cellIJK.y, cellIJK.z) >= 0;
}


void MACGrid::UpdateVelocityGridsByPressure() {

    float scale = DELTA_TIME / (float) DX;
    
    
    for (int i = 0; i < dimX ; i++) {
        for (int j = 0; j < dimY ; j++) {
            for (int k = 0; k < dimZ ; k++) {
            
                glm::ivec3 curU(i, j, k);
                glm::ivec3 neighbU = curU - gridU->gridDir; //get prev
    
                if (isFluidOrAir(curU) || isFluidOrAir(neighbU)) {
                    
                    if (isSolid(curU) || isSolid(neighbU)) {
                        gridU->setValueAt(0, curU.x, curU.y, curU.z);
                    }
                    else {
                        float pressureChange = (*gridP)(curU) - (*gridP)(neighbU);
                        gridU->addValueAt(-scale * pressureChange, i, j, k);
                    }
                }
            }
        }
    }
    
    
    
    for (int i = 0; i < dimX; i++) {
        for (int j = 0; j < dimY; j++) {
            for (int k = 0; k < dimZ; k++) {

                
                glm::ivec3 curV(i, j, k);
                glm::ivec3 neighbV = curV - gridV->gridDir;
                
                if (isFluidOrAir(curV) || isFluidOrAir(neighbV)) {
                    
                    if (isSolid(curV) || isSolid(neighbV)) {
                        gridV->setValueAt(0, curV.x, curV.y, curV.z);
                    } //set solid
                    
                    else {
                        
                        float pressureChange = (*gridP)(curV) - (*gridP)(neighbV);
                        gridV->addValueAt(-scale * pressureChange, i, j, k);
                    }
                    
                }
            }
        }
    }

    for (int i = 0; i < dimX ; i++) {
        for (int j = 0; j < dimY ; j++) {
            for (int k = 0; k < dimZ ; k++) {
                
                glm::ivec3 curW(i, j, k);
                glm::ivec3 neighbW = curW - gridW->gridDir; //get prev
                
                if (isFluidOrAir(curW) || isFluidOrAir(neighbW)) {
                    
                    if (isSolid(curW) || isSolid(neighbW)) {
                        gridW->setValueAt(0, curW.x, curW.y, curW.z);
                    }
                    
                    else {
                        float pressureChange = (*gridP)(curW) - (*gridP)(neighbW);
                        gridW->addValueAt(-scale * pressureChange, i, j, k);
                    }
                }
            }
        }
    }
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
            printf("[ %02i:  %i] ", i, gridMarker->data.at(i));
            //std::cout << "[" << i << ": " << gridMarker->data.at(i) << "]  ";
        }
        else if(gridMarker->data.at(i) == 0){
            printf("%03i:  %i   ", i, gridMarker->data.at(i));
        }
        else {
            printf("%03i: %i   ", i, gridMarker->data.at(i));
        }
    }
    std::cout << std::endl;
}

void MACGrid::printDimensions() {
    std::cout << "--- McGRID  ---" << std::endl;
    
    std::cout << "grid dimensions " << dimX << " x " << dimY << "  " << dimZ <<std::endl;
    std::cout << "grid size " << (dimX * dimY * dimZ) << std::endl;
    std::cout << "box bounds: " << bbX << " " << bbY << " " << bbZ << std::endl;

    
    
    std::cout << "-------" << std::endl;
    
    //std::cout << "particle bounds " << std::endl;
    
}