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
    

    

}


void MACGrid::addExternalForcesToGrids(glm::vec3 forces) {
    
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



bool MACGrid::isFluid(int i, int j, int k) {
    
    //(*gridMarker)(i, j, k)
    return (*gridMarker)(i, j, k) > 0;
}


void MACGrid::extrapolateVelocities() {
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
    
       float scale = DELTA_TIME/(DX * DX);
    
    
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





void MACGrid::UpdateVelocityGridsByPressure() {
    
    //subtract pressure value
    
    float scale = DELTA_TIME/DX;
    
    //ONLY DO THIS IF THERE IS AT LEAST ONE FLUID OUT OF BEFORE/BEHIND
    for (int i = 0; i < dimX; i++) {
        for (int j = 0; j < dimY; j++) {
            for (int k = 0; k < dimZ; k++) {
                
                int id;
                int id2;
                float pressureChange;
                
                //get P
                glm::vec3 cur(i, j, k);
                
                //std::cout << " u: previously this " << (*gridU)(i, j, k) << std::endl;
                
                glm::vec3 neighbU = cur - (glm::vec3) gridU->gridDir;
               /* if(isFluid(neighbU) ||  isFLUID(cur)) {
                    if (isSolid(neighbU) || isSOLID(cur)) {
                        velocity is just zero LOL
                    }
                    else {
                        currentcell velocity -= pressurechange * scale;
                    }
                    
                    //DEBUGGING
                    //if calculating divergence after updating for velocity
                    //RECALATE DIVERGENCE AND CHECK IF THEY'RE ALL ZERO
                    //possible bug cuases -- marking cells incorrectly or a matrix-building
                    //lol @ pressure ste[[a[sdpfkalsdfkaslfdj
                    
                }*/
                
                
                id = gridP->ijkToIndex(cur);
                id2 = gridP->ijkToIndex(neighbU);
                pressureChange = (*gridP)((glm::ivec3) cur) - (*gridP)((glm::ivec3) neighbU);
                gridU->addValueAt(-scale * pressureChange, i, j, k);

                //get P - 1
                //std::cout << " v: previously this: does this change " << (*gridV)(i, j, k) << std::endl;
                glm::vec3 neighbV = cur - (glm::vec3) gridV->gridDir;
                id = gridP->ijkToIndex(cur);
                id2 = gridP->ijkToIndex(neighbV);
                pressureChange = (*gridP)((glm::ivec3) cur) - (*gridP)((glm::ivec3) neighbV);
                gridV->addValueAt(-scale * pressureChange, i, j, k);
                //std::cout << " v: does this change " << (*gridV)(i, j, k) << std::endl;
                
               
                /*if ((*gridW)(i, j, k) > 0 ) {
                    std::cout << " w: previously this " << (*gridW)(i, j, k) << std::endl;
                }*/
         
                glm::vec3 neighbW = cur - (glm::vec3) gridW->gridDir;
                id = gridP->ijkToIndex(cur);
                id2 = gridP->ijkToIndex(neighbW);
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
    std::cout << " cell side: " << cellSidelength << std::endl;
    std::cout << " particle sep " << cellSidelength/2 << std::endl;
    
    
    std::cout << "-------" << std::endl;
    
    //std::cout << "particle bounds " << std::endl;
    
}