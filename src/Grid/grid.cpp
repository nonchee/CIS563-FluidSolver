#include "grid.h"
#include "MACGrid.h"
#include <algorithm>
#include <vector>

template<typename T>
Grid<T>::Grid() {
    
}

template<typename T>
Grid<T>::Grid(int bx, int by, int bz, float side) {
    //take in dimensions
     dimX = bx;
     dimY = by;
     dimZ = bz;
     cellSidelength = side;
    
    int notMaxDim = std::min(dimX, std::min(dimY, dimZ));
    
    //will be 1 in the direction that has an extra size
    //tells use the direction that this grid takes care of
    //ie a 2 3 2 grid ends up with a glmvec3 0 1 0
    gridDir = glm::vec3(dimX - notMaxDim, dimY - notMaxDim, dimZ - notMaxDim);
    axis = 0;
    for (axis = 0; axis < 3; axis++) {
        if (gridDir[axis] == 1) {
            break;
        }
    }
    
    
    for (int i = 0; i < dimX * dimY * dimZ; i++) {
        data.push_back(0);
    }
    
}



template<typename T>
float Grid<T>::getKernelWeight(float pcomponent,
                               glm::vec3 offsetParticlePos,
                               glm::ivec3 neighborIJK,
                               float W) {
    
    float kerneldist = glm::distance(offsetParticlePos, (glm::vec3) neighborIJK);
    if (kerneldist > 0) {

        float hX = kerneldist/ (float) cellSidelength;

        if (hX >= 0 && hX <= 1) {
            return (1-hX)/W;
        }
        
        else if (hX >= -1 && hX <= 0) {
            return (1 + hX)/W;
        }
        else {
            return 0;
        }
    }
    
    return 1.0f/kerneldist;
}

template<typename T>
bool Grid<T>::inGridBounds(glm::ivec3 perhaps) {
    return (perhaps.x < dimX && perhaps.x >= 0) && (perhaps.y < dimY && perhaps.y >= 0)
        && (perhaps.z < dimZ && perhaps.z >= 0);
}


//previously getNeighborPositions

template<typename T>
//give grid direction to check bounds
std::vector<glm::ivec3> Grid<T>::getTrilinNeighbors(glm::ivec3 pIJK) {
    
    std::vector<glm::ivec3> cellsToSplat;
    cellsToSplat.push_back(pIJK);
    
    for (int axis = 0; axis < 3; axis++) {
        for (int dir = -1; dir < 2; dir+=2) {
            glm::ivec3 neighbDir(0, 0, 0);
            //std::cout << "possible neighbor Dir " << glm::to_string(neighbDir) << std::endl;
            neighbDir[axis] = dir;
            
            if (inGridBounds(pIJK + neighbDir)) {
                cellsToSplat.push_back(pIJK + neighbDir);
            }
        }
    }
    return cellsToSplat;
}




//@params ijk of the grid's cube       //@return a 1D vector index
template<typename T>
int Grid<T>::ijkToGridIndex(glm::vec3 IJK) {
    return (IJK.z * dimX * dimY) + (IJK.y * dimX) + IJK.x;
}


//@params ijk of the grid's cube       //@return a 1D vector index
template<typename T>
int Grid<T>::ijkToGridIndex(glm::ivec3 IJK) {
    return (IJK.z * dimX * dimY) + (IJK.y * dimX) + IJK.x;
}


glm::vec3 posToGridIJK(glm::vec3 pos, float cellSidelength){
    //because a nancy never forgets
    float epsilon = 0.001;
    
    //find the i, j, and k indices
    int xidx = (pos.x + epsilon)/cellSidelength;
    int yidx = (pos.y + epsilon)/cellSidelength;
    int zidx = (pos.z + epsilon)/cellSidelength;
    
    return glm::vec3(xidx, yidx, zidx);
}


//trilinearly interpolate for each value
template<typename T>
float Grid<T>::trilinearlyInterpolate(glm::vec3 pos, glm::vec3 offset) {
    float interpolatedVel;
    
    //get offset to put into gridSpace
    glm::vec3 offsetPos = pos + offset;
    
    //because a nancy never forgets
    float epsilon = 0.001;
    
    //find the i, j, and k indices
    int xidx = (pos.x + epsilon)/cellSidelength;
    int yidx = (pos.y + epsilon)/cellSidelength;
    int zidx = (pos.z + epsilon)/cellSidelength;
    
    glm::vec3 indices = glm::vec3(xidx, yidx, zidx);
    
    for (int i = 0; i < 3; i++) {
        
        //get closest idx on axis
        int idx = indices[i];
        
        //i, j, k, is the prior to direction, and otherIdx(i, j, k) is the before direction;
        glm::vec3 otherIdx = indices;
        otherIdx[i] = indices[i] + 1;
        
        //get weight of position (in grid space) between idx-1 and idx
        float weightX = offsetPos[i]  -  otherIdx[i];
        
        //interpolate the velocity!
        interpolatedVel = weightX * ((*this)(otherIdx.x, otherIdx.y, otherIdx.z))
        + (1 - weightX) * ((*this)(indices.x, indices.y, indices.z));
    }
    
    //test to make sure this is some sort of (x, 0, 0) (0, y, 0) or (0, 0, z)
    return interpolatedVel;
    
    
    /*float interpolatedVel;
    
    //get offset to put into gridSpace
    glm::vec3 offsetPos = pos + offset;
    glm::vec3 gridIndices = posToGridIJK(offsetPos, cellSidelength);
    
    
    int i = floor(offsetPos.x);
    int j = floor(offsetPos.y);
    int k = floor(offsetPos.z);
    
    /*return (i+1-offsetPos.x) * (j+1-offsetPos.y) * (k+1-offsetPos.z) * cell(i, j, k).u[index] +
    (x-i) * (j+1-y) * (k+1-z) * cell(i+1, j, k).u[index] +
    (i+1-x) * (y-j) * (k+1-z) * cell(i, j+1, k).u[index] +
    (x-i) * (y-j) * (k+1-z) * cell(i+1, j+1, k).u[index] +
    (i+1-x) * (j+1-y) * (z-k) * cell(i, j, k+1).u[index] +
    (x-i) * (j+1-y) * (z-k) * cell(i+1, j, k+1).u[index] +
    (i+1-x) * (y-j) * (z-k) * cell(i, j+1, k+1).u[index] +
    (x-i) * (y-j) * (z-k) * cell(i+1, j+1, k+1).u[index];
    

    //i, j, k, is the forward direction, and otherIdx(i, j, k)
    //is the backward direction;
    glm::vec3 prevIndices = gridIndices;
    prevIndices = gridIndices - backwardsDir;
    float backwardsVal = (*this)(prevIndices.x, prevIndices.y, prevIndices.z);
    float forwardsVal = (*this)(gridIndices.x, gridIndices.y, gridIndices.z);
    

    
    //get weight of position (in grid space) between idx-1 and idx
    float weightX = offsetPos[axis] - prevIndices[axis];
        
    //interpolate the velocity!
    interpolatedVel = weightX * backwardsVal + (1 - weightX) * forwardsVal;

    //test to make sure this is some sort of (x, 0, 0) (0, y, 0) or (0, 0, z)
    return interpolatedVel;*/

}

template<typename T>
void Grid<T>::printGridValueAt(std::string s, int i, int j, int k) {
    if (k*dimX*dimY + j*dimX + i >= data.size()) {
        std::cout << (k*dimX*dimY + j*dimX + i) << " lol called printgridval but u out of bounds fool " << std::endl;
        return;
    }
    
    std::cout << s << " " << i << " " << k << " " << k << " (" << (k*dimX*dimY + j*dimX + i) << ") ";
    std::cout << data.at(k * dimX * dimY + j * dimX + i) << std::endl;
    
}



template<typename T>
float Grid<T>::operator()(int i, int j, int k) {
    if (k*dimX*dimY + j*dimX + i >= data.size()) {
        
        /*std::cout << (k*dimX*dimY + j*dimX + i) << " lol u out of bounds fool " << std::endl;
        std::cout << "  " << i <<  " " << j << " " <<k <<  " need to be within ";
        std::cout << "  " << dimX <<  " " << dimY << " " << dimZ <<  std::endl;*/
        return 0;
    }
    else  {
        return data.at(k * dimX * dimY + j * dimX + i);
    }
}






template<typename T>
void Grid<T>::storeParticleVelocityToGrid(Particle p, glm::vec3 offset, float W) {
    
    float pcomponent = glm::dot(gridDir, p.speed);
    if (pcomponent == 0) {
        return; //if zero in this dir then nah
    }
    
    glm::vec3 offsetPos = ((glm::vec3) p.gridIJK) - offset;

    //get neighbors of particle in grid
    //find for now because never at grid bounds
    std::vector<glm::ivec3> cellsToSplat = getTrilinNeighbors(p.gridIJK);
    

    //splat onto neighbors
    for (glm::ivec3 neighborCell : cellsToSplat) {
        int neighbor1DIndex = ijkToGridIndex(neighborCell);
        float kernelWeight = getKernelWeight(pcomponent, offsetPos, neighborCell, W);
        std::cout << glm::to_string(p.gridIJK) << std::endl;
        std::cout << "neighbor ind " << neighbor1DIndex << std::endl;
        data.at(neighbor1DIndex) += kernelWeight * pcomponent;
    }

}


//might just fuse this with resetToZero
template<typename T>
void Grid<T>::updateVel(int i, int j, int k, float delU) {
    int index = ijkToGridIndex(glm::vec3(i, j, k));
    data[index] += delU;
    
}

//oops rather redundant
template<typename T>
void Grid<T>::addValueAt(float value, int i, int j, int k)  {
    int gridIndex = ijkToGridIndex(glm::vec3(i, j, k));
    
    if (gridIndex < data.size()) {
        data[gridIndex] += value;
    }
}




template<typename T>
void Grid<T>::addValueAt(float value, int gridIndex) {
    if (gridIndex < data.size()) {
        data[gridIndex] += value;
    }
}



template<typename T>
void Grid<T>::setValueAt(float value, int gridIndex) {
    
    if (gridIndex < data.size()) {
        data[gridIndex] = value;
    }
}

template<typename T>
void Grid<T>::setValueAt(float value, int i, int j, int k) {
    int gridIndex = ijkToGridIndex(glm::vec3(i, j, k));
    if (gridIndex < data.size()) {
        data[gridIndex] = value;
    }
}

template<typename T>
void Grid<T>::pressureUpdate(int index, float scale) {
    //int index = getGridIndexFromIJK(glm::vec3(i, j, k));
   // int forwardNeighborIndex = index;
    //int backwardNeighborIndex = index; // - axisValue;
    //float changeInPressure = data[forwardNeighborIndex] - data[backwardNeighborIndex];
    data[index] = data[index] - scale;
}

template<typename T>
void Grid<T>::pressureUpdate(Grid<float>* gridP) {
    for (int i = 0; i < dimX; i++) {
        for (int j = 0; j < dimY; j++) {
            for (int k= 0; k < dimZ; k++) {
                
                glm::vec3 one = glm::vec3(i, j, k);
                int gridIndex = ijkToGridIndex(one);
                float pressure1 = (*gridP)(i, j, k);
                
                glm::vec3 two = one - gridDir;
               // std::cout << "two " << glm::to_string(two) << std::endl;
                
                float pressure2 = (*gridP)(two.x, two.y, two.z);
                float pressurechange = pressure2 - pressure1;
                /*std::cout << "pressure 1 " << pressure2 << std::endl;
                std::cout << "pressure 2 " << pressure1 << std::endl;
                std::cout << "pressure change " << pressurechange << std::endl;
                */
                data[gridIndex] -= pressurechange;
            }
        }
    }
    
    std::cout << std::endl;
}

template<typename T>
float Grid<T>::getDelta(int i, int j, int k) {
    
    
    int index = ijkToGridIndex(glm::vec3(i, j, k));
    if (index >= delta.size()) {
        //std::cout << index << " was out of delta bounds " << delta.size() << std::endl;
        return 0;
    }
    
    //std::cout << index<< std::endl;
    //std::cout << delta.size() << std::endl;
    return delta[index];
}

template<typename T>
void Grid<T>::setDeltas(std::vector<float> calculatedDeltas) {
    //std::cout << " calculated deltas! " << calculatedDeltas.size() << std::endl;
    delta = calculatedDeltas;
    //std::cout << " deltas! " << delta.size() << std::endl;
}




template<typename T>
void Grid<T>::extrapolateVelocities(Grid<int>* marker) {

    std::vector<T> oldData(data);
    
    for (int i = 0; i < dimX; i++) {
        for (int j = 0; j < dimY; j++) {
            for (int k = 0; k < dimZ; k++) {
               
                //if empty or solid
                if ((*marker)(i, j, k) <= 0) {
                    
                    float totalVel = 0;
                    
                    int gridIndex = ijkToGridIndex(glm::vec3(i, j, k));
                    
                    ///printVector(glm::vec3(i, j,k), "me");
                    //std::cout << " my index " << gridIndex << std::endl;
                    
                    
                    std::vector<glm::ivec3> neighbors = getTrilinNeighbors(glm::ivec3(i, j, k));
                    int numNeighbors = 0;
                    
                    for (glm::ivec3 neighbor : neighbors) {
                        //printVector((glm::vec3)neighbor, "  was neighbor");
                        //if neighbor was fluid, get its vel and add to neighbcount
                        if ((*marker)(neighbor.x, neighbor.y, neighbor.z) >=0) {
                            totalVel+= oldData.at(ijkToGridIndex(neighbor));
                            //std::cout << "    new total vel " << totalVel << "from " << ijkToGridIndex(neighbor) << std::endl;
                            numNeighbors++;
                        }
                        
                    }
                    
                    //can't just use neighbrs.size() because trilin is quite general
                    if (numNeighbors > 0) {
                       
                       // std::cout << "  pls extrap " << totalVel/(float)numNeighbors << " to grid index " << gridIndex << std::endl;
                        //std::cout << "  this many fluid neighbors " << numNeighbors  <<std::endl<< std::endl;
                        data.at(gridIndex) = totalVel/(float)numNeighbors;
                    }
                }
            }
        }
    }
}


//definition of class template functions

template<typename T>
inline
void Grid<T>::resetToZero() {
    data.clear();
    
    for (int i= 0; i < dimX * dimY * dimZ; i++) {
        data.push_back(0);
    }
    
}


template<typename T>
void Grid<T>::printContents(std::string message) {
    
    std::cout << message << std::endl;
    std::cout << dimX << " x " << dimY << " x " << dimZ << std::endl;
    
    
    for (int i = 0; i < data.size(); i++) {
        if (i > 0 && i % dimX == 0) {
            std::cout << std::endl;
        }
        
        if (i % (dimY * dimX) == 0) {
            std::cout << std::endl;
        }
        std::cout << i << ": " << data.at(i) << "  ";
    }
    
    std::cout << std::endl << "--- eog ---" << std::endl;
}

template<typename T>
void Grid<T>::printNeighbors(std::vector<glm::ivec3> neighborIndices) {
    std::cout << "my neighbors" << std::endl;
    for (glm::ivec3 neighbor : neighborIndices) {
        std::cout << glm::to_string(neighbor) << std::endl;
    }
    std::cout << std::endl;
    
}

template<typename T>
void Grid<T>::printVector(glm::vec3 index, std::string message) {
    std::cout << message << " : " << glm::to_string(index) << std::endl;
}


//tells compiler ahead of time what classes will be used
template class Grid<int>;
template class Grid<float>;



