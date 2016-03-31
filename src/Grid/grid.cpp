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
    backwardsDir = glm::vec3(dimX - notMaxDim, dimY - notMaxDim, dimZ - notMaxDim);
    axis = 0;
    for (axis = 0; axis < 3; axis++) {
        if (backwardsDir[axis] == 1) {
            break;
        }
    }
    
    
    for (int i = 0; i < dimX * dimY * dimZ; i++) {
        data.push_back(0);
    }
    
}

/*template<typename T>
void Grid<T>::storeParticlesToGrid(std::map<int,
                                   std::vector<Particle>>* particlesByIndex,
                                   glm::vec3 offset) {
    
    //loop through each particle
    for (int i = 0; i < dimX; i++) {
        for (int j = 0; j < dimY; j++) {
            for (int k = 0; k < dimZ; k++) {
                
                //do i need to add stuff to this and stuff
                glm::vec3 staggeredPos = glm::vec3(i, j, k);
                
                glm::vec3 newVel;
                
                std::vector<glm::vec3> neighborIndices =
                    getNeighborPositions(staggeredPos, glm::vec3(1, 1, 1));
                
                for (glm::vec3 neighborPos : neighborIndices) {
                    int neighborIndex = getGridIndexFromIJK(neighborPos);
                    std::vector<Particle> neighborParticles =
                        (*particlesByIndex)[neighborIndex];
                    
                    for (Particle p : neighborParticles) {
                        float kernelWeight = getKernelWeight(p.pos - offset,
                                                             glm::vec3(i, j, k));
                        newVel += kernelWeight * p.speed;
                    }
                }
            
            }
            
        }
    }
    
}*/

template<typename T>
float Grid<T>::getKernelWeight(float pcomponent, glm::vec3 offsetParticlePos, glm::vec3 staggeredGridPos, float W) {
    float kerneldist = glm::distance(offsetParticlePos, staggeredGridPos);
    
    ///might want to replace this with specific kernel dist
    /// in the specified direction
   
    if (kerneldist > 0) {
         float hX = pcomponent/kerneldist;
        if (pcomponent != 0) {
            std::cout << pcomponent << "  / " << kerneldist << std::endl;
            std::cout << "hX : " << hX << std::endl;
        }
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
    
    return 1.0f/glm::distance(offsetParticlePos, staggeredGridPos);
}

template<typename T>
std::vector<glm::vec3> Grid<T>::getNeighborPositions(glm::vec3 particleIndex, glm::vec3 dir) {
    
    std::vector<glm::vec3> neighbors;
    
    glm::vec3 otherDir1 = glm::vec3(dir[1], dir[2], dir[0]);
    glm::vec3 otherDir2 = glm::vec3(dir[2], dir[0], dir[1]);

    neighbors.push_back(particleIndex);
    
    bool atbound1Dir1 = glm::dot(particleIndex, -otherDir1) <= 0;
    bool atbound2Dir1 = glm::dot(particleIndex, otherDir1) >=
            glm::dot(glm::vec3(dimX, dimY, dimZ), otherDir1);
    
    if(!atbound1Dir1) {
        neighbors.push_back(particleIndex - otherDir1);
    }
    
    if(!atbound2Dir1) {
        neighbors.push_back(particleIndex + otherDir1);
    }
    
    
    
    if (glm::dot(particleIndex, -dir) > 0) {
        neighbors.push_back(particleIndex - dir);
        
        if(glm::dot(particleIndex, -otherDir1) > 0) {
            neighbors.push_back(particleIndex - otherDir1 - dir);
        }
        
        if (glm::dot(particleIndex, otherDir1) < glm::dot(glm::vec3(dimX, dimY, dimZ), otherDir1)) {
        neighbors.push_back(particleIndex + otherDir1 - dir);
        }
        
        if(glm::dot(particleIndex, -otherDir2) > 0) {
            neighbors.push_back(particleIndex - otherDir2 - dir);
        }

        if (glm::dot(particleIndex, otherDir2) < glm::dot(glm::vec3(dimX, dimY, dimZ), otherDir2)) {
            neighbors.push_back(particleIndex + otherDir2 - dir);
        }
        
    }
    
    bool atbound1Dir2 = glm::dot(particleIndex, -otherDir2) <= 0;
    bool atbound2Dir2 = glm::dot(particleIndex, otherDir2) >= glm::dot(glm::vec3(dimX, dimY, dimZ), otherDir2);
    
    if(!atbound1Dir2) {
        neighbors.push_back(particleIndex - otherDir2);
    }
    
    if (!atbound2Dir2) {
        neighbors.push_back(particleIndex + otherDir2);
    }
    
    
  if(glm::dot(particleIndex, dir) < glm::dot(glm::vec3(dimX, dimY, dimZ), dir)) {
      neighbors.push_back(particleIndex + dir);
      
      if(glm::dot(particleIndex, -otherDir1) > 0) {
          neighbors.push_back(particleIndex - otherDir1 + dir);
      }
      
      if (glm::dot(particleIndex, otherDir1) < glm::dot(glm::vec3(dimX, dimY, dimZ), otherDir1)) {
          neighbors.push_back(particleIndex + otherDir1 + dir);
      }
      

      
      if(!atbound1Dir2) {
          neighbors.push_back(particleIndex - otherDir2 + dir);
      }
      
      if (!atbound2Dir2) {
          neighbors.push_back(particleIndex + otherDir2 + dir);
      }
      
      
  }
    
   
    
   // neighbors.push_back(particleIndex + otherDir1 + otherDir2);
    
/*
    neighbors.push_back(particleIndex);
    
    if (glm::dot(particleIndex, direction) > 0) {
       neighbors.push_back(particleIndex - direction);
    }
    
    if(glm::dot(particleIndex, direction) < glm::dot(glm::vec3(dimX, dimY, dimZ), direction)){
        
    }

    
    
    /*neighbors.push_back(glm::vec3(i, j, k));
    neighbors.push_back(glm::vec3(i, j - 1, k));
    neighbors.push_back(glm::vec3(i, j + 1, k));
    
    neighbors.push_back(glm::vec3(i - 1, j, k));
    neighbors.push_back(glm::vec3(i - 1, j - 1, k));
    neighbors.push_back(glm::vec3(i - 1, j + 1, k));
    
    neighbors.push_back(glm::vec3(i, j, k + 1));
    neighbors.push_back(glm::vec3(i, j - 1, k + 1));
    neighbors.push_back(glm::vec3(i, j + 1, k + 1));
    neighbors.push_back(glm::vec3(i - 1, j, k + 1));
    neighbors.push_back(glm::vec3(i - 1, j - 1, k + 1));
    neighbors.push_back(glm::vec3(i - 1, j + 1, k + 1));
    
    neighbors.push_back(glm::vec3(i, j, k - 1));
    neighbors.push_back(glm::vec3(i, j - 1, k - 1));
    neighbors.push_back(glm::vec3(i, j + 1, k - 1));
    neighbors.push_back(glm::vec3(i - 1, j, k - 1));
    neighbors.push_back(glm::vec3(i - 1, j - 1, k - 1));
    neighbors.push_back(glm::vec3(i - 1, j + 1, k - 1));*/
    
    return neighbors;
}


template<typename T>
int Grid<T>::getGridIndexFromPosition(glm::vec3 position) {
    
    //WE ALL LOVE FLOATING POINT ERRORS
    float epsilon = 0.001;
    

    //find closest multiple of cellSideLength
    int x = (position.x + epsilon)/cellSidelength;
        //std::cout << x<< "  " << position.x <<  "  /"  << cellSidelength << std::endl;
    int y = (position.y + epsilon)/cellSidelength;
    int z = (position.z + epsilon)/cellSidelength;
    

    //give a 1D location for 3D index
    return (z * dimX * dimY) + (y * dimX) + x;
    
}

template<typename T>
int Grid<T>::getGridIndexFromIJK(glm::vec3 IJK) {


    //give a 1D location for 3D index
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


//TODO GOTTA TEST THIS
//returns a vec3 of the velocity
template<typename T>
float Grid<T>::trilinearlyInterpolate(glm::vec3 pos, glm::vec3 offset) {

    float interpolatedVel;
    
    //get offset to put into gridSpace
    glm::vec3 offsetPos = pos + offset;
    glm::vec3 gridIndices = posToGridIJK(offsetPos, cellSidelength);
    

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
    return interpolatedVel;

}

template<typename T>
float Grid<T>::operator()(int i, int j, int k) {
    if (k*dimX*dimY + j*dimX + i >= data.size()) {
        return 0;
    }
    else {
        return data.at(k * dimX * dimY + j + dimX + i);
    }
}



template<typename T>
glm::vec3 Grid<T>::getGridIndices(glm::vec3 pos) {
    float epsilon = 0.001;
    
    int xidx = (pos.x + epsilon)/cellSidelength;
    int yidx = (pos.y + epsilon)/cellSidelength;
    int zidx = (pos.z + epsilon)/cellSidelength;
    
    glm::vec3 indices = glm::vec3(xidx, yidx, zidx);
    
    return indices;
    
}


void printNeighbors(std::vector<glm::vec3> neighborPositions) {
    std::cout << "my neighbors" << std::endl;
    for (glm::vec3 neighbor : neighborPositions) {
        std::cout << glm::to_string(neighbor) << std::endl;
    }
    std::cout << std::endl;
    
}

void printVector(glm::vec3 index, std::string message) {
    std::cout << message << " : " << glm::to_string(index) << std::endl;
    
}


template<typename T>
void Grid<T>::storeParticleVelocityToGrid(Particle p, glm::vec3 offset, glm::vec3 direction, float W) {
    
    //calculate IJK on staggered grid
    glm::vec3 gridIndices = getGridIndices(p.pos);
    
    //get direction of p that we want to splat
    float pcomponent = glm::dot(direction, p.speed);
    if (pcomponent != 0) {
        std::cout << pcomponent << " WOOOO  " << std::endl; 
    }
    
    //get neighbors
    std::vector<glm::vec3> gridNeighbors = getNeighborPositions(gridIndices, direction);

    //splat onto neighbors
    for (glm::vec3 gridIJK : gridNeighbors) {
        int gridIndexInArray = getGridIndexFromIJK(gridIJK); //kernelweight
        float kernelWeight = getKernelWeight(pcomponent, p.pos - offset, gridIJK, W); //send offsetPos, grid
        if (kernelWeight >0) {
            std::cout << "kernel weight " << kernelWeight << std::endl;
            
        }
        if (gridIndexInArray < data.size()) {
            data.at(gridIndexInArray) += kernelWeight * pcomponent; //add to each neighbor
        }
    }
    std::cout << std::endl;

}



template<typename T>
void Grid<T>::colorSplattedFaces(Particle p) {
    
}


//might just fuse this with resetToZero
template<typename T>
void Grid<T>::addForce(float f) {

    for (int i = 0 ; i < data.size(); i++) {
        data[i] += f;
        if (data[i] > 100) {
            std::cout << " WHAT HAPPENED " << std::endl;
        }
    }
}

template<typename T>
void Grid<T>::addValueAt(float value, int gridIndex) {
    
    if (gridIndex < data.size()) {
        data[gridIndex] += value;
    }

}

template<typename T>
void Grid<T>::printContents(std::string message) {
    
    std::cout << message << std::endl;
    std::cout << dimX << " x " << dimY << " x " << dimZ << std::endl;
    
    int largerDim = dimX;
    switch(axis) {
        case 1: largerDim = dimY; break;
        case 2: largerDim = dimZ; break;
        default: dimX;
    }
    

    for (int i = data.size() - dimX; i >= 0; i++) {
        if (i % dimX == 0) {
            std::cout << std::endl;
        }
        
        std::cout << "i: " << i << "  "<< data[i] << " " ;
        
        if (i > 0 && ((i + 1) % dimX == 0)) {
            i -= (dimX + 2);
        }
        
    }
    
/*std::vector<std::vector<int>> toPrint;

    
    for (int i = 0; i < data.size(); i++) {
        std::vector<int> printRow;
        
        if (i > 0 && (i % largerDim == 0)) {
           // toPrint.push_back(printRow);
           // printRow.clear();
            std::cout << std::endl;
        }
        
        if (i > 0 && i % (dimY * dimZ) == 0)  {
            std::cout << std::endl;
        }
        
        //std::cout << i << ": " << data[i] <<" " << dimX << " " << (i % dimY) << " " << (i % dimZ);
        //printRow.push_back(data[i]);
        std::cout << i << ": " << data[i] << "   ";
        
    }
    
    /*for (int i = 0; i < toPrint.size(); i++) {
        std::vector<int> printRow = toPrint[i];
        for (int j = 0; j < printRow.size(); j++) {
            std::cout << printRow[j] << " " << std::endl;
        }
    }*/
    
    std::cout << std::endl << "--- end of grid ---" << std::endl;
}

template<typename T>
void Grid<T>::extrapolateVelocities() {
    for (float cell : data) {
        /*If (g is solidOrEmptyMark AND g is not fluidMark)
        foreach face of g, do //neighboring cell
            If neighourGridCells for any face has fluidMark
                find averageGridVelocity from neighbor cells*/
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




//tells compiler ahead of time what classes will be used
template class Grid<int>;
template class Grid<float>;



