#include "grid.h"
#include "MACGrid.h"


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
    
    for (int i = 0; i < dimX * dimY * dimZ; i++) {
        data.push_back(0);
    }
    
}

template<typename T>
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
    
}

template<typename T>
float Grid<T>::getKernelWeight(glm::vec3 offsetParticlePos, glm::vec3 staggeredPos) {
    /*return 1.0f/glm::distance(offsetParticlePos, staggeredPos);*/
}

template<typename T>
std::vector<glm::vec3> Grid<T>::getNeighborPositions(glm::vec3 particleIndex, glm::vec3 dir) {
    
    std::vector<glm::vec3> neighbors;
    
    glm::vec3 otherDir1 = glm::vec3(dir[1], dir[2], dir[0]);
    glm::vec3 otherDir2 = glm::vec3(dir[2], dir[0], dir[1]);

    neighbors.push_back(particleIndex);
    
    bool atbound1Dir1 = glm::dot(particleIndex, -otherDir1) <= 0;
    bool atbound2Dir1 = glm::dot(particleIndex, otherDir1) >= glm::dot(glm::vec3(dimX, dimY, dimZ), otherDir1);
    
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


//TODO GOTTA TEST THIS
//returns a vec3 of the velocity
template<typename T>
glm::vec3 Grid<T>::trilinearlyInterpolate(glm::vec3 pos, glm::vec3 offset) {

    glm::vec3 interpolatedVel;
    
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
        
        //i, j, k, is the forward direction, and otherIdx(i, j, k) is the backward direction;
        glm::vec3 otherIdx = indices;
        otherIdx[i] = indices[i] - 1;
        
        //get weight of position (in grid space) between idx-1 and idx
        float weightX = offsetPos[i]  -  otherIdx[i];
        
        //interpolate the velocity!
        interpolatedVel[i] =
                weightX * ((*this)(otherIdx.x, otherIdx.y, otherIdx.z))
                + (1 - weightX) * ((*this)(indices.x, indices.y, indices.z));
    }

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


template<typename T>
void Grid<T>::storeParticleVelocityToGrid(Particle p, glm::vec3 offset, glm::vec3 direction) {
    
    //calculate i j k on this staggered grid
    glm::vec3 gridIndices = getGridIndices(p.pos);

    
    float pcomponent = glm::dot(direction, p.speed);
    
    std::vector<glm::vec3> neighborPositions = getNeighborPositions(gridIndices, direction);
    
 
    for (glm::vec3 neighborPos : neighborPositions) {

        float kernelWeight = getKernelWeight(p.pos - offset, neighborPos);
        int neighborIndex = getGridIndexFromIJK(neighborPos);
        //std::cout << " me index " << glm::to_string(gridIndices) << " " << p.gridIndex << std::endl;
        //std::cout << "neighbor index " << neighborIndex << std::endl;
        if (neighborIndex < data.size()) {
            data.at(neighborIndex) += kernelWeight * pcomponent;
        }
    }
    

    
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
        data[gridIndex] = value;
    }

}

template<typename T>
void Grid<T>::printContents() {
    
    std::cout << "--- printing " << dimX << " x " << dimY << " x " << dimZ << " grid---" << std::endl;
    
    for (int i = 0; i < data.size(); i++) {
        std::cout << data[i]<< " ";
        if (i % dimY == 0) {
            std::cout <<std::endl;
        }
        
        if (i % (dimY * dimZ) == 0)  {
            std::cout << std::endl;
        }
    }
    
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


//tells compiler ahead of time what classes will be used
template class Grid<int>;
template class Grid<float>;



