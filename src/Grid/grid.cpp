#include "Grid.h"
#include <algorithm>
#include <vector>
#include <iomanip>

#define DX 1.f

template<typename T>
Grid<T>::Grid(int dX, int dY, int dZ) {
    //take in dimensions
     dimX = dX;
     dimY = dY;
     dimZ = dZ;
    
    int notMaxDim = std::min(dimX, std::min(dimY, dimZ));
    
    //COLLISIONS???
    
    //CLAMP
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
bool Grid<T>::inGridBounds(glm::ivec3 perhaps) {
    
    return (perhaps.x < dimX && perhaps.x >= 0) && (perhaps.y < dimY && perhaps.y >= 0)
        && (perhaps.z < dimZ && perhaps.z >= 0);
}


//previously getNeighborPositions




template<typename T>
float Grid<T>::getInterpedVelocity(glm::vec3 pos, glm::vec3 offset, std::vector<float> deltas) {
    
    
    float velThis = 0;
    glm::vec3 gridPos = pos - offset;
    
    gridPos += glm::vec3(DX / 2.f, DX/2.f , DX /2.f);
    gridPos[axis] -= DX/2.f;
    
    
    bool printme = false;// = true;
    //gets grid ijk
    int x = floor(gridPos.x/DX);
    int y = floor(gridPos.y/DX);
    int z = floor(gridPos.z/DX);
    
    //gets grid into pos
   
    

   /* std::cout << " interp to MEEEE " << std::endl;
     std::cout << "    pos " << glm::to_string(pos) << std::endl;
     std::cout << "    amount to offset " << glm::to_string(offset) << std::endl;
     std::cout << "    gridPos " << glm::to_string(gridPos) << std::endl;
     std::cout << "    neighborpos " << glm::to_string(glm::vec3(x, y, z)) << std::endl;
    */
    
    float fract_partx = (gridPos[0] - x*DX)/DX;
    float fract_party = (gridPos[1] - y*DX)/DX;
    float fract_partz = (gridPos[2] - z*DX)/DX;
    
    
    float weight;
    float totalweight = 0;
    weight = (1-fract_partx)*(1-fract_party)*(1-fract_partz);
    velThis += weight * (*this)(x, y, z);
    if (printme)std::cout << weight << " * " << (*this)(x, y, z) << std::endl;
    totalweight += weight;
    
    weight = (fract_partx)*(1-fract_party)*(1-fract_partz);
    velThis += weight * (*this)(x+1, y, z);
    if (printme) std::cout << weight << " * " << (*this)(x + 1, y, z) << std::endl;
    totalweight += weight;
    
    
    weight = (fract_partx)*(fract_party)*(1-fract_partz);
    velThis += weight * (*this)(x +1, y +1, z);
    if (printme) std::cout << weight << " * " << (*this)(x + 1, y + 1, z) << std::endl;
    totalweight += weight;
    
    weight = (1-fract_partx)*(fract_party)*(1-fract_partz);
    velThis += weight * (*this)(x, y+1, z);
       if (printme) std::cout << weight << " * " << (*this)(x , y + 1, z) << std::endl;
    totalweight += weight;
    
    weight = (1-fract_partx)*(fract_party)*(fract_partz);
    velThis += weight * (*this)(x, y+1, z+1);
    if (printme)std::cout << weight << " * " << (*this)(x , y + 1, z+1) << std::endl;
    totalweight += weight;
    
    weight = (1-fract_partx)*(1-fract_party)*(fract_partz);
    velThis += weight * (*this)( x, y, z+1 );
        if (printme)   std::cout << weight << " * " << (*this)(x , y , z+1) << std::endl;
    totalweight += weight;
    
    weight = (fract_partx)*(1-fract_party)*(fract_partz);
    velThis += weight * (*this)(x+1, y, z+1);
        if (printme)std::cout << weight << " * " << (*this)(x + 1 , y , z+1) << std::endl;
    totalweight += weight;
    
    weight = (fract_partx)*(fract_party)*(fract_partz);
    velThis += weight * (*this)( x+1, y+1, z+1);
    if (printme) std::cout << weight << " * " << (*this)(x+1 , y + 1, z+1) << std::endl;
    totalweight += weight;
    
    return velThis;
    
}

template<typename T>
float Grid<T>::getDiv(int i, int j, int k) {
    glm::ivec3 here(i, j, k);
    
    glm::ivec3 there = here + (glm::ivec3) gridDir;
    
    int ind0 = ijkToIndex(here);
    int ind1 = ijkToIndex(there);
    
    
    if(inGridBounds(there)) {
        
        return data.at(ind1) - data.at(ind0);
    }
       
    else {
           return 0 - data.at(ind0);
    }
}



template<typename T>
int Grid<T>::ijkToIndex(glm::vec3 IJK) {
    return (IJK.z * dimX * dimY) + (IJK.y * dimX) + IJK.x;
}

template<typename T>
int Grid<T>::ijkToIndex(glm::ivec3 IJK) {
    return (IJK.z * dimX * dimY) + (IJK.y * dimX) + IJK.x;
}


template<typename T>
glm::ivec3 Grid<T>::posToIJK(glm::vec3 pos){
   
    //find closest multiple of cellSideLength
    int x = pos.x/DX;
    int y = pos.y/DX;
    int z = pos.z/DX;
    
    return glm::ivec3(x, y, z);
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
   
    i = i < 0 ? 0 : (i >= dimX ? dimX-1 : i);
    j = j < 0 ? 0 : (j >= dimY ? dimY-1 : j);
    k = k < 0 ? 0 : (k >= dimZ ? dimZ-1 : k);


    return data[k * dimX * dimY + j * dimX + i];
    
}

template<typename T>
float Grid<T>::operator()(glm::ivec3 gridIJK) {
    

    int i = gridIJK.x;
    int j = gridIJK.y;
    int k = gridIJK.z;
    //printf("grid IJK: %i %i %i \n", i, j, k);
    
    return (*this)(i, j, k);
}




template<typename T>
void Grid<T>::splatParticleVelocityToGrid(Particle par, glm::vec3 offset) {
    
    float pcomponent = par.speed[axis];
    glm::vec3 gridPos = par.pos - offset;
    
    //gets pos ready to be subtracted from neighbor
    gridPos += glm::vec3(DX / 2.f, DX/2.f , DX /2.f);
    gridPos[axis] -= DX/2.f;
    
    
    //gets  ijk in staggered grid
    int x = floor(gridPos.x/DX);
    int y = floor(gridPos.y/DX);
    int z = floor(gridPos.z/DX);
    

    /*std::cout << " splat MEEEE " << std::endl;
    std::cout << "    pos " << glm::to_string(par.pos) << std::endl;
    std::cout << "    amount to offset " << glm::to_string(offset) << std::endl;
    std::cout << "    gridPos " << glm::to_string(gridPos) << std::endl;
    std::cout << "    neighborpos " << glm::to_string(glm::vec3(x, y, z)) << std::endl;
    */
    float fract_partx = (gridPos[0] - x*DX);
    float fract_party = (gridPos[1] - y*DX);
    float fract_partz = (gridPos[2] - z*DX);
    
    //splatting to all neighbors that are/will be involved in trilerp
    float weight;
    
    weight = (1-fract_partx)*(1-fract_party)*(1-fract_partz);
    addValueAt(pcomponent * weight, x, y, z);
    addWeightAt(weight, x, y, z);


    weight = (fract_partx)*(1-fract_party)*(1-fract_partz);
    addValueAt(pcomponent * weight, x+1, y, z);
    addWeightAt(weight, x + 1, y, z);

    
    weight = (fract_partx)*(fract_party)*(1-fract_partz);
    addValueAt(pcomponent * weight, x+1, y+1, z);
    addWeightAt(weight, x+1, y+1, z);
    
    
    weight = (1-fract_partx)*(fract_party)*(1-fract_partz);
    addValueAt(pcomponent * weight, x, y+1, z);
    addWeightAt(weight,  x, y+1, z);
    
    weight = (1-fract_partx)*(fract_party)*(fract_partz);
    addValueAt(pcomponent * weight, x, y+1, z+1);
    addWeightAt(weight, x, y+1, z+1);

    
    weight = (1-fract_partx)*(1-fract_party)*(fract_partz);
    addValueAt(pcomponent * weight, x, y, z+1 );
    addWeightAt(weight, x, y, z+1);

    
    weight = (fract_partx)*(1-fract_party)*(fract_partz);
    addValueAt(pcomponent * weight, x+1, y, z+1);
    addWeightAt(weight, x+1, y, z+1);

    
    weight = (fract_partx)*(fract_party)*(fract_partz);
    addValueAt(pcomponent * weight, x+1, y+1, z+1);
    addWeightAt(weight, x+1, y+1, z+1);


}


template<typename T>
void Grid<T>::addValueAt(float value, int i, int j, int k)  {
    int gridIndex = ijkToIndex(glm::vec3(i, j, k));
    
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
void Grid<T>::addValueAt(int value, int gridIndex) {

    if (gridIndex < data.size()) {
        data[gridIndex] += value;
    }
    
}


template<typename T>
void Grid<T>::addWeightAt(float weight, int i, int j, int k) {
    int index = ijkToIndex(glm::vec3(i, j, k));

    
    if (index < totalWeights.size()) {
        totalWeights[index]+= weight;
    }
    
    /* if (index == 74  || index == 54) {
     std::cout << "weight added to "<< index << ": " << weight << std::endl;
     std::cout << "total weight now at "<< index << ": " << totalWeights[index] << std::endl;
     }*/
    
    
}



template<typename T>
void Grid<T>::ApplyBoundaryConditions() {
    
    for (int i = 0; i < dimX; i++) {
        for (int j = 0; j < dimY; j++) {
            for (int k= 0; k < dimZ; k++) {
                
                
                switch(axis) {
                    case 0: if (i <= 1 ||i >= dimX -2) {
                            setValueAt(0.f, ijkToIndex(glm::vec3(i, j, k)));
                    }
                        break;
                    case 1: if (j <= 1 ||j >= dimY -2) {
                        setValueAt(0.f, ijkToIndex(glm::vec3(i, j, k)));
                    }
                        break;
                    case 2: if (k <= 1 ||k >= dimZ -2) {
                        setValueAt(0.f, ijkToIndex(glm::vec3(i, j, k)));
                    }
                        break;
                        
                        
                }
            }
        }
    }
}

template<typename T>
void Grid<T>::setValueAt(float value, int i, int j, int k) {
    int gridIndex = ijkToIndex(glm::vec3(i, j, k));
    if (gridIndex < data.size()) {
        data[gridIndex] = value;
    }
}

template<typename T>
void Grid<T>::setValueAt(float value, int gridIndex) {

    if (gridIndex < data.size()) {
        data[gridIndex] = value;
    }
}


template<typename T>
void Grid<T>::pressureUpdate(Grid<float>* gridP) {
    
    for (int i = 0; i < dimX; i++) {
        for (int j = 0; j < dimY; j++) {
            for (int k= 0; k < dimZ; k++) {
                
                glm::vec3 one = glm::vec3(i, j, k);
                int gridIndex = ijkToIndex(one);
                float pressure1 = (*gridP)(i, j, k);
                
                glm::vec3 two = one - (glm::vec3) gridDir;
                
                float pressure2 = (*gridP)(two.x, two.y, two.z);
                float pressurechange = pressure2 - pressure1;

                data[gridIndex] -= pressurechange;
            }
        }
    }
    
}

template<typename T>
float Grid<T>::getDelta(int i, int j, int k) {
    
    
    int index = ijkToIndex(glm::vec3(i, j, k));
    if (index >= delta.size()) {
        return 0;
    }
    
    return delta[index];
}

template<typename T>
void Grid<T>::setDeltas(std::vector<float> calculatedDeltas) {
    delta = calculatedDeltas;
}




template<typename T>
void Grid<T>::extrapolateVelocities(Grid<int>* marker) {
    
    /*for (i)i = 1 to max(2, dkc f le)
        for each cell, C, such that C.layer == −1
            if C has a neighbor, N, such that N.layer == i−1
                for velocity components of C not bordering fluid cells, uj
                    set uj
                    to the average of the neighbors of C
                    in which N.layer == i−1.
                    C.layer = i*/
    
    //put velocity where fluids CAN GO
    //so that interpolation will work next step!!!
    //
    // tempgrid
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
    
 
}


//definition of class template functions

template<typename T>
inline
void Grid<T>::resetToZero() {
    std::fill(data.begin(), data.end(), 0);
}


template<typename T>
void Grid<T>::printContents(std::string message) {
    
    gridFile.open(message + ".txt", std::ofstream::app);
    
    if (gridFile.is_open())
    {
        gridFile<< message << "\n";
    }
    
    
    
    std::cout << message << std::endl;
    std::cout << dimX << " x " << dimY << " x " << dimZ << std::endl;
    
    
    for (int i = 0; i < data.size(); i++) {
        if (i > 0 && i % dimX == 0) {
            std::cout << std::endl;
            if (gridFile.is_open()) { gridFile<< "\n"; }
        }
        
        if (i % (dimY * dimX) == 0) {
            std::cout << std::endl;
            if (gridFile.is_open()) { gridFile<< "\n"; }
        }
        
        if (gridFile.is_open())
        {
            gridFile<< i << ": " << std::setprecision(3) << data.at(i) << "  ";
        }
        
        
        std::cout << i << ": " << std::setprecision(3) << data.at(i) << "  ";
    }
    
    
    
    std::cout << std::endl << "--- eog ---" << std::endl;
    
    gridFile.close();
}

template<typename T>
void Grid<T>::printDeltas(std::string message) {
    
    std::cout << message << std::endl;
    std::cout << dimX << " x " << dimY << " x " << dimZ << std::endl;
    
    
    for (int i = 0; i < data.size(); i++) {
        if (i > 0 && i % dimX == 0) {
            std::cout << std::endl;
        }
        
        if (i % (dimY * dimX) == 0) {
            std::cout << std::endl;
        }
        std::cout << i << ": " << delta.at(i) << "  ";
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



