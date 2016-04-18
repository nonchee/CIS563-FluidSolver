#include "Grid.h"
#include <algorithm>
#include <vector>
#include <iomanip>

#define DX 1.f

template<typename T>
Grid<T>::Grid(int dX, int dY, int dZ) {
    //take in dimensions
     dimX = dX; dimY = dY; dimZ = dZ;
    
    int notMaxDim = std::min(dimX, std::min(dimY, dimZ));
    
    gridDir = glm::vec3(dimX - notMaxDim, dimY - notMaxDim, dimZ - notMaxDim);
    axis = 0;
    for (axis = 0; axis < 3; axis++) {
        if (gridDir[axis] > 0) {
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

template<typename T>
float Grid<T>::getInterpedVelocity(glm::vec3 pos, glm::vec3 offset, std::vector<float> deltas) {
    
    
    float velThis = 0;
    glm::vec3 gridPos = pos - offset;
    
    gridPos -= glm::vec3(DX / 2.f, DX/2.f , DX /2.f);
    gridPos[axis] += DX/2.f;
    
    
    bool printme = false;
    //gets grid ijk
    int x = floor(gridPos.x/DX); ///DX;
    int y = floor(gridPos.y/DX);//DX;
    int z = floor(gridPos.z/DX);//DX;


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
    
    int ind0 = ijkToIndex(here); //?
    int ind1 = ijkToIndex(there); //?
    
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
glm::ivec3 Grid<T>::posToStaggeredIJK(glm::vec3 pos){
    
    pos -= glm::vec3(DX / 2.f, DX/2.f , DX /2.f);
    pos[axis] += DX/2.f;
    
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
    
    float pcomponent = par.speed[axis];//glm::vec3(0, -1, 0)[axis];  //
    glm::vec3 gridPos = par.pos - offset;
    
    //gets pos ready to be subtracted from neighbor
    gridPos -= glm::vec3(DX / 2.f, DX/2.f , DX /2.f);
    gridPos[axis] += DX/2.f;
    
    /*if (par.gridIndex == 154 && axis == 1) {
        printf("STOP");
    }*/
    
    //gets  ijk in staggered gr
    
    /*std::cout << "lol what x" << (gridPos.x/DX) << std::endl;
    std::cout << "lol what y" << (gridPos.y/DX) << std::endl;
    */
    int x = floor(gridPos.x/DX); //DX;
    int y = floor(gridPos.y/DX); //DX;
    int z = floor(gridPos.z/DX);

    /*std::cout << " splat MEEEE " << par.gridIndex << std::endl;
    std::cout << "    pos " << glm::to_string(par.pos) << std::endl;
    std::cout << "    amount to offset " << glm::to_string(offset) << std::endl;
    std::cout << "    gridPos " << glm::to_string(gridPos) << std::endl;
    std::cout << "    neighborpos " << glm::to_string(glm::vec3(x, y, z)) << std::endl;
    std::cout << " what if i just used " << glm::to_string(posToIJK(gridPos)) << std::endl;
    std::cout << " or even just " << glm::to_string(posToIJK(par.pos - offset)) << std::endl;
    */
    
    float fract_partx = (gridPos[0] - x*DX)/(float) DX;
    float fract_party = (gridPos[1] - y*DX)/(float)DX;
    float fract_partz = (gridPos[2] - z*DX)/(float) DX;
    
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
    

    
 
}


//definition of class template functions

template<typename T>
inline
void Grid<T>::resetToZero() {
    std::fill(data.begin(), data.end(), 0);
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

        
        
        std::cout << i << ": " << std::setprecision(3) << data.at(i) << "  ";
    }
    
    
    
    std::cout << std::endl << "--- eog ---" << std::endl;
    

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



