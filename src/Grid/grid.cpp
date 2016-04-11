#include "grid.h"
#include "MACGrid.h"
#include <algorithm>
#include <vector>

#include <iomanip>


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
    
    for (int dimension = 0; dimension < 3; dimension++) {
        for (int dir = -1; dir < 2; dir+=2) {
            glm::ivec3 neighbDir(0, 0, 0);

            if (dimension == axis && dir != -1) {
                neighbDir[dimension] = dir;
            
            
                if (inGridBounds(pIJK + neighbDir)) {
                    cellsToSplat.push_back(pIJK + neighbDir);
                }
            }
        }
    }
    return cellsToSplat;
}
/*
template<typename T>
//give grid direction to check bounds
float Grid<T>::getInterpedVelocity(glm::vec3 ppos, glm::vec3 poffset) {//(glm::vec3 pos, glm::vec3 offset) { //offsetPos) {
    
    float interpedVelocity;
    
    //put into gridspace and get that ijk
    glm::vec3 offsetPos = ppos - poffset;
    glm::ivec3 ijk = posToIJK(offsetPos);
    
    glm::ivec3 dir0 = (glm::ivec3) gridDir;
    glm::ivec3 dir1 = glm::ivec3(gridDir[1], gridDir[2], gridDir[0]);
    glm::ivec3 dir2 = glm::ivec3(gridDir[2], gridDir[0], gridDir[1]);
    
    std::vector<glm::ivec3> trilinNeighbors;
    
    trilinNeighbors.push_back(ijk); //-dir0

    float picWeight = 0.05;
    float flipWeight = 0.95;
   
    float PICinterp1;
    float FLIPinterp1;
    if (inGridBounds(ijk + dir0)) {
        
        int neighb0index = ijkToGridIndex(ijk);
        //int neighb1index = ijkToGridIndex(neighb0);
        
        glm::ivec3 neighb0 = ijk + dir0; //()
         trilinNeighbors.push_back(neighb0); // +dir1
        
        float totalDist = glm::distance(offsetPos, (glm::vec3) neighb0) + glm::distance(offsetPos, (glm::vec3) ijk);
        float weight = glm::distance(offsetPos, (glm::vec3) ijk) /totalDist;
    
        PICinterp1= weight * (*this)(ijk) + (1-weight)*(*this)(neighb0);
        //FLIP interp1 = weight * deltas.at(neighb0index) + (1- weight) * (deltas.at(neighb1index));
    }
    else {
        PICinterp1 = (*this)(ijk);
    }
    

    float interp2;
    if (inGridBounds(ijk + dir1)) {
        glm::ivec3 neighb0 = ijk + dir1;
        trilinNeighbors.push_back(ijk + dir1); //-dir1
        
        
        float interp2a = (*this)(neighb0);
        
        if (inGridBounds(ijk + dir1 + dir0)) {
            
            glm::ivec3 neighb1 = ijk + dir1 + dir0;
            trilinNeighbors.push_back(ijk + dir1 + dir0); //+dir1
            
            float totalDist = glm::distance(offsetPos, (glm::vec3) neighb1) + glm::distance(offsetPos, (glm::vec3) neighb0);
            float weight = glm::distance(offsetPos, (glm::vec3) neighb0) / totalDist;
            
            interp2 = weight * interp2a + (*this)(neighb1);
        }
        else {
            interp2 = interp2a;
        }
    }

    
    float interp3;
    if (inGridBounds(ijk + dir2)) {
        glm::ivec3 neighb0 = ijk + dir2;
        trilinNeighbors.push_back(ijk + dir2); //-dir1
        
        
        float interp2a = (*this)(neighb0);
        
        if (inGridBounds(ijk + dir2 + dir0)) {
            glm::ivec3 neighb1 = ijk + dir2 + dir0;
            trilinNeighbors.push_back(ijk + dir2 + dir0); //+dir1
            
            float totalDist = glm::distance(offsetPos, (glm::vec3) neighb0) + glm::distance(offsetPos, (glm::vec3) ijk);
            float weight = glm::distance(offsetPos, (glm::vec3) neighb0) / totalDist;
            
            interp3 = weight * interp2a + (*this)(neighb1);
            
        }
        else {
            interp3 = interp2a;
        }
    }
    
    float interp4;
    if (inGridBounds(ijk + dir1 + dir2)) {
        glm::ivec3 neighb0 = ijk + dir1 + dir2;
        trilinNeighbors.push_back(ijk + dir1 + dir2); //-dir1
        
        
        float interpa = (*this)(neighb0);
        
        if (inGridBounds(ijk + dir2 + dir1 + dir0)) {
            glm::ivec3 neighb1 = ijk + dir2 + dir1 + dir0;
            trilinNeighbors.push_back(ijk + dir2 + dir1 + dir0); //+dir1
            
            
            float totalDist = glm::distance(offsetPos, (glm::vec3) neighb0) + glm::distance(offsetPos, (glm::vec3) ijk);
            float weight = glm::distance(offsetPos, (glm::vec3) neighb0) / totalDist;
        
            interp4 = weight * interpa + (*this)(neighb1);
        }
        else {
            interp4 = interpa;
        }
    }
    
    
    


    return interpedVelocity;
}
*/

template<typename T>
//give grid direction to check bounds
float Grid<T>::getInterpedVelocity(glm::vec3 ppos, glm::vec3 poffset, std::vector<float> deltas) {
    
    float interpedFLIPVelocity;
    std::vector<glm::ivec3> trilinNeighbors;
    
    //put into gridspace and get that ijk
    glm::vec3 offsetPos = ppos - poffset;
    glm::ivec3 ijk = posToIJK(offsetPos);
    
    if (!inGridBounds(ijk)) {
        
        std::cout << " LOL WHAT YOU DOING OUT OF BOUNDS " << std::endl;
        std::cout << " ppos " << glm::to_string(ppos) << std::endl;
        std::cout << " ijk " << glm::to_string(ijk) << std::endl;
    }
                 
    int ijkindex = ijkToGridIndex(ijk);

    
    trilinNeighbors.push_back(ijk); //-dir0
    
    glm::ivec3 dir0 = (glm::ivec3) gridDir;
    glm::ivec3 dir1 = glm::ivec3(gridDir[1], gridDir[2], gridDir[0]);
    glm::ivec3 dir2 = glm::ivec3(gridDir[2], gridDir[0], gridDir[1]);
    
    float FLIPinterp1;
    std::vector<float> interps;
    if (inGridBounds(ijk + dir0)) {
    
        glm::ivec3 neighb1 = ijk + dir0;
        int neighb1index = ijkToGridIndex(neighb1);
        
        float totalDist = glm::distance(offsetPos, (glm::vec3) neighb1) + glm::distance(offsetPos, (glm::vec3) ijk);
        float weight = glm::distance(offsetPos, (glm::vec3) ijk) /totalDist;
        
        FLIPinterp1= weight * deltas.at(ijkindex) + (1-weight)*(deltas.at(neighb1index));
        }
    else {
        
        FLIPinterp1 = deltas.at(ijkindex);
    }
    
    interps.push_back(FLIPinterp1);
    
    float FLIPinterp2;
    if (inGridBounds(ijk + dir1)) {
        glm::ivec3 neighb0 = ijk + dir1;
        //trilinNeighbors.push_back(ijk + dir1); //-dir1
        int neighb0index = ijkToGridIndex(neighb0);
        
        float interp2a = deltas.at(neighb0index);
        
        if (inGridBounds(ijk + dir1 + dir0)) {
            
            glm::ivec3 neighb1 = ijk + dir1 + dir0;
            int neighb1index = ijkToGridIndex(neighb1);
            
            //trilinNeighbors.push_back(ijk + dir1 + dir0); //+dir1
            
            float totalDist = glm::distance(offsetPos, (glm::vec3) neighb0) + glm::distance(offsetPos, (glm::vec3) neighb1);
            float weight = glm::distance(offsetPos, (glm::vec3) neighb0) / totalDist;
            
            FLIPinterp2 = weight * interp2a + (1 - weight) *(deltas.at(neighb1index));
        }
        else {
            FLIPinterp2 = interp2a;
        }
    }
    else {
        FLIPinterp2 = deltas.at(ijkindex);
    }
    
    interps.push_back(FLIPinterp2);
    
    
    float FLIPinterp3;
    if (inGridBounds(ijk + dir2)) {
        glm::ivec3 neighb0 = ijk + dir2;
        trilinNeighbors.push_back(ijk + dir2); //-dir1
        
        int neighb0index = ijkToGridIndex(neighb0);
        float interp2a = deltas.at(neighb0index);
        
        if (inGridBounds(ijk + dir2 + dir0)) {
            glm::ivec3 neighb1 = ijk + dir2 + dir0;
            //trilinNeighbors.push_back(ijk + dir2 + dir0); //+dir1
            
            float totalDist = glm::distance(offsetPos, (glm::vec3) neighb0)
                + glm::distance(offsetPos, (glm::vec3) neighb1);
            
            float weight = glm::distance(offsetPos, (glm::vec3) neighb0) / totalDist;
            
            int neighb1index = ijkToGridIndex(neighb1);
            FLIPinterp3 = weight * interp2a + (1 - weight)* deltas.at(neighb1index);
            
        }
        else {
            FLIPinterp3 = interp2a;
        }
    }
    else {
        FLIPinterp3 = deltas.at(ijkindex);
    }
    
    
    interps.push_back(FLIPinterp3);
    
    
    float FLIPinterp4;
    if (inGridBounds(ijk + dir1 + dir2)) {
        glm::ivec3 neighb0 = ijk + dir1 + dir2;
        trilinNeighbors.push_back(ijk + dir1 + dir2); //-dir1
        
        
        int neighb0index = ijkToGridIndex(neighb0);
        float interpa = deltas.at(neighb0index);
        
        if (inGridBounds(ijk + dir2 + dir1 + dir0)) {
            glm::ivec3 neighb1 = ijk + dir2 + dir1 + dir0;
            //trilinNeighbors.push_back(ijk + dir2 + dir1 + dir0); //+dir1
            int neighb1index = ijkToGridIndex(neighb1);
            
            float totalDist = glm::distance(offsetPos, (glm::vec3) neighb0)
                + glm::distance(offsetPos, (glm::vec3) neighb1);
            float weight = glm::distance(offsetPos, (glm::vec3) neighb0) / totalDist;
            
            FLIPinterp4 = weight * interpa + (1-weight)*deltas.at(neighb1index);
        }
        else {
            FLIPinterp4 = interpa;
        }
    }
    else {
        FLIPinterp4 = deltas.at(ijkindex);
    }
    
    
    interps.push_back(FLIPinterp4);
    
    std::vector<glm::ivec3> trilinNeighbors2;
    std::vector<float> interps2;
    for (int i = 0; i < trilinNeighbors.size() - 1; i+=2) {
        
        glm::ivec3 neighb0 = trilinNeighbors.at(i);
        glm::ivec3 neighb1 = trilinNeighbors.at(i + 1);
        
   
        
        float totalDist = glm::distance(offsetPos, (glm::vec3) neighb0) + glm::distance(offsetPos, (glm::vec3) neighb1);
        float weight = glm::distance(offsetPos, (glm::vec3) neighb0) / totalDist;
        
        trilinNeighbors2.push_back(neighb0);
        interps2.push_back(weight * interps.at(i)  + (1- weight)* (interps.at(i + 1))); //)
        
    }
    if (interps.size() < 2 || trilinNeighbors.size() < 2) {
        interps2.push_back(interps.at(0));
    }
    
    if(interps.size() < 1 || trilinNeighbors.size() < 1) {
        std::cout << " lol where'd all my neighbors go" << std::endl;
    }
    
    float FLIPinterp6 = 0;
    if (trilinNeighbors2.size()> 1) {
        glm::ivec3 neighb0 = trilinNeighbors2.at(0);
        glm::ivec3 neighb1 = trilinNeighbors2.at(1);
        float totalDist = glm::distance(offsetPos, (glm::vec3) neighb0) + glm::distance(offsetPos, (glm::vec3) neighb1);
        float weight = glm::distance(offsetPos, (glm::vec3) neighb0) / totalDist;
        
        FLIPinterp6 += weight * deltas.at(interps2.at(0))  + (1- weight)*(interps2.at(1));
        
    }
    
    else {
        FLIPinterp6 = interps2.at(0);
        
    }
    
    if (trilinNeighbors2.size()> 2) {
        std::cout << " whaaaat " << trilinNeighbors2.size() << " neighbors lol " << std::endl;
    }

    
    interpedFLIPVelocity = FLIPinterp6;
    return interpedFLIPVelocity;
}

template<typename T>
float Grid<T>::getDiv(int i, int j, int k) {
    glm::ivec3 here(i, j, k);
    glm::ivec3 there = here + (glm::ivec3) gridDir;
    
    int ind0 = ijkToGridIndex(here);
    int ind1 = ijkToGridIndex(there);
    
    
    if(inGridBounds(there)) {
        
        return data.at(ind1) - data.at(ind0);
    }
       
    else {
           return 0 - data.at(ind0);
    }
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


template<typename T>
glm::ivec3 Grid<T>::posToIJK(glm::vec3 pos){
   
    float epsilon = 0.001;
    

    for(int i = 0; i < 3; i++) {
        if (pos[i] != 0)  {
            pos[i] -= epsilon;
        }
    }
    
    //find closest multiple of cellSideLength
    int x = pos.x/cellSidelength;
    int y = pos.y/cellSidelength;
    int z = pos.z/cellSidelength;
    
    return glm::ivec3(x, y, z);
}

/*template<typename T>
bool Grid<T>::isInBounds(glm::ivec3 ijk) {
    return ijk.x >= 0 && ijk.y >= 0 && ijk.z >= 0
        && ijk.x < dimX && ijk.y < dimY && ijk.z < dimZ ;
    
}*/



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
        
        return 0;
    }
    else  {
        return data.at(k * dimX * dimY + j * dimX + i);
    }
}

template<typename T>
float Grid<T>::operator()(glm::ivec3 indices) {
    int i = indices.x;
    int j = indices.y;
    int k = indices.z;
    
    return (*this)(i, j,k);
}




template<typename T>
void Grid<T>::storeParticleVelocityToGrid(Particle p, glm::vec3 offset, float W) {
    
    float pcomponent = glm::dot(gridDir, p.speed);
    if (pcomponent == 0) {
        return; //if zero in this dir then nah
    }
    
    glm::vec3 offsetPos = ((glm::vec3) p.gridIJK) - offset;

    //get neighbors of particle in grid
    std::vector<glm::ivec3> cellsToSplat = getTrilinNeighbors(p.gridIJK);
    

    //splat onto neighbors
    for (glm::ivec3 neighborCell : cellsToSplat) {
        if (!inGridBounds(neighborCell)) {
            continue;
        }
        
        int neighbor1DIndex = ijkToGridIndex(neighborCell);
        float kernelWeight = getKernelWeight(pcomponent, offsetPos, neighborCell, W);
        
        std::cout <<neighbor1DIndex << std::endl;
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
                
                float pressure2 = (*gridP)(two.x, two.y, two.z);
                float pressurechange = pressure2 - pressure1;

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



