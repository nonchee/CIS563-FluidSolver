//grid.hpp
#pragma once

#include "../solvers/Particle.hpp"
#include <map>
#include <vector>
#include <GL/glew.h>
#include <GLFW/glfw3.h>

template<typename T>
class Grid {
    
    static const GLfloat* grid_vertex_buffer_data_array;
    
public:
    //dimensions
    int dimX ;
    int dimY ;
    int dimZ ;
    float cellSidelength;
    
    std::vector<T> data;

    Grid();
    Grid(int bx, int by, int bz, float side);

    //grid to particle
    glm::vec3 trilinearlyInterpolate(glm::vec3 pos, glm::vec3 offset);
    float operator()(int i, int j, int k);
    
    //particle to grid
    int getGridIndexFromPosition(glm::vec3 position);
    int getGridIndexFromIJK(glm::vec3 IJK);
    void storeParticlesToGrid(std::map<int, std::vector<Particle>>* particlesByIndex, glm::vec3 offset);
    float getKernelWeight(glm::vec3 offsetParticlePos, glm::vec3 staggeredPos) ;
    //std::vector<glm::vec3> getNeighborPositions(int i, int j, int k);
    std::vector<glm::vec3> getNeighborPositions(glm::vec3 particleIndex, glm::vec3 direction);
    void resetToZero();
    void storeParticleVelocityToGrid(Particle p, glm::vec3 offset, glm::vec3 direction);
    glm::vec3 getGridIndices(glm::vec3 pos);
    void colorSplattedFaces(Particle p);
    
    void addForce(float f);
    void addValueAt(float value, int gridIndex); 
    
    //for debugging
    void printContents();
    
    void extrapolateVelocities(); 

    
};


//definition of class template functions

template<typename T>
inline
void Grid<T>::resetToZero() {
    data.clear();
    
    for (int i= 0; i < dimX * dimY * dimZ; i++) {
        data.push_back(0);
    }
}





