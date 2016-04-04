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
    int axis;
    glm::vec3 backwardsDir;
    std::vector<float> delta;
    std::vector<T> data;

    //constructors
    Grid();
    Grid(int bx, int by, int bz, float side);
    
    
    //grid to particle
    float trilinearlyInterpolate(glm::vec3 pos, glm::vec3 offset);
    float operator()(int i, int j, int k);
    
    //indexing
    int getGridIndexFromPosition(glm::vec3 position);
    int getGridIndexFromIJK(glm::vec3 IJK);
    glm::vec3 getGridIndices(glm::vec3 pos);

    //particle to grid
    void resetToZero();
    std::vector<glm::vec3> getNeighborPositions(glm::vec3 particleIndex, glm::vec3 direction);
    float getKernelWeight(float pcomponent, glm::vec3 offsetParticlePos, glm::vec3 staggeredPos, float W) ;
    void storeParticleVelocityToGrid(Particle p, glm::vec3 offset, glm::vec3 direction, float W);
    
    
    //extrapolation
    void extrapolateVelocities(Grid<int>* marker); //(int i, int j, int k);
    
    //force resolution
    void addForce(float f);
    void addValueAt(float value, int gridIndex);
    void setValueAt(float value, int gridIndex);

    //float getDivergence(int i, int j, int k);
    
    //pressure solving
    void pressureUpdate(int index, float scale);
    void pressureUpdate(Grid<float>* gridP);
    float getDelta(int i, int j, int k);
    void setDeltas(std::vector<float> calculatedDeltas);
    
    //debugging
    void printContents(std::string message);

    
};







