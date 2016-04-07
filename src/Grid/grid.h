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
    
    //some params
    int axis;
    glm::vec3 gridDir;
    
    //data
    std::vector<float> delta;
    std::vector<T> data;

    //constructors
    Grid();
    Grid(int bx, int by, int bz, float side);
    
    
    //grid to particle
    float trilinearlyInterpolate(glm::vec3 pos, glm::vec3 offset);
    float operator()(int i, int j, int k);
    
    //indexing
    int ijkToGridIndex(glm::vec3 IJK);
    int ijkToGridIndex(glm::ivec3 IJK);
    glm::vec3 getGridIndices(glm::vec3 pos);

    //particle to grid
    void resetToZero();
    
   
    
    
    float getKernelWeight(float pcomp, glm::vec3 offsetParticlePos, glm::ivec3 neighborIJK, float W) ;
    void storeParticleVelocityToGrid(Particle p, glm::vec3 offset, float W);
    
     std::vector<glm::ivec3> getTrilinNeighbors(glm::ivec3 pIJK);
    //void splatFromPar();
    
    //extrapolation
    void extrapolateVelocities(Grid<int>* marker); //(int i, int j, int k);
    
    //force resolution
    void addForce(float f);
    void addValueAt(float value, int gridIndex);
    void addValueAt(float value, int i, int j, int k);
    void setValueAt(float value, int gridIndex);
    void setValueAt(float value, int i, int j, int k);

    //float getDivergence(int i, int j, int k);
    
    //pressure solving
    void pressureUpdate(int index, float scale);
    void pressureUpdate(Grid<float>* gridP);
    float getDelta(int i, int j, int k);
    void setDeltas(std::vector<float> calculatedDeltas);
    
    bool inGridBounds(glm::ivec3 perhaps);
    
    //debugging
    void printContents(std::string message);
    void printGridValueAt(std::string s, int i, int j, int k);
    void printVector(glm::vec3 index, std::string message);
    void printNeighbors(std::vector<glm::ivec3> neighborPositions);
    

    
};







