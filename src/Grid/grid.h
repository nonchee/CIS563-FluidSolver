//grid.hpp
#pragma once

#include "../solvers/Particle.hpp"
#include <map>
#include <vector>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <fstream>

template<typename T>
class Grid {
    
    static const GLfloat* grid_vertex_buffer_data_array;
    //ofstream myfile;

    
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
    float operator()(glm::ivec3 indices);
    
    //indexing
    int ijkToGridIndex(glm::vec3 IJK);
    int ijkToGridIndex(glm::ivec3 IJK);
    glm::ivec3 posToIJK(glm::vec3 pos);

    //particle to grid
    void resetToZero();
    
   
    
    
    float getKernelWeight(float pcomp, glm::vec3 offsetParticlePos, glm::ivec3 neighborIJK, float W) ;
    void storeParticleVelocityToGrid(Particle p, glm::vec3 offset, float W);
    
    std::vector<glm::ivec3> getTrilinNeighbors(glm::ivec3 pIJK);
    std::vector<glm::ivec3> getTrilinNeighbors(glm::vec3 pos, glm::vec3 offset);
    

    
    //float getInterpedVelocity(glm::vec3 ppos, glm::vec3 poffset);
    float getInterpedVelocity(glm::vec3 ppos, glm::vec3 poffset, std::vector<float>deltas);
    //void splatFromPar();
    
    //extrapolation
    void extrapolateVelocities(Grid<int>* marker); //(int i, int j, int k);
    
    //force resolution
    void updateVel(int i, int j, int k, float f);
    void addValueAt(float value, int gridIndex);
    void addValueAt(float value, int i, int j, int k);
    void setValueAt(float value, int gridIndex);
    void setValueAt(float value, int i, int j, int k);

    //float getDivergence(int i, int j, int k);
    
    //pressure solving
    void pressureUpdate(int index, float scale);
        void pressureUpdate(Grid<float>* gridP );

    float getDelta(int i, int j, int k);
    void setDeltas(std::vector<float> calculatedDeltas);
    
    bool inGridBounds(glm::ivec3 perhaps);
    
    float getDiv(int i, int j, int k);
    
    //debugging
    void printContents(std::string message);
    void printDeltas(std::string message);
    void printGridValueAt(std::string s, int i, int j, int k);
    void printVector(glm::vec3 index, std::string message);
    void printNeighbors(std::vector<glm::ivec3> neighborPositions);

    
};







