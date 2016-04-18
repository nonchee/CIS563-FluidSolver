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
    
public:
    //dimensions
    int dimX ;
    int dimY ;
    int dimZ ;

    
    //some params
    int axis;
    glm::ivec3 gridDir;
    
    //data
    std::vector<float> delta;
    std::vector<T> data;
    std::vector<float> totalWeights;

    //constructors
    Grid();
    Grid(int dimX, int dimY, int dimZ);
    
    
    //grid to particle
    float trilinearlyInterpolate(glm::vec3 pos, glm::vec3 offset);
    float operator()(int i, int j, int k);
    float operator()(glm::ivec3 indices);
    
    //indexing
    int ijkToIndex(glm::vec3 IJK);
    int ijkToIndex(glm::ivec3 IJK);
    glm::ivec3 posToIJK(glm::vec3 pos);
    glm::ivec3 posToStaggeredIJK(glm::vec3 pos);


    //getting splat
    void resetToZero();
    void splatParticleVelocityToGrid(Particle p, glm::vec3 offset);
    void addWeightAt(float weight, int i, int j, int k);
    
   
    float getInterpedVelocity(glm::vec3 ppos, glm::vec3 poffset, std::vector<float>deltas);
    void ApplyBoundaryConditions();
    
    //extrapolation
    void extrapolateVelocities(Grid<int>* marker); //(int i, int j, int k);
    
    //force resolution
    //void updateVel(int i, int j, int k, float f);
    void addValueAt(int value, int gridIndex);
    void addValueAt(float value, int gridIndex);
    void addValueAt(float value, int i, int j, int k);
    void setValueAt(float value, int gridIndex);
    void setValueAt(float value, int i, int j, int k);

    
    //pressure solving
    void pressureUpdate(int index, float scale);
        void pressureUpdate(Grid<float>* gridP );

    float getDelta(int i, int j, int k);
    void setDeltas(std::vector<float> calculatedDeltas);
    
    bool inGridBounds(glm::ivec3 perhaps);
    
    float getDiv(int i, int j, int k);
    
    void UpdateGridByPressure(); 
    
    //debugging
    void printContents(std::string message);
    void printDeltas(std::string message);
    void printGridValueAt(std::string s, int i, int j, int k);
    void printVector(glm::vec3 index, std::string message);
    void printNeighbors(std::vector<glm::ivec3> neighborPositions);

    
};







