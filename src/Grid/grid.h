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
    
    //all the data
    std::vector<T> data;

    //constructors
    Grid();
    Grid(int bx, int by, int bz, float side);

    //grid to particle
    float trilinearlyInterpolate(glm::vec3 pos, glm::vec3 offset);
    float operator()(int i, int j, int k);
    
    //particle to grid
    int getGridIndexFromPosition(glm::vec3 position);
    int getGridIndexFromIJK(glm::vec3 IJK);
    //void storeParticlesToGrid(std::map<int, std::vector<Particle>>* particlesByIndex, glm::vec3 offset);
    float getKernelWeight(float pcomponent, glm::vec3 offsetParticlePos, glm::vec3 staggeredPos, float W) ;
    
    std::vector<glm::vec3> getNeighborPositions(glm::vec3 particleIndex, glm::vec3 direction);
    
    void resetToZero();
    void storeParticleVelocityToGrid(Particle p, glm::vec3 offset, glm::vec3 direction, float W);
    glm::vec3 getGridIndices(glm::vec3 pos);
    void colorSplattedFaces(Particle p);
    
    //extrapolation
    void extrapolateVelocities(Grid<int>* marker); //(int i, int j, int k);
    
    //force resoltion
    void addForce(float f);
    void addValueAt(float value, int gridIndex);
    
    //pressure solving
    //void lol
    
    //for debugging
    void printContents(std::string message);

    //void extrapolateVelocities();
    //void extrapolateToCell(int i);

    
};







