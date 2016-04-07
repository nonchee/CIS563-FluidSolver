#pragma once
#include "Grid.h"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/norm.hpp>
#include "glm/gtx/string_cast.hpp"

#include "../solvers/Particle.hpp"
#include <map>
#include <vector>

class MACGrid {
    
public:
    MACGrid(float fluidBoundX, float fluidBoundY, float fluidBoundZ, float gridCellSidelengths);
    
    int particleCount; 
    
    int dimX ;
    int dimY ;
    int dimZ ;
    float cellSidelength;
    float W; //avg number of particles per grid, for kernel function
    
    float bbX;
    float bbY;
    float bbZ;
    
    Grid<float>* gridU;    //u
    Grid<float>* gridV;    //v
    Grid<float>* gridW;    //w
    Grid<float>* gridP;    //pressure
    Grid<int>* gridMarker; //markerGrid<float> gridDiv"
    int numFluidCells;
    void setNumFluidCells(int num);
    
    int getGridIndex(glm::vec3 position);
    int getGridIndex(int i , int j, int k);
    glm::ivec3 getGridIJK(glm::vec3 position);
    
    
    void resetGrids();
    void markSolidBoundaries();
    
    
    //particle to grid
    void storeParVelToGrids(Particle p);
    
    //grid to particle
    glm::vec3 interpolateFromGrid(glm::vec3 pos) const;


    glm::vec3 giveNewVelocity(Particle p);
    void calculateAvgNumberOfParticlesPerGrid();
    
    void extrapolateVelocities();
    void printMarker(std::string caption);
    void printDimensions(); 
    
};