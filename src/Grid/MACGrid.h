#ifndef MACGRID_H
#define MACGRID_H

#include "../solvers/FlipSolver.h"
#include "../solvers/Particle.hpp"
#include "Grid.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/norm.hpp>
#include "glm/gtx/string_cast.hpp"

#include <vector>
#include <Eigen/IterativeLinearSolvers>
#include <Eigen/Sparse>
#include <Eigen/Core>


class MACGrid {
    
public:
    MACGrid(float bbX, float bbY, float bbZ);
    
    int particleCount; 
    
    int dimX ;
    int dimY ;
    int dimZ ;

    
    float bbX;
    float bbY;
    float bbZ;
    
    Grid<float>* gridU;    //u
    Grid<float>* gridV;    //v
    Grid<float>* gridW;    //w
    Grid<float>* gridP;    //pressure
    Grid<int>* gridMarker; //markerGrid<float> gridDiv"
    
    ///save old grids
    std::vector<float> deltaU; //(mGrid->gridU->data);
    std::vector<float> deltaV; //(mGrid->gridV->data);
    std::vector<float> deltaW;
    
    
    int numFluidCells;
    int posToIndex(glm::vec3 position);
    int posToIndex(float i , float j, float k);
    glm::ivec3 posToIJK(glm::vec3 position);
    int ijkToIndex(glm::ivec3 ijk);
    int ijkToIndex(int i, int j, int k);
 
    

    
    void resetGrids();
    void storeParticleToMarker(Particle p);
    
    
    void addExternalForcesToGrids(glm::vec3 force);

    void collisionResponse(glm::vec3 pos);
    
    //grid to particle
    glm::vec3 interpolateFromGrid(glm::vec3 pos,
                                  std::vector<float> deltaU,
                                  std::vector<float> deltaV,
                                  std::vector<float> deltaW) const;

    
    void UpdatePressureGrid(Eigen::SparseMatrix<float> &A, Eigen::VectorXf &p);
    void UpdateVelocityGridsByPressure();
    
    void extrapolateVelocities();
    void printMarker(std::string caption);
    void printDimensions();
    
    bool isSolid(glm::ivec3 cellIJK);
    bool isFluidOrAir(glm::ivec3 cellIJK); 
    
    
};

#endif