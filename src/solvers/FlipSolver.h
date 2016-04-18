#ifndef FLIPSOLVER_H
#define FLIPSOLVER_H

#include <vector>
#include <glm/glm.hpp>

#include <Eigen/Sparse>
#include <Eigen/IterativeLinearSolvers>
#include <Eigen/Core>
#include <vector>

#include "FluidSolver.hpp"
#include "../grid/MACGrid.h"
#include "../geom/geom.hpp"



#define DELTA_TIME 0.001f
#define PSEP 0.25f
#define PARTICLES_PER_CELL 1.f/PSEP //technically for each dimension lol
#define DX 1.f


#define GRAVITY -9.8f


class MACGrid;
//typedef Eigen::Triplet<double> T;

class FlipSolver : public FluidSolver {
    
    float bbX; float bbY; float bbZ;
    
    MACGrid* mGrid;
    int mx; int my; int mz;


public:
    FlipSolver(float boxScaleX, float boxScaleY , float boxScaleZ);

    void Init(); // : Initialize the grid and particle positions and velocities.
    void InitializeParticles();
    void ConstructMACGrid(); //Create the MAC grid
    
    void FlipUpdate();
  
    
    void SetGridsToZero();
    void MarkSolidBoundaries();
    
    void ResetWeightCounts(); 
    void MarkFluidCells();
    void StoreParticlesToGrid();
    
    void AddExternalForcesToGrids();
    
    void ApplyBoundaryConditions(); 
    
    void StoreGridToParticles(std::vector<float> deltaU,
                          std::vector<float> deltaV,
                          std::vector<float> deltaW);
    
    void UpdateParticlePositions();
    

    bool isSolid(int i, int j, int k);
    bool isFluid(int i , int j , int k);
    bool outOfBounds(int i, int j, int k);
    
    float insertCoeff(int id, int i, int j, int k,
                std::vector<Eigen::Triplet<float>>& coeffs);

    bool withinFluidBounds(float i, float j, float k);
    
    void buildA(Eigen::SparseMatrix<float>& A);
    void buildb(Eigen::VectorXf& b);
    void ComputePressures();
    void AdjustForPressure(Eigen::SparseMatrix<float> &A,Eigen::VectorXf &p);
    
    void printParticles(std::string message);
};


#endif
