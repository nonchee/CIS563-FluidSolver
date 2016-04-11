/*Define a class Grid
 The easiest way to verify correct indices for a grid is drawing a simple 2x2 grid on paper as we did in class
 Operator() (int i, int j, int k) is an elegant way to translate i,j,k to the correct index in the 1D std::vector. Remember grids U, V & P grids have different dimensions.
 If grid has vec3 dimension; observe that index in the 1D std::vector for (i,j,k) = k*dimension.x*dimension.y + j*dimension.x + i
 class MACGrid can simply be an encapsulation (composition) of the 4 staggered grids we need - gridU, gridV, gridW & gridP
 
 
 Define a derived class FlipSolver from base class FluidSolver
 */


#include <vector>
#include <map>

#include <glm/glm.hpp>

#include "FluidSolver.hpp"
#include "../grid/Grid.h"
#include "../grid/MACGrid.h"
#include "../geom/geom.hpp"
#include <Eigen/Sparse>
#include <Eigen/IterativeLinearSolvers>
#include <vector>

//typedef Eigen::Triplet<double> T;

class FlipSolver : public FluidSolver {
    
    Geom* container;
    bool gravityEnabled = true;
    bool withinFluidBounds(float i, float j, float k);
    
    float bbX;
    float bbY;
    float bbZ;
    
    
    int mx; // = mGrid->dimX;
    int my; // = mGrid->dimY;
    int mz;// = mGrid->dimZ;

public:
    FlipSolver(Geom* g);
    
    std::map<int, std::vector<Particle>> particlesByIndex;
    
    MACGrid* mGrid;

    void FlipUpdate(float delta, float boxScaleX, float boxScaleY, float boxScaleZ, glm::vec3 CameraPosition);

    
    void Init(); // : Initialize the grid and particle positions and velocities.
    void InitializeParticles();
    void ConstructMACGrid(); //Create the MAC grid
    
    void StoreParticleVelocitiesToGrid();
    
    glm::vec3 InterpolateVelocity(const glm::vec3& pos, const MACGrid& mGrid);
    
    //void MACGrid2Particle(float delta);
    void MACGrid2Particle(float delta, std::vector<float> deltaU, std::vector<float> deltaV, std::vector<float> deltaW);
    
    
    void updateGravity();
    void enableGravity();
    void disableGravity();
    
 
    bool isSolid(int i, int j, int k);
    bool isFluid(int i , int j , int k);
    bool outOfBounds(int i, int j, int k);
    
    int insertCoeff(int id, int i, int j, int k,
                std::vector<Eigen::Triplet<float>>& coeffs);
    int countFluidCells();
    
    void buildA(Eigen::SparseMatrix<float>& A, std::vector<Eigen::Triplet<float> >& coeffs);
    void buildb(Eigen::VectorXf& b);
    void PressureSolve(float dt);
    void PressureUpdate(Eigen::SparseMatrix<float> &A,Eigen::VectorXf &p, float dt);

    
};



