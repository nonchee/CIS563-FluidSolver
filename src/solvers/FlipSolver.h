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

class FlipSolver : public FluidSolver {
    
   
    bool gravityEnabled = true;
    bool withinFluidBounds(float i, float j, float k);

public:
    
    std::map<int, std::vector<Particle>> particlesByIndex;
    
    MACGrid* mGrid;

    void FlipUpdate(float delta, float boxScaleX, float boxScaleY, float boxScaleZ, glm::vec3 CameraPosition);
    

    
    void Init(); // : Initialize the grid and particle positions and velocities.
    void InitializeParticles();
    void ConstructMACGrid(); //Create the MAC grid
    
    void StoreParticleVelocitiesToGrid();
    
    //: This function will calculate a weighted average of the particles' velocities in the neighborhood (define a neighborhood of 1 cell width) and store these values on corresponding grid cells. A simple stiff kernel can be used to calculate the weight for the average velocity.
    glm::vec3 InterpolateVelocity(const glm::vec3& pos, const MACGrid& mGrid);
    //: Calculate the trilinear interpolation for a velocity value at particle position. Using the worldPos of the particle, find the cell index (i,j,k) in the grid. Note that the grids are staggered differently. So, you will need find the actual gridPos to get the correct index (i,j,k). Using this index, we interpolate separately for each component. Think of how you want to design your function calls for good modularity and code reuse
    
    void enableGravity();
    void disableGravity();
    
};
