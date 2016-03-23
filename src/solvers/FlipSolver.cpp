#include "FlipSolver.h";
#include "../grid/MACGrid.h"


float fluidBoundX = 3.6;
float fluidBoundY = 3.6;
float fluidBoundZ = 3.6;



/// Create the MAC grid ///

void FlipSolver::ConstructMACGrid() {
    
    //doing this for now lol
    //will fix in json later
    particleSeparation = 0.9;
    
    //each grid dimension is enough so that a gridcell holds about 8 particles total
    float gridCellSidelength = particleSeparation * 2;
   
    //construct a MAC grid out of particleBound dimensions
    mGrid = new MACGrid(fluidBoundX, fluidBoundY, fluidBoundZ, gridCellSidelength);
}

void FlipSolver::InitializeParticles() {
    //IT WAS FLOATING POINT ERRORS THE WHOLE TIME
    float epsilon = 0.001;
    
    //9 total particles along each axis
    for (float i = 0; i + epsilon < fluidBoundX; i+= particleSeparation) {
        for (float j = 0; j + epsilon < fluidBoundY; j += particleSeparation) {
            for (float k = 0; k + epsilon  < fluidBoundZ; k += particleSeparation) {
                
                if (withinFluidBounds(i, j, k) ) {
                    
                    Particle p;
                    
                    //glm::vec3 jitter = glm::vec3(0.01 * rand(), 0.01 * rand(), 0.01 * rand());
                    p.pos = glm::vec3(i, j, k); //+ jitter;
                    p.speed = glm::vec3(0, 0, 0);
                    
                    p.r = 0;
                    p.g = 0;
                    p.b = 255;
                    p.a = 255;
                    
                    p.size = 0.1;
                    
                    //set grid index
                    p.gridIndex = mGrid->getGridIndex(p.pos);
                    std::vector<Particle>* particleSet = &(particlesByIndex[p.gridIndex]);
                    particleSet->push_back(p);
                    
                    ParticlesContainer.push_back(p);
                }
            }
        }
    }
}


void FlipSolver::enableGravity() {
    gravityEnabled = true;
}

void FlipSolver::disableGravity() {
    gravityEnabled = false;
}

/// Initialize the grid and particle positions and velocities.
//Using a helper function can simplify this task. Saving gridIndex on the particle can be useful later.
void FlipSolver::Init() {
    
    ConstructMACGrid();
    
    InitializeParticles();
    
    StoreParticleVelocitiesToGrid();

}




void FlipSolver::FlipUpdate(float delta, float boxScaleX, float boxScaleY, float boxScaleZ, glm::vec3 CameraPosition) {
    
    for (Particle p : ParticlesContainer) {
        InterpolateVelocity(p.pos, *mGrid);
    }
    
    //send info to the GPU by falling the fluidsolver's update() function
    update(delta, boxScaleX, boxScaleY, boxScaleZ, CameraPosition);
    
}



bool FlipSolver::withinFluidBounds(float i, float j, float k) {
    
    return (i < particleBoundX && j < particleBoundY && k < particleBoundZ);
    
}

//: This function will calculate a weighted average of the particles' velocities in the neighborhood (define a neighborhood of 1 cell width) and store these values on corresponding grid cells.
//A simple stiff kernel can be used to calculate the weight for the average velocity.
void FlipSolver::StoreParticleVelocitiesToGrid(){
    
    mGrid->resetToZero(ParticlesContainer.size());
    
    
    for (Particle p : ParticlesContainer) {
        mGrid->storeParticleVelocityToGrid(p);
    }
    
    //mGrid->storeParticlesToGrid(&particlesByIndex);

    
}

//Calculate the trilinear interpolation for a velocity value at particle position.
//Using the worldPos of the particle, find the cell index (i,j,k) in the grid.
//Note that the grids are staggered differently.
//So, you will need find the actual gridPos to get the correct index (i,j,k).
//Using this index, we interpolate separately for each component.
//Think of how you want to design your function calls for good modularity and code reuse
glm::vec3 FlipSolver::InterpolateVelocity(const glm::vec3& pos, const MACGrid& mGrid)
{
    return mGrid.interpolateFromGrid(pos);
    
}