#include "FlipSolver.h";



float fluidBoundX = 1.8;
float fluidBoundY = 1.8;
float fluidBoundZ = 1.8;


FlipSolver::FlipSolver(Geom* g) {
    container = g;
}

/// Create the MAC grid ///
void FlipSolver::ConstructMACGrid() {
    
    //doing this for now lol
    //will fix in json later
    //this is ratchettttt
    particleSeparation = 0.6;
    
    //each grid dimension is enough so that a gridcell holds about 8 particles total
    float gridCellSidelength = particleSeparation * 2;
   
    //construct a MAC grid out of container dimensions
    //mGrid = new MACGrid(fluidBoundX, fluidBoundY, fluidBoundZ, gridCellSidelength);
    mGrid = new MACGrid(container->boxBoundX,
                        container->boxBoundY,
                        container->boxBoundZ,
                        gridCellSidelength);
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
                    
                    glm::vec3 jitter = glm::vec3(0.01 * rand(), 0.01 * rand(), 0.01 * rand());
                    p.pos = glm::vec3(i, j, k); //+ jitter;
                    p.speed = glm::vec3(0, 0, 0);
                    
                    p.r = 0;
                    p.g = 0;
                    p.b = 255;
                    p.a = 255;
                    
                    p.size = 0.1;
                    
                    //set grid index, also mark gird index
                    p.gridIndex = mGrid->getGridIndex(p.pos);
                    
                    
                    mGrid->gridMarker->addValueAt(1, p.gridIndex);
                    //std::cout << glm::to_string(p.pos) << " --> " << p.gridIndex << std::endl;
                    //
                    
                    ParticlesContainer.push_back(p);
                }
            }
        }
    }
    
    mGrid->gridMarker->printContents("marker grid!");
    
    //set the W for kernel weight function
    mGrid->calculateAvgNumberOfParticlesPerGrid();
    
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
    

    //put particle onto grid
    StoreParticleVelocitiesToGrid();
    
    

    //extrapolate velocities
    mGrid->extrapolateVelocities();   //markerGrid.data());

    
    //save old gridV
    std::vector<float> deltaU(mGrid->gridU->data);
    std::vector<float> deltaV(mGrid->gridV->data);
    std::vector<float> deltaW(mGrid->gridW->data);
    
    //update for gravity, data is now PIC velocities
      //  std::cout << " THIS IS GRIDVVVVV AFTER FORCE IS ADDED " << delta << std::endl;
    mGrid->gridV->addForce(-9.8 * delta);
    std::cout << (9.8 * delta) << std::endl;
    //mGrid->gridV->printContents();

    //
    

    //calculate delta velocity
    for (int i = 0; i < mGrid->gridV->data.size(); i++) {
        deltaV[i] = mGrid->gridV->data[i] - deltaV[i];
        //std::cout << " THIS IS DELTAAAAA_V " << std::endl;
        //std::cout << deltaV[i] << " ";
    }
    
    float dummyPIC = mGrid->gridV->data[0];
    float dummyFLIP = deltaV[0];
 
   /* std::cout << " DUMPIC " << dummyPIC << std::endl;
    std::cout << " DUMFLIP " << dummyFLIP << std::endl;
    */
    //TODO ACTUALLY IMPLEMENT THIS
    //put grid back onto each particle
    std::vector<Particle> updatedParticles;
    
    for (Particle p : ParticlesContainer) {
        
        //getInterpolatedVelocity(p.pos);
        
        //giving a dummy speed based on gridV * delta
        glm::vec3 picVel = glm::vec3(0, 0.05 * dummyPIC, 0);
        glm::vec3 flipVel = glm::vec3(0, 0.95 * dummyFLIP, 0);
        
        p.speed = picVel + flipVel;
        //std::cout << " SPEED | " << glm::to_string(p.speed) << std::endl;
        
        //p.updatePositionWithRK2();
        glm::vec3 fwdEulerPos = p.pos + p.speed;
        glm::vec3 midpoint = glm::vec3((fwdEulerPos.x + p.pos.x)/2,
                                       (fwdEulerPos.y + p.pos.y)/2,
                                       (fwdEulerPos.z + p.pos.z)/2);
       // std::cout << " MIDPOINT " << glm::to_string(midpoint) << std::endl;
        
        //glm::vec3 midEuler = getInterpolatedVelocity(midPoint);
        //clarify the difference between midpoint distance and midpoint time
        glm::vec3 midEuler = picVel + flipVel; // super basic ho for now
        //std::cout << " MIDEULER " << glm::to_string(midpoint) << std::endl;
        //do i just replace this speed?
        //or do i somehow weight it with the originally calculated vel?
        p.speed = midEuler;
        
        p.pos =  midpoint + midEuler * delta/2.0f;
        //std::cout << " NEW POS " << glm::to_string(p.pos) << std::endl;
        
        p.r = 0;
        p.g = 0;
        p.b = 255;
        p.a = 255;
        
        //collision detection!
        if (!container->insideContainer(p.pos)) {
           p.pos.y = -container->boxBoundY;
            //std::cout << " this is my pos " << glm::to_string(p.pos) << std::endl;
            p.r = 0;
            p.g = 255;
            p.b = 255;
        }
        
        updatedParticles.push_back(p); //InterpolateVelocity(p.pos, *mGrid);
    }
    
    
    //container collision detection
    
    
    ParticlesContainer = updatedParticles;
  
    //send info to the GPU by calling the fluidsolver's update() function
    update(delta, boxScaleX, boxScaleY, boxScaleZ, CameraPosition);
    
}

void FlipSolver::MACGrid2Particle() {
    
    //save old MACGrid data
    //oldMACGrid = mGrid;
    
    //update velocities in each grid
    //based on the forces
    //gridU->update();
    //gridV->update();
    //gridW->update();
    
    //do later
    //gridP->update();
    
    for (Particle p : ParticlesContainer) {
        p.speed = mGrid->giveNewVelocity(p);
    }
    
}


bool FlipSolver::withinFluidBounds(float i, float j, float k) {
    
    return (i < particleBoundX && j < particleBoundY && k < particleBoundZ);
    
}

//: This function will calculate a weighted average of the particles' velocities
//in the neighborhood (define a neighborhood of 1 cell width) and store these
//values on corresponding grid cells.
//A simple stiff kernel can be used to calculate the weight for the average velocity.
void FlipSolver::StoreParticleVelocitiesToGrid(){
    
    mGrid->gridMarker->resetToZero(); // macgrid to zero
    mGrid->resetToZero();
    mGrid->gridV->printContents("grid V reset to zero!");
    
    for (Particle p : ParticlesContainer) {
        mGrid->gridMarker->addValueAt(1, p.gridIndex);
        mGrid->storeParticleVelocityToGrid(p);
    }
    
    mGrid->gridV->printContents("particles stored to gridV");
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

