#include "FlipSolver.h";
#include <Eigen/IterativeLinearSolvers>
#include <Eigen/Sparse>
#include <Eigen/Core>




FlipSolver::FlipSolver(Geom* g) {
    container = g;
    
    bbX = g->boxBoundX;
    bbY = g->boxBoundY;
    bbZ = g->boxBoundZ;
}

/// Create the MAC grid ///
void FlipSolver::ConstructMACGrid() {
    
    mGrid = new MACGrid(bbX, bbY, bbZ, particleSeparation * 2);
    
    mx = mGrid->dimX;
    my = mGrid->dimY;
    mz = mGrid->dimZ; 

}

void FlipSolver::InitializeParticles() {
    
    float epsilon = 0.001;
    for (float i = -bbX; i + epsilon < bbX; i+= particleSeparation) {
        for (float j = -bbY; j + epsilon < bbY; j += particleSeparation) {
            for (float k = -bbZ; k + epsilon  < bbZ; k += particleSeparation) {
                
                if (withinFluidBounds(i, j, k) ) { //later make this a bounding box
                    Particle p;
                   
                    p.pos = glm::vec3(i, j, k);
                    
                    p.r = 0; p.g = 0; p.b = 255; p.a = 255; p.size = 0.1; //aesthetic choices lol
                    
                    p.speed = glm::vec3(0, 0, 0);
                    
                    //set grid index
                    p.gridIndex = mGrid->getGridIndex(p.pos);
                    p.gridIJK = mGrid->getGridIJK(p.pos);
                    mGrid->gridMarker->setValueAt(1, p.gridIndex);
                    ParticlesContainer.push_back(p);
                }
            }
        }
    }
    


    int numFluidCells = countFluidCells();
    mGrid->setNumFluidCells(numFluidCells);
    mGrid->particleCount = ParticlesContainer.size();
    mGrid->calculateAvgNumberOfParticlesPerGrid();
}


/// Initialize the grid and particle positions and velocities.
void FlipSolver::Init() {
    
    ConstructMACGrid();
    
    //mGrid->printMarker("before fluid marker");
    
    
    InitializeParticles();
    //mGrid->printMarker("after fluid marker");
    
    StoreParticleVelocitiesToGrid();

    //mGrid->printMarker("after velocities stored marker");


}


glm::vec3 getSolidVelocityNormal() {
    return glm::vec3(0, 0, 0);
}

void FlipSolver::FlipUpdate(float delta, float boxScaleX, float boxScaleY,
                            float boxScaleZ, glm::vec3 CameraPosition) {
    
    //mGrid->gridV->printContents("grid V after flip update");
    ///put particle onto grid
    StoreParticleVelocitiesToGrid();
    
    //mGrid->gridU->printContents("FLIP() PAR2GRID");
    mGrid->gridV->printContents("FLIP() PAR2GRID");
    //mGrid->gridW->printContents("FLIP() PAR2GRID");
   
    ///save old grids
    std::vector<float> deltaU(mGrid->gridU->data);
    std::vector<float> deltaV(mGrid->gridV->data);
    std::vector<float> deltaW(mGrid->gridW->data);
    
    ///update for gravity, data is now PIC velocities
    //mGrid->gridV->printContents("flip update() gridV before forces added ");
    mGrid->addExternalForcesToGrids(glm::vec3(0, -9.8, 0), delta); //-9.8 * delta);
    //mGrid->gridV->printContents("FLIP() ADD BODY FORCES ");
    
    ///calculate delta velocity
    for (int i = 0; i < mGrid->gridV->data.size(); i++) {
        deltaU[i] = mGrid->gridU->data[i] - deltaV[i];
        deltaV[i] = mGrid->gridV->data[i] - deltaV[i];
        deltaW[i] = mGrid->gridW->data[i] - deltaW[i];
    }
    
    mGrid->gridU->setDeltas(deltaU);
    mGrid->gridV->setDeltas(deltaV);
    mGrid->gridW->setDeltas(deltaW);
    
    //mGrid->gridU->printContents("FLIP() GRID U BEFORE PRESSURES");
    
    PressureSolve(delta);
    
    //mGrid->gridU->printContents("FLIP() GRID U AFTER PRESSURES");
    
    mGrid->extrapolateVelocities();
    //mGrid->gridV->printContents("FLIP() EXTRAPOLATE");
    
    
    MACGrid2Particle(delta, deltaU, deltaV, deltaW);
    
  
    //send drawing info to GPU
    update(delta, boxScaleX, boxScaleY, boxScaleZ, CameraPosition);
    
}



void FlipSolver::MACGrid2Particle(float delta, std::vector<float> deltaU,
                                  std::vector<float> deltaV,
                                  std::vector<float> deltaW) {
    


    ///put grid back onto each particle
    std::vector<Particle> updatedParticles;
    
    for (Particle p : ParticlesContainer) {
        
        //collision detection! if the particle leaves the container
        if (!container->insideContainer(p.pos)) {

            p.pos.y = -bbY;
            
            //reverse the speed
            p.speed = glm::vec3(0, 1, 0);
            
            //set particle color
            p.r = 0; p.g = 255; p.b = 255;
        }
        
        
        //GET TRILIN INTERP
        glm::vec3 poffset;
        
        poffset = glm::vec3(-bbX, -bbY, -bbZ) + glm::vec3(0, 0.5, 0.5);
        float picVelX = mGrid->gridU->getInterpedVelocity(p.pos, poffset, mGrid->gridU->data); //
        float flipVelX = mGrid->gridU->getInterpedVelocity(p.pos, poffset, deltaU);
        
        poffset = glm::vec3(-bbX, -bbY, -bbZ) + glm::vec3(0.5, 0, 0.5);
        float picVelY = mGrid->gridV->getInterpedVelocity(p.pos, poffset, mGrid->gridV->data); //
        float flipVelY = mGrid->gridV->getInterpedVelocity(p.pos, poffset, deltaV);
        
        poffset = glm::vec3(-bbX, -bbY, -bbZ) + glm::vec3(0.5, 0.5, 0);
        float picVelW = mGrid->gridW->getInterpedVelocity(p.pos, poffset, mGrid->gridW->data); //
        float flipVelW = mGrid->gridW->getInterpedVelocity(p.pos, poffset, deltaW);
        
        float picWeight = 0.05;
        glm::vec3 picVel(picWeight * picVelX, picWeight * picVelY, picWeight * picVelW);
        
        float flipWeight = 0.95;
        glm::vec3 flipVel(flipWeight * flipVelX, flipWeight * flipVelY, flipWeight * flipVelW);
        
        
        p.speed = picVel + flipVel;
        
        glm::vec3 fwdEulerPos = p.pos + p.speed * delta;
        glm::vec3 midpoint = glm::vec3((fwdEulerPos.x + p.pos.x)/2,
                                       (fwdEulerPos.y + p.pos.y)/2,
                                       (fwdEulerPos.z + p.pos.z)/2);
        
        glm::vec3 midEuler = picVel + flipVel; // super basic ho for now
        
        p.speed = midEuler;
        
        p.pos =  midpoint + midEuler * (delta/2.0f);

        
        p.gridIJK = mGrid->getGridIJK(p.pos);
        p.gridIndex = mGrid->getGridIndex(p.pos);
        
        p.r = 0; p.g = 0; p.b = 255;
        
        updatedParticles.push_back(p); //InterpolateVelocity(p.pos, *mGrid);
    }
    
    ParticlesContainer = updatedParticles;
}


bool FlipSolver::withinFluidBounds(float i, float j, float k) {
    return (i >= 0 && j >= 0 && k >= 0 && i < particleBoundX && j < particleBoundY && k < particleBoundZ);
    
    /*return (i >= -particleBoundX && j >= -particleBoundY && k >= -particleBoundZ && i < particleBoundX && j < particleBoundY && k < particleBoundZ);
    */
}

//calculate a weighted average of the particles' velocities
//in the neighborhood (define a neighborhood of 1 cell width) and store these
//values on corresponding grid cells.
//A simple stiff kernel can be used to calculate the weight for the average velocity.
void FlipSolver::StoreParticleVelocitiesToGrid(){
    
    //properly resets each grid
    mGrid->resetGrids();
    
    //mGrid->gridV->printContents("grid V reset to zero!");

    for (Particle p : ParticlesContainer) {
        
        int markerIndex = mGrid->gridMarker->ijkToGridIndex(p.gridIJK);
        mGrid->gridMarker->setValueAt(1, markerIndex); //cool so marker grid gets updated
        mGrid->storeParVelToGrids(p);
    }
    
    //mGrid->printMarker("particles stored to marker");
    mGrid->gridV->printContents("flipsolver::storepar2grid()  particles stored to gridV");
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
    return glm::vec3(); //mGrid.interpolateFromGrid(pos);
    
}


bool isInBoundsNeighbor(int i1, int j1, int k1, int i2, int j2, int k2, int maxN) {
    
    return i2 < maxN && j2 < maxN && k2 < maxN
            && i1 < maxN && j1 < maxN && k1 < maxN &&
    (fabs(i1 - i2) == 1 || fabs(j1 - j2 == 1) || fabs(k1 - k2 == 1));
}

bool FlipSolver::outOfBounds(int i, int j, int k) {
    
    return i >= mx || j >= my || k >= mz || i < 0 || j < 0 || k < 0;
}

bool FlipSolver::isSolid(int i, int j, int k) {
    return ((*mGrid->gridMarker))(i, j, k) < 0;
}


//might not need to pass in the val
// [] check if at boundary [] check if solid [] check if empty [] check if fluid

int FlipSolver::insertCoeff(int id, int i, int j, int k,
                            std::vector<Eigen::Triplet<float>>& coeffs) {
    bool printme;
 
    
    //calculate neighbor id from the grid dimensions
    int moi = i + j * mx + (k * my * mx);
    
    /*if (id == 112) {
        printme = true;
    }*/
    
    if (printme) {
        std::cout << "   moi " << moi << " ";
    }
    
    //boundary conditions! decr coeff by 1
    if (outOfBounds(i, j, k) || isSolid(i, j, k)) {
        if (printme) { std::cout << " was solid/oob"<< std::endl; }
        //coeffs.push_back(Eigen::Triplet<float> (id, moi, 0));
        return -1;
    }
    
    //if neighbor is fluid, push back value for coeffs,
    if (isFluid(i, j, k)) {
        if (printme) { std::cout << " was fluid"<< std::endl; }
        coeffs.push_back(Eigen::Triplet<float> (id, moi, -1));
        return 0;
    }

    //neighbor was empty
    else {
        if (printme) { std::cout << " was empty"<< std::endl; }
        //coeffs.push_back(Eigen::Triplet<float> (id, moi, 0));
        return 0;
    }
    
    if (printme) { std::cout<< std::endl;}
}


bool FlipSolver::isFluid(int i , int j , int k ) {
    return mGrid->isFluid(i, j, k);
}

void FlipSolver::PressureSolve(float dt) {
    
    //size of grids
    int n = mx * my * mz;
   
    Eigen::VectorXf x(n);
    Eigen::VectorXf b(n);
    Eigen::SparseMatrix<float> A(n,n);
    
    //triplets vector
    std::vector<Eigen::Triplet<float>> coeffs;

    
    //build the matrix of coefficients
    buildA(A, coeffs);
    buildb(b);
    
    
    Eigen::ConjugateGradient<Eigen::SparseMatrix<float> > cg;
    cg.compute(A);
    
    /*Eigen::LLT<Eigen::MatrixXd> lltOfA(A); // compute the Cholesky decomposition of A
     if(lltOfA.info() == Eigen::NumericalIssue)
    {   std::cout << "AYYYY" << A << std::endl;  //.isCompressed() << std::endl;
        std::cout << A.isVector() << std::endl;
        throw std::runtime_error("Possibly non semi-positive definite matrix!");
    }*/
    

    x = cg.solve(b);
    
    PressureUpdate(A, x, dt);
    
    
}

void FlipSolver::PressureUpdate(Eigen::SparseMatrix<float> &A, Eigen::VectorXf &p, float dt) {

    mGrid->UpdatePressureGrid(A, p, dt);


    mGrid->UpdateVelocityGridsByPressure(dt);
    
}

//pressure solving things
void FlipSolver::buildA(Eigen::SparseMatrix<float>& A, std::vector<Eigen::Triplet<float> >& coeffs) {
    
    A.setZero();

    
    for (int i = 0; i < mx; i++) {
        for (int j = 0; j < my; j++) {
            for (int k=0; k < mz; k++) {
                
                int pressure_id = i + (j * mx) + (k * mx * my);
                
                //fluid cell at ijk
                if (isFluid(i, j, k)) {
                    
                    //std::cout << " pressure id " << pressure_id << "  (" << i << ", " << j << ", " << k << ")" << std::endl;
                    
                    int fluidNeighborCount = 6;
                    
                    //-x neighbor
                    fluidNeighborCount += insertCoeff(pressure_id, i-1, j, k, coeffs);
                
                    
                    //+x neighbor
                    fluidNeighborCount +=insertCoeff(pressure_id, i+1,j, k, coeffs);
                    
                    
                    //-y neighbor
                    fluidNeighborCount += insertCoeff(pressure_id, i,j-1, k,coeffs);
                  
                    //+y neighbor
                    fluidNeighborCount += insertCoeff(pressure_id, i,j+1, k,  coeffs);
                
                    
                    //-z neighbor
                    fluidNeighborCount += insertCoeff(pressure_id, i,j, k-1, coeffs);
                    
                    
                    //+z neighbor
                    fluidNeighborCount += insertCoeff(pressure_id, i,j, k + 1, coeffs);
                
                    
                    //std::cout << " fluid neighbors: " << fluidNeighborCount << std::endl;
                    //current
                    coeffs.push_back(Eigen::Triplet<float> (pressure_id, pressure_id, fluidNeighborCount));
                    
                    
                }
                /*else {
                    //was solid or empty
                    //solid's div will be set to 0 in buildB
                    coeffs.push_back(Eigen::Triplet<float> (pressure_id, pressure_id, 1));
                }*/
               
            }
        }
    }
    
    A.setFromTriplets(coeffs.begin(), coeffs.end());
    
    std::cout << "AYY " << A << std::endl;
    
}



void FlipSolver::buildb(Eigen::VectorXf& b) {
    
    //do this once for each
    
    b.setZero();

    //loop over all cells to calculate the b
    for (int i = 0; i < mx; i++) {
        for (int j = 0; j < my; j++) {
            for (int k=0; k < mz; k++) {
                
                int id = i + (j * mx) + (k * mx * my);
                
                
                float div = 0;
                if (isFluid(i, j, k)) {
                    //calculate divergences in each direction
                    
                    float divU = mGrid->gridV->getDiv(i, j, k);
                    float divV = mGrid->gridU->getDiv(i, j, k);
                    float divW = mGrid->gridW->getDiv(i, j, k);
                    b(id) = -(divU + divV + divW);
                }
                
                else {
                    b(id) = 0;
                }

            }
        }
    }
    
     std::cout << "B " << b << std::endl;
}


int FlipSolver::countFluidCells() {
    //count number of fluid cells
    int numFluidCells = 0;
    for (int i = 0; i < mGrid->gridMarker->dimX; i++) {
        for (int j = 0; j < mGrid->gridMarker->dimY; j++) {
            for (int k = 0; k < mGrid->gridMarker->dimZ; k++) {
                if ((*(mGrid->gridMarker))(i, j, k) > 0 ) {
                    numFluidCells++;
                }
            }
        }
    }
    return numFluidCells;
    
}

void FlipSolver::enableGravity() {
    gravityEnabled = true;
}

void FlipSolver::disableGravity() {
    gravityEnabled = false;
}
