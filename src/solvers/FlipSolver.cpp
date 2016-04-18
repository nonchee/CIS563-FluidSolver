#include "FlipSolver.h"

#define PICWEIGHT 0.05
#define FLIPWEIGHT 0.95
#define SOLID -1
#define AIR 0
#define FLUID 1


FlipSolver::FlipSolver(float boxScaleX, float boxScaleY , float boxScaleZ) {
    bbX = boxScaleX;
    bbY = boxScaleY;
    bbZ = boxScaleZ;
}

/// Initialize the grid and particle positions and velocities.
void FlipSolver::Init() {
    
    ConstructMACGrid();
    
    SetGridsToZero();
    
    InitializeParticles();
    
    printParticles("after init"); 
    
    MarkFluidCells();
    
    MarkSolidBoundaries();
    StoreParticlesToGrid();
    mGrid->printMarker("MARKER AFTER INIT");
    mGrid->gridV->printContents("GRIDV AFTER INIT");
    
}


void FlipSolver::ConstructMACGrid() {
    
    mGrid = new MACGrid(bbX, bbY, bbZ);
    std::cout << "box orig " << -bbX << " " << -bbY << " " << -bbZ << std::endl;
    std::cout << "box dims " << 2*bbX << " " << 2*bbY << " " << 2* bbZ << std::endl;
    std::cout << "PARTICLES PER CELL " << PARTICLES_PER_CELL << std::endl;
    
    mx = mGrid->dimX;
    my = mGrid->dimY;
    mz = mGrid->dimZ;
    
    
}

bool FlipSolver::withinFluidBounds(float i, float j, float k) {
    
    float wallWidth = 1;
    float fX = bbX - DX;
    float fY = bbY - wallWidth * DX;
    float fZ = bbZ - wallWidth * DX;
    
    return i > -(bbX - DX) && i < (bbX - DX) && j > -fY && j< fY
    && k > -fZ && k < fZ ;
    
}

void FlipSolver::InitializeParticles() {
    
    /*Particle p;
    
    p.pos = glm::vec3(0, 0, 0) + glm::vec3(DX/(2* PARTICLES_PER_CELL), DX/(2*PARTICLES_PER_CELL), DX/(2*PARTICLES_PER_CELL));
    
    p.r = 0; p.g = 0; p.b = 255; p.a = 255; p.size = 0.1; //aesthetic choices lol
    
    p.speed = glm::vec3(0, 0, 0);
    
    //set grid index
    p.gridIndex = mGrid->posToIndex(p.pos);
    p.gridIJK = mGrid->posToIJK(p.pos);
    
    ParticlesContainer.push_back(p);
    mGrid->particleCount++;*/
    
    for (float i = -0.5; i < 0.5; i+= PSEP) {
        for (float j =  -0.5; j  < 0.5; j += PSEP) {
            for (float k = -0.5 ; k   < 0.5 ; k += PSEP) {
    
    
    
    
                //if (withinFluidBounds(i, j, k)) {
                //if (!isSolid(i, j, k) {
                    Particle p;
                   
                    p.pos = glm::vec3(i, j, k) + glm::vec3(DX/(2* PARTICLES_PER_CELL), DX/(2*PARTICLES_PER_CELL), DX/(2*PARTICLES_PER_CELL));
                
                    p.r = 0; p.g = 0; p.b = 255; p.a = 255; p.size = 0.1; //aesthetic choices lol
                    
                    p.speed = glm::vec3(0, 0, 0);
                
                    //set grid index
                    p.gridIndex = mGrid->posToIndex(p.pos);
                    p.gridIJK = mGrid->posToIJK(p.pos);
            
                    ParticlesContainer.push_back(p);
                    mGrid->particleCount++;
               // }
            }
        }
    }
    
    

}




void FlipSolver::printParticles(std::string message) {
    
    std::cout << "----" <<message << "----"<<std::endl;
    printf("num particles: %i\n", mGrid->particleCount);
    
    for (Particle p : ParticlesContainer) {
     std::cout << " particle " << std::endl;
       std::cout << "    position " << glm::to_string(p.pos) << std::endl;
       std::cout << "    speed " << glm::to_string(p.speed) << std::endl;
       std::cout << "    ijk " << glm::to_string(p.gridIJK) << std::endl;
       std::cout << "    index " << glm::to_string(p.gridIndex) << std::endl;
       
       }
    
    std::cout << "--- done printing particles ----" << std::endl;
}

void FlipSolver::SetGridsToZero() {
    mGrid->resetGrids();
}

void FlipSolver::MarkSolidBoundaries() {
    
    for (int i = 0; i < mx; i++) {
        for (int j = 0; j < my; j++) { //don't mark the ceiling as solid
            for (int k = 0; k < mz; k++) {
                
                if (i == mx - 1 || i == 0 ||
                    j == 0 || j == my -1 ||
                    k == 0 || k == mz - 1) {
                    mGrid->gridMarker->setValueAt(SOLID, mGrid->ijkToIndex(glm::ivec3(i, j, k)));
                }
            }
        }
    }
    
}

void FlipSolver::ApplyBoundaryConditions() {
    mGrid->gridU->ApplyBoundaryConditions();
    mGrid->gridV->ApplyBoundaryConditions();
    mGrid->gridW->ApplyBoundaryConditions();
    
    
}
void FlipSolver::FlipUpdate(){
    
    //ONE
    SetGridsToZero();
    MarkSolidBoundaries();
    MarkFluidCells();
    
    StoreParticlesToGrid();
    mGrid->gridV->printContents("PARTICLES STORED TO GRIDV");
    mGrid->printMarker("SET MARKED PARTICLES");


    
    //TWO
    std::vector<float> deltaU(mGrid->gridU->data);
    std::vector<float> deltaV(mGrid->gridV->data);
    std::vector<float> deltaW(mGrid->gridW->data);
    

    AddExternalForcesToGrids();
    mGrid->gridV->printContents("gravity stored to gridV");
    mGrid->printMarker("marked after gravity");
    
    ApplyBoundaryConditions();
    mGrid->gridV->printContents("boundary enforced on gridV");
    mGrid->printMarker("marked after boundaries");
        //setsolidCellstoZeroForEachGrid();
    
    //THREE
   // PressureSolve(dt);
    
    //boundaryconditions()
    //setsolidCellstoZeroForEachGrid();
    
    //FOUR
   // mGrid->extrapolateVelocities();
    
    //FIVE
    //mGrid->gridV->printContents("these shall be stored to the particles");
    StoreGridToParticles(deltaU, deltaV, deltaW);
    mGrid->printMarker("marked after grid to particles");
    printParticles("\n STORED GRID TO PARTICLES");
   
    
    UpdateParticlePositions();
    
    printParticles("\n STORED GRID TO PARTICLES");
    mGrid->printMarker("marked after grid to particles");
    //send drawing info to GPU
    
}


void FlipSolver::AddExternalForcesToGrids(){
    
    mGrid->addExternalForcesToGrids(glm::vec3(0, GRAVITY * DELTA_TIME, 0));
    
}


void FlipSolver::StoreGridToParticles(std::vector<float> deltaU,
                                  std::vector<float> deltaV,
                                  std::vector<float> deltaW) {
    


    ///put grid back onto each particle
    std::vector<Particle> updatedParticles;
    
    for (Particle p : ParticlesContainer) {
        

        //GET TRILIN INTERP
        glm::vec3 poffset = glm::vec3(-bbX, -bbY, -bbZ);
        float picVelX = mGrid->gridU->getInterpedVelocity(p.pos, poffset, mGrid->gridU->data); //
        
        float picVelY = mGrid->gridV->getInterpedVelocity(p.pos, poffset, mGrid->gridV->data); //
        
        float picVelZ = mGrid->gridW->getInterpedVelocity(p.pos, poffset, mGrid->gridW->data); //
        

        //glm::vec3 flipVel(flipWeight * flipVelX, flipWeight * flipVelY, flipWeight * flipVelZ);
        
        p.speed = glm::vec3(picVelX, picVelY, picVelZ);
        //std::cout << " p speed " << glm::to_string(p.speed) << std::endl;

        updatedParticles.push_back(p); //InterpolateVelocity(p.pos, *mGrid);
    }
    
    ParticlesContainer = updatedParticles;
}

void FlipSolver::UpdateParticlePositions() {

    
    ///put grid back onto each particle
    std::vector<Particle> updatedParticles;
    
    for (Particle p : ParticlesContainer) {
        
        p.pos += (p.speed * DELTA_TIME);
        
        p.pos.x = p.pos.x < -bbX + DX ? -bbX + DX + 0.001 : (p.pos.x >= bbX ? bbX - DX - 0.001 : p.pos.x);
        p.pos.y = p.pos.y < -bbY + DX ? -bbY + DX +  0.001 : (p.pos.y >= bbY ? bbY - DX - 0.001 : p.pos.y);
        p.pos.z = p.pos.z < -bbZ + DX ? -bbZ + DX +  0.001 : (p.pos.z >= bbZ ? bbZ - DX - 0.001 : p.pos.z);

    
        p.r = 0; p.g = 0; p.b = 255;
        
        updatedParticles.push_back(p);

        }
    
    ParticlesContainer = updatedParticles;
}

//NOTE if your particles are moving sluggily perhaps it is because
//youa re dleta_timeing twiceover


void FlipSolver::MarkFluidCells() {
    for (Particle p : ParticlesContainer) {
        
        mGrid->storeParticleToMarker(p);
    }
}

void FlipSolver::ResetWeightCounts() {
    mGrid->gridU->totalWeights.clear();
    mGrid->gridV->totalWeights.clear();
    mGrid->gridW->totalWeights.clear();
    
    for (int i = 0; i < mGrid->gridU->data.size(); i++) {
        mGrid->gridU->totalWeights.push_back(0);
    }
    for (int i = 0; i < mGrid->gridV->data.size(); i++) {
        mGrid->gridV->totalWeights.push_back(0);
    }
    for (int i = 0; i < mGrid->gridW->data.size(); i++) {
        mGrid->gridW->totalWeights.push_back(0);
    }
}

void FlipSolver::StoreParticlesToGrid(){

   
    ResetWeightCounts();

    glm::vec3 offset = glm::vec3(-bbX, -bbY, -bbZ);
    for (Particle p : ParticlesContainer) {
        
        mGrid->gridU->splatParticleVelocityToGrid(p, offset);
        mGrid->gridV->splatParticleVelocityToGrid(p, offset);
        mGrid->gridW->splatParticleVelocityToGrid(p, offset);
    }
    
    
    //normalize
    for (int i = 0; i < mGrid->gridU->data.size(); i++) {
        if (mGrid->gridU->totalWeights[i] != 0) {
            //printf("not quite normalized at %i = %f\n", i, mGrid->gridU->data[i]);
            mGrid->gridU->data[i] /= (float) mGrid->gridU->totalWeights[i];
            //printf("normalized at %i = %f\n", i, mGrid->gridU->data[i]);
        }
    }
    //normalize
    
    //printf("normalizing grid V" );
    for (int i = 0; i < mGrid->gridV->data.size(); i++) {
        if (mGrid->gridV->totalWeights[i] != 0 &&  mGrid->gridMarker->data[i] != -1) {
            
            //printf("not yet normalized at %i = %f\n", i, mGrid->gridV->data[i]);
            mGrid->gridV->data[i] /= (float) mGrid->gridV->totalWeights[i];
            //printf("totalweight at %i =  %i \n", i,  mGrid->gridV->totalWeights[i]);
            //printf("normalized at %i = %f\n\n", i, mGrid->gridV->data[i]);
  
            //mGrid->gridV->data[i] = 0.f; //nothing splatted to the cell
            
           /* printf("totalweight at %i =  %i \n", i,  mGrid->gridV->totalWeights[i]);
            printf("normalized at %i = %f\n\n", i, mGrid->gridV->data[i]);*/
        }
    }
    
    //normalize
    for (int i = 0; i < mGrid->gridW->data.size(); i++) {
        if (mGrid->gridW->totalWeights[i] != 0) {
            mGrid->gridW->data[i] /= (float) mGrid->gridW->totalWeights[i];
        }
    }
    
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



float FlipSolver::insertCoeff(int id, int i, int j, int k,
                            std::vector<Eigen::Triplet<float>>& coeffs) {
    bool printme;
    float scale = DELTA_TIME /(DX * DX);
 
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
        return -scale;
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

void FlipSolver::ComputePressures() {
    
    //size of grids
    int n = mx * my * mz;
   
    Eigen::VectorXf p(n);
    Eigen::VectorXf b(n);
    Eigen::SparseMatrix<float> A(n,n);
    

    
    //build the matrix of coefficients
    A.setZero();
    buildA(A);
    
    buildb(b);
    
    
    Eigen::ConjugateGradient<Eigen::SparseMatrix<float> > cg;
    cg.compute(A);
    
    /*Eigen::LLT<Eigen::MatrixXd> lltOfA(A); // compute the Cholesky decomposition of A
     if(lltOfA.info() == Eigen::NumericalIssue)
    {   std::cout << "AYYYY" << A << std::endl;  //.isCompressed() << std::endl;
        std::cout << A.isVector() << std::endl;
        throw std::runtime_error("Possibly non semi-positive definite matrix!");
    }*/
    

    p = cg.solve(b);
    AdjustForPressure(A, p);
    
}

void FlipSolver::AdjustForPressure(Eigen::SparseMatrix<float> &A, Eigen::VectorXf &p) {

    mGrid->UpdatePressureGrid(A, p);


    mGrid->UpdateVelocityGridsByPressure();
    
}

//pressure solving things
void FlipSolver::buildA(Eigen::SparseMatrix<float>& A) {
    
    //triplets vector
    std::vector<Eigen::Triplet<float>> coeffs;
    

    
    for (int i = 1; i < mx -1; i++) {
        for (int j = 1; j < my - 1; j++) {
            for (int k=1; k < mz - 1; k++) {
                
                int pressure_id = i + (j * mx) + (k * mx * my);
                
                //fluid cell at ijk
                if (isFluid(i, j, k)) {
                    
                    float coeff = 0.0;
                    
                    //coeff = scale * number of FLUID OR AIR
                    //neighbor = -scale if fluid
                    //neighbor = 0 otherwise
                    
                    //-x neighbor
                    coeff += insertCoeff(pressure_id, i-1, j, k, coeffs);
                
                    
                    //+x neighbor
                    coeff +=insertCoeff(pressure_id, i+1,j, k, coeffs);
                    
                    
                    //-y neighbor
                    coeff += insertCoeff(pressure_id, i,j-1, k,coeffs);
                  
                    //+y neighbor
                    coeff += insertCoeff(pressure_id, i,j+1, k,  coeffs);
                
                    
                    //-z neighbor
                    coeff += insertCoeff(pressure_id, i,j, k-1, coeffs);
                    
                    
                    //+z neighbor
                    coeff += insertCoeff(pressure_id, i,j, k + 1, coeffs);
                
                    
                    //current
                    coeffs.push_back(Eigen::Triplet<float> (pressure_id, pressure_id, coeff));
                    
                    
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
    
    //std::cout << "AYY " << A << std::endl;
    
}



void FlipSolver::buildb(Eigen::VectorXf& b) {
    
    //do this once for each
    
    b.setZero();

    float scale = 1.0f/DX;
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
                    b(id) = -scale * (divU + divV + divW);
                }
                
                else {
                    b(id) = 0;
                }

            }
        }
    }
    
    // std::cout << "B " << b << std::endl;
}

