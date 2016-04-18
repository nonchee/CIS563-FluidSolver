#include "FlipSolver.h"

#include <Eigen/Dense>

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
    printf("number of particles generated %i", mGrid->particleCount);
    
    //printParticles("after init");
    
    MarkFluidCells();
    
    MarkSolidBoundaries();
    StoreParticlesToGrid();
    //mGrid->printMarker("MARKER AFTER INIT");
    //mGrid->gridV->printContents("GRIDV AFTER INIT");
    
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
    
    float negativeBounds = -1;
    float positiveBounds = 1;
    for (float i = negativeBounds; i < positiveBounds; i+= PSEP) {
        for (float j =  negativeBounds; j  < positiveBounds; j += PSEP) {
            for (float k = negativeBounds ; k   < positiveBounds ; k += PSEP) {
    
    
                    Particle p;
                   
                    p.pos = glm::vec3(i, j, k) + glm::vec3(DX/(2* PARTICLES_PER_CELL), DX/(2*PARTICLES_PER_CELL), DX/(2*PARTICLES_PER_CELL));
                
                    p.r = 0; p.g = 0; p.b = 255; p.a = 255; p.size = 0.1; //aesthetic choices lol
                    
                    p.speed = glm::vec3(0, 0, 0);
                
                    //set grid index
                    p.gridIndex = mGrid->posToIndex(p.pos);
                    p.gridIJK = mGrid->posToIJK(p.pos);
            
                    ParticlesContainer.push_back(p);
                    mGrid->particleCount++;
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



void FlipSolver::ExtrapolateVelocities() {
    
    //put velocity where fluids CAN GO
    //so that interpolation will work next step!!!
    //
    
    
    //ugh this is not correct at all
    //
    Grid<int>* tempMarker = new Grid<int>(mx, my, mz);
    
    for (int i = 0; i < mx; i++) {
        for (int j = 0; j < my; j++) {
            for (int k = 0; k < mz; k++) {
                if (isFluid(i, j, k) || isFluid(i, j - 1, k)) {
                    tempMarker->setValueAt(FLUID, i, j, k);
                }
                
                else {
                    tempMarker->setValueAt(AIR, i, j, k);
                }
            }
        }
    }
    
#ifdef DEBUG
    mGrid->printMarker("ACTUAL THINGS");
    tempMarker->printContents("TEMP MARKERRR");
#endif
        
    //extrapolating onto gridVV
    
    for (int i = 0; i < mGrid->gridU->dimX; i++) {
        for (int j = 0; j < mGrid->gridU->dimY; j++) {
            for (int k = 0; k < mGrid->gridU->dimZ; k++) {
            
                if ((*tempMarker)(i, j, k) != FLUID || outOfBounds(i, j, k)) {
                    
                    int fluidNeighborCount = 0;
                    float totalNeighbVelocities;
                    
                        
                    if ((*tempMarker)(i - 1 , j, k) == FLUID) {
                        totalNeighbVelocities+= (*mGrid->gridU)(i-1, j, k);
                        fluidNeighborCount++;
                    }
                    if ((*tempMarker)(i + 1, j, k) == FLUID) {
                        totalNeighbVelocities += (*mGrid->gridU)(i+1, j, k);
                         fluidNeighborCount++;
                        
                    }
                    if ((*tempMarker)(i, j +1, k) == FLUID) {
                        totalNeighbVelocities += (*mGrid->gridU)(i, j+1, k);
                        fluidNeighborCount++;
                    }
                    if ((*tempMarker)(i, j - 1, k) == FLUID) {
                        totalNeighbVelocities += (*mGrid->gridU)(i, j-1, k);
                         fluidNeighborCount++;
                    }
                    if ((*tempMarker)(i, j, k-1) == FLUID) {
                        totalNeighbVelocities += (*mGrid->gridU)(i, j, k-1);
                         fluidNeighborCount++;
                    }
                    if ((*tempMarker)(i, j, k+1) == FLUID) {
                        totalNeighbVelocities += (*mGrid->gridU)(i, j, k+1);
                        fluidNeighborCount++;
                    }
                    
                    if (fluidNeighborCount > 0) {
                        float avgNeighbU = totalNeighbVelocities / (float) fluidNeighborCount;
                        int gridIndex = mGrid->gridU->ijkToIndex(glm::vec3(i, j, k));
                        mGrid->gridU->setValueAt(avgNeighbU, gridIndex);
                    }
                    
                }
            }
        }
    }
    
    
    for (int i = 0; i < mGrid->gridV->dimX; i++) {
        for (int j = 0; j < mGrid->gridV->dimY; j++) {
            for (int k = 0; k < mGrid->gridV->dimZ; k++) {
                
                if ((*tempMarker)(i, j, k) != FLUID || outOfBounds(i, j, k)) {
                    
                    int fluidNeighborCount = 0;
                    float totalNeighbVelocities;
                    
                    
                    if ((*tempMarker)(i - 1 , j, k) == FLUID) {
                        totalNeighbVelocities+= (*mGrid->gridV)(i-1, j, k);
                        fluidNeighborCount++;
                    }
                    if ((*tempMarker)(i + 1, j, k) == FLUID) {
                        totalNeighbVelocities += (*mGrid->gridV)(i+1, j, k);
                        fluidNeighborCount++;
                        
                    }
                    if ((*tempMarker)(i, j +1, k) == FLUID) {
                        totalNeighbVelocities += (*mGrid->gridV)(i, j+1, k);
                        fluidNeighborCount++;
                    }
                    if ((*tempMarker)(i, j - 1, k) == FLUID) {
                        totalNeighbVelocities += (*mGrid->gridV)(i, j-1, k);
                        fluidNeighborCount++;
                    }
                    if ((*tempMarker)(i, j, k-1) == FLUID) {
                        totalNeighbVelocities += (*mGrid->gridV)(i, j, k-1);
                        fluidNeighborCount++;
                    }
                    if ((*tempMarker)(i, j, k+1) == FLUID) {
                        totalNeighbVelocities += (*mGrid->gridV)(i, j, k+1);
                        fluidNeighborCount++;
                    }
                    
                    if (fluidNeighborCount > 0) {
                        float avgNeighbV = totalNeighbVelocities / (float) fluidNeighborCount;
                        int gridIndex = mGrid->gridV->ijkToIndex(glm::vec3(i, j, k));
                        mGrid->gridV->setValueAt(avgNeighbV, gridIndex);
                    }
                    
                }
            }
        }
    }
    
    for (int i = 0; i < mGrid->gridW->dimX; i++) {
        for (int j = 0; j < mGrid->gridW->dimY; j++) {
            for (int k = 0; k < mGrid->gridW->dimZ; k++) {
                
                if ((*tempMarker)(i, j, k) != FLUID || outOfBounds(i, j, k)) {
                    
                    int fluidNeighborCount = 0;
                    float totalNeighbVelocities;
                    
                    
                    if ((*tempMarker)(i - 1 , j, k) == FLUID) {
                        totalNeighbVelocities+= (*mGrid->gridW)(i-1, j, k);
                        fluidNeighborCount++;
                    }
                    if ((*tempMarker)(i + 1, j, k) == FLUID) {
                        totalNeighbVelocities += (*mGrid->gridW)(i+1, j, k);
                        fluidNeighborCount++;
                        
                    }
                    if ((*tempMarker)(i, j +1, k) == FLUID) {
                        totalNeighbVelocities += (*mGrid->gridW)(i, j+1, k);
                        fluidNeighborCount++;
                    }
                    if ((*tempMarker)(i, j - 1, k) == FLUID) {
                        totalNeighbVelocities += (*mGrid->gridW)(i, j-1, k);
                        fluidNeighborCount++;
                    }
                    if ((*tempMarker)(i, j, k-1) == FLUID) {
                        totalNeighbVelocities += (*mGrid->gridW)(i, j, k-1);
                        fluidNeighborCount++;
                    }
                    if ((*tempMarker)(i, j, k+1) == FLUID) {
                        totalNeighbVelocities += (*mGrid->gridW)(i, j, k+1);
                        fluidNeighborCount++;
                    }
                    
                    if (fluidNeighborCount > 0) {
                        float avgNeighbW = totalNeighbVelocities / (float) fluidNeighborCount;
                        int gridIndex = mGrid->gridW->ijkToIndex(glm::vec3(i, j, k));
                        mGrid->gridW->setValueAt(avgNeighbW, gridIndex);
                    }
                    
                }
            }
        }
    }
    
    
    
    
}


void FlipSolver::FlipUpdate(){
    
    //ONE
    SetGridsToZero();
    MarkSolidBoundaries();
    MarkFluidCells();
    
#ifdef DEBUGONE
    mGrid->printMarker("SET MARKED PARTICLES");
    mGrid->gridV->printContents("GRIDV INIT");
#endif
    
    StoreParticlesToGrid();
#ifdef DEBUGTWO
    mGrid->printMarker("MARKED PARTICLES STILL");
    mGrid->gridV->printContents("PARTICLES STORED TO GRIDV");
#endif

    AddExternalForcesToGrids();
    
#ifdef DEBUG
    mGrid->gridV->printContents("gravity stored to gridV");
    mGrid->printMarker("marked after gravity");
#endif
    
    ApplyBoundaryConditions();
#ifdef DEBUGAPPLY
    mGrid->gridV->printContents("boundary enforced on gridV");
   
#endif
    
     //mGrid->printMarker("marked after boundaries");
    PressureUpdate();
    
    //FOUR
    ExtrapolateVelocities();
    
    ApplyBoundaryConditions();
    //mGrid->printMarker("these have fluids ");
    //mGrid->gridV->printContents("gridV after extrapolation");
    
    //FIVE
    //mGrid->gridV->printContents("these shall be stored to the particles");
    StoreGridToParticles();
    
    UpdateParticlePositions();

    
}


void FlipSolver::AddExternalForcesToGrids(){
    
    mGrid->addExternalForcesToGrids(glm::vec3(0, GRAVITY * DELTA_TIME, 0));
    
}


void FlipSolver::StoreGridToParticles() {

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
        
        p.gridIJK = mGrid->posToIJK(p.pos); 
        p.gridIndex = mGrid->ijkToIndex(p.gridIJK);

    
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
    
    //mGrid->printMarker("whoa nelly");

    glm::vec3 offset = glm::vec3(-bbX, -bbY, -bbZ);
    for (Particle p : ParticlesContainer) {
        
        mGrid->gridU->splatParticleVelocityToGrid(p, offset);
        mGrid->gridV->splatParticleVelocityToGrid(p, offset);
        mGrid->gridW->splatParticleVelocityToGrid(p, offset);
    }
    
    
    //normalize
    for (int i = 0; i < mGrid->gridU->data.size(); i++) {
        if (mGrid->gridU->totalWeights[i] != 0) {
            mGrid->gridU->data[i] /= (float) mGrid->gridU->totalWeights[i];
        }
    }
    //normalize
    for (int i = 0; i < mGrid->gridV->data.size(); i++) {
        if (mGrid->gridV->totalWeights[i] != 0) {
            mGrid->gridV->data[i] /= (float) mGrid->gridV->totalWeights[i];
        }
    }

    //normalize
    for (int i = 0; i < mGrid->gridW->data.size(); i++) {
        if (mGrid->gridW->totalWeights[i] != 0) {
            mGrid->gridW->data[i] /= (float) mGrid->gridW->totalWeights[i];
        }
    }
    
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
    int neighbor = i + j * mx + (k * my * mx);
    
    if (printme) {std::cout << "   neighbor " << neighbor << " "; }
    

    if (outOfBounds(i, j, k) || isSolid(i, j, k)) {
        if (printme) { std::cout << " was solid/oob"<< std::endl; }
        return 0;
    }
    
    //if neighbor is fluid, push back value for coeffs,
    if (isFluid(i, j, k)) {
        if (printme) { std::cout << " was fluid"<< std::endl; }
        coeffs.push_back(Eigen::Triplet<float> (id, neighbor, -scale));
        return scale;
    }

    //neighbor was empty
    else {
        if (printme) { std::cout << " was empty"<< std::endl; }
        return scale;
    }
    
    if (printme) { std::cout<< std::endl;}
}


bool FlipSolver::isFluid(int i, int j , int k ) {
    return (*mGrid->gridMarker)(i, j, k) > 0;
}


void FlipSolver::PressureUpdate() {
    
    //size of grids
    int n = mx * my * mz;
   
    Eigen::VectorXf p(n);
    Eigen::VectorXf b(n);
    Eigen::SparseMatrix<float> A(n,n);

    
    //build the matrix of coefficients
    //mGrid->printMarker("come on naow");
    A.setZero();
    buildA(A);
    
    b.setZero();
    buildb(b);
    
    Eigen::ConjugateGradient<Eigen::SparseMatrix<float> > cg;
    cg.compute(A);
    
    p.setZero();
    p = cg.solve(b);
    
    
//#define DEBUGPRESSURE
#ifdef DEBUGPRESSURE
    
    mGrid->gridP->printContents("gridP prepressure");
    mGrid->gridU->printContents("gridU prepressure");
    mGrid->gridV->printContents("gridV prepressure");
    mGrid->gridW->printContents("gridW prepressure");
    
#endif
    
    mGrid->UpdatePressureGrid(A, p);
    mGrid->UpdateVelocityGridsByPressure();
    

#ifdef DEBUGPRESSURE

    
    mGrid->gridP->printContents("gridP adjusted for pressure");
    mGrid->gridU->printContents("gridU adjusted for pressure");
    mGrid->gridV->printContents("gridV adjusted for pressure");
    mGrid->gridW->printContents("gridW adjusted for pressure");
    
    buildb(b);
#endif
    
    
    
}


//pressure solving things
void FlipSolver::buildA(Eigen::SparseMatrix<float>& A) {
    
    //triplets vector
    std::vector<Eigen::Triplet<float>> coeffs;
    
    for (int i = 0; i < mx; i++) {
        for (int j = 0; j < my; j++) {
            for (int k = 0; k < mz ; k++) {
                
                int id = i + (j * mx) + (k * mx * my);
                
                if (isFluid(i, j, k)) {
                    
                    float coeff = 0.0;
                    
                    coeff += insertCoeff(id, i-1, j, k, coeffs);
            
                    coeff +=insertCoeff(id, i+1,j, k, coeffs);
                    
                    coeff += insertCoeff(id, i,j-1, k,coeffs);

                    coeff += insertCoeff(id, i,j+1, k, coeffs);
                
                    coeff += insertCoeff(id, i,j, k-1, coeffs);
                    
                    coeff += insertCoeff(id, i,j, k+1, coeffs);
                
                    coeffs.push_back(Eigen::Triplet<float> (id, id, coeff));
                }
            }
        }
    }
    
    A.setFromTriplets(coeffs.begin(), coeffs.end());
    
//#define DEBUGAYY
#ifdef DEBUG
    std::cout << "AYY " << A << std::endl;
#endif

}



void FlipSolver::buildb(Eigen::VectorXf& b) {
    
    //do this once for each
    
    b.setZero();

    float scale = DELTA_TIME/(float) DX;
    
    //loop over all cells to calculate the b
    for (int i = 0; i < mx; i++) {
        for (int j = 0; j < my; j++) {
            for (int k=0; k < mz; k++) {
                
                if (isFluid(i, j, k)) {
                    float div = 0;
                    int id = i + (j * mx) + (k * mx * my);
                    //calculate divergences in each direction
                    
                    float divU = mGrid->gridU->getDiv(i, j, k);
                    float divV = mGrid->gridV->getDiv(i, j, k);
                    float divW = mGrid->gridW->getDiv(i, j, k);
                    
                    b(id) = -scale * (divU + divV + divW);
                }


            }
        }
    }
#define DEBUGB
#ifdef DEBUGB
    
    if (!(b.isZero(0))) {
        std::cout << "\n nuuuuuuu B \n" << b << std::endl;
    }
        
    
    
#endif
}

