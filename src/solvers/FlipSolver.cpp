#include "FlipSolver.h";
#include <Eigen/IterativeLinearSolvers>
#include <Eigen/Sparse>
#include <Eigen/Core>
#include <Eigen/Dense>



FlipSolver::FlipSolver(Geom* g) {
    container = g;
}

/// Create the MAC grid ///
void FlipSolver::ConstructMACGrid() {
    
    //particleBoundX = fluidBoundX;
    //particleBoundY = fluidBoundY;
    //particleBoundZ = fluidBoundZ;
    //particleSeparation = 0.6;
    
    //each grid dimension is enough so that a gridcell holds about 8 particles total
    float gridCellSidelength = particleSeparation * 2;
    
    std::cout << "aaaaaaagh" << std::endl;
    std::cout << container->boxBoundX << std::endl;
    std::cout << container->boxBoundY << std::endl;
    std::cout <<  container->boxBoundZ << std::endl;
   
    //construct a MAC grid out of container dimensions
    mGrid = new MACGrid(container->boxBoundX,
                        container->boxBoundY,
                        container->boxBoundZ,
                        gridCellSidelength);
}

void FlipSolver::InitializeParticles() {
    //IT WAS FLOATING POINT ERRORS THE WHOLE TIME
    float epsilon = 0.001;
    
    std::cout << " why are you outside your container" << std::endl;
    std::cout << particleBoundX << std::endl;
    std::cout << particleBoundY << std::endl;
    std::cout << particleBoundZ << std::endl;
    
    //9 total particles along each axis
    for (float i = 0; i + epsilon < particleBoundX; i+= particleSeparation) {
        for (float j = 0; j + epsilon < particleBoundY; j += particleSeparation) {
            for (float k = 0; k + epsilon  < particleBoundZ; k += particleSeparation) {
                
                if (withinFluidBounds(i, j, k) ) {
                    
                    Particle p;
                    
                    glm::vec3 jitter = glm::vec3(0.01 * rand(), 0.01 * rand(), 0.01 * rand());
                    p.pos = glm::vec3(i, j, k); //+ jitter;
                    //p.speed = glm::vec3(0, -9.8, 0);
                    
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
    


    int numFluidCells = countFluidCells();
    mGrid->setNumFluidCells(numFluidCells);
    mGrid->particleCount = ParticlesContainer.size();
    mGrid->calculateAvgNumberOfParticlesPerGrid();
    
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

/// Initialize the grid and particle positions and velocities.
void FlipSolver::Init() {
    
    ConstructMACGrid();
    
    InitializeParticles();
    
    StoreParticleVelocitiesToGrid();
    //mGrid->gridV->printContents("particles stored to gridV");
    mGrid->gridMarker->printContents("after fluid marker");

}


glm::vec3 getSolidVelocityNormal() {
    return glm::vec3(0, 0, 0);
}

void FlipSolver::FlipUpdate(float delta, float boxScaleX, float boxScaleY,
                            float boxScaleZ, glm::vec3 CameraPosition) {
    
    mGrid->gridV->printContents("flip update");
    ///put particle onto grid
    StoreParticleVelocitiesToGrid();

    

    ///extrapolate velocities
    mGrid->extrapolateVelocities();   //markerGrid.data());

    
    ///save old gridV
    std::vector<float> deltaU(mGrid->gridU->data);
    std::vector<float> deltaV(mGrid->gridV->data);
    std::vector<float> deltaW(mGrid->gridW->data);
    
    ///update for gravity, data is now PIC velocities
    mGrid->gridV->addForce(0.1 * delta); 
    mGrid->gridV->addForce(-9.8 * delta);
    
    ///calculate delta velocity
    for (int i = 0; i < mGrid->gridV->data.size(); i++) {
        deltaU[i] = mGrid->gridU->data[i] - deltaU[i];
        deltaV[i] = mGrid->gridV->data[i] - deltaV[i];
        deltaW[i] = mGrid->gridW->data[i] - deltaW[i];
    }
    
    
    mGrid->gridV->setDeltas(deltaV);
   // std::cout << "now hur " << mGrid->gridV->delta.size() << std::endl;
    
    //PressureSolve(delta);
    
    float dummyPIC = mGrid->gridV->data[0];
    float dummyFLIP = deltaV[0];
 
    ///put grid back onto each particle
    std::vector<Particle> updatedParticles;
    
    for (Particle p : ParticlesContainer) {
        
        //collision detection! if the particle leaves the container
        if (!container->insideContainer(p.pos)) {
            ///do bounce
            //previously:
            //std::cout << "particle went out of bounds " << glm::to_string(p.pos) << std::endl;
            p.pos.y = -container->boxBoundY;
            
            //reverse the speed
            p.speed = glm::vec3(0, 1, 0);
            
            //set particle color
            p.r = 0; p.g = 255; p.b = 255;
        }
        
        //getInterpolatedVelocity(p.pos);
        
        //giving a dummy speed based on gridV * delta
        glm::vec3 picVel = glm::vec3(0, 0.05 * dummyPIC, 0);
        glm::vec3 flipVel = glm::vec3(0, 0.95 * dummyFLIP, 0);
        
        p.speed = picVel + flipVel;
        
        //p.updatePositionWithRK2();
        glm::vec3 fwdEulerPos = p.pos + p.speed;
        glm::vec3 midpoint = glm::vec3((fwdEulerPos.x + p.pos.x)/2,
                                       (fwdEulerPos.y + p.pos.y)/2,
                                       (fwdEulerPos.z + p.pos.z)/2);
        
        glm::vec3 midEuler = picVel + flipVel; // super basic ho for now

        p.speed = midEuler;
        
        p.pos =  midpoint + midEuler * delta/2.0f;
        
        p.r = 0;
        p.g = 0;
        p.b = 255;
        p.a = 255;

        updatedParticles.push_back(p); //InterpolateVelocity(p.pos, *mGrid);
    }
    
    
   
    ParticlesContainer = updatedParticles;
  
    //send drawing info to GPU
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
   // mGrid->gridV->printContents("grid V reset to zero!");

    for (Particle p : ParticlesContainer) {
        mGrid->gridMarker->addValueAt(1, p.gridIndex); //cool so marker grid gets updated
        mGrid->storeParticleVelocityToGrid(p);
    }
    
    //mGrid->gridMarker->printContents("particles stored to marker");
    //mGrid->gridV->printContents("particles stored to gridV");
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


bool isInBoundsNeighbor(int i1, int j1, int k1, int i2, int j2, int k2, int maxN) {
    
    return i2 < maxN && j2 < maxN && k2 < maxN
            && i1 < maxN && j1 < maxN && k1 < maxN &&
    (fabs(i1 - i2) == 1 || fabs(j1 - j2 == 1) || fabs(k1 - k2 == 1));
}

bool FlipSolver::outOfBounds(int i, int j, int k) {
    
    int mx = mGrid->dimX;
    int my = mGrid->dimY;
    int mz = mGrid->dimZ;
    return i >= mx || j >= my || k >= mz || i < 0 || j < 0 || k < 0;
}

bool FlipSolver::isSolid(int i, int j, int k) {
    return ((*mGrid->gridMarker))(i, j, k) < 0;
}


//might not need to pass in the val
// [] check if at boundary [] check if solid [] check if empty [] check if fluid

int FlipSolver::insertCoeff(int id, int i, int j, int k, std::vector<Eigen::Triplet<float>>& coeffs) {
    /*int mx = particleBoundX;
    int my = particleBoundY;
    int mz = particleBoundZ;
    */
    int mx = mGrid->dimX;
    int my = mGrid->dimY;
    int mz = mGrid->dimZ;
    
    //calculate neighbor id from the dimensions
    int myID = i + j * mx + (k * my * mx);
    
    if (outOfBounds(i, j, k)) {
        return -1;
    }

    ///std::cout << i << " " << j << "  " << k << std::endl;
    int marker = (*(mGrid->gridMarker))(i, j, k);
    ///std::cout << marker << std::endl;
    //get type of cell
   // mGrid->gridMarker->printContents("am i just a bunch of zeros");
    
    //at boundary
    //solids decrement the coeff by -1, and removes neighbor from consideration
  
    if(marker < 0) { //isSolid
        return -1;
    }
    
    //if neighbor is fluid, push back value for coeffs,
    if (marker > 0) {
        //std::cout << myID << " " << id << " " << "set to -1" << std::endl;
        coeffs.push_back(Eigen::Triplet<float> (myID, id, -1));
        return 0;
    }

    //else was air, and don't need to decrement count or fluid cell
    
}


bool FlipSolver::isFluid(int i , int j , int k ) {
    //mGrid->gridMarker->printContents("this be the marker grid");
    //std::cout << i << " " << j << " " << k << " was fluid " << ((*(mGrid->gridMarker))(i, j, k)) << std::endl;
    
    return ((*(mGrid->gridMarker))(i, j, k) > 0);
    
}

void FlipSolver::PressureSolve(float dt) {
    
    //just basic ho make this the size of the grid
    //and not try to only make it fluids
    int n = mGrid->dimX * mGrid->dimY * mGrid->dimZ;
    std::cout << " n vs marker grid size" << n << "  " << (mGrid->gridMarker->data.size()) <<std::endl;
    Eigen::VectorXf x(n);
    Eigen::VectorXf b(n);
    Eigen::SparseMatrix<float> A(n,n);
    
    //triplets vector
    std::vector<Eigen::Triplet<float>> coeffs;

    
    //build the matrix of coefficients
    buildA(A, coeffs);
    buildb(b);
    
    //std::cout << "AYYY " << A << std::endl;
    //std::cout << " BBBBB" << std::endl << b << std::endl;
    
    
    Eigen::ConjugateGradient<Eigen::SparseMatrix<float> > cg;
    cg.compute(A);
    
    Eigen::LLT<Eigen::MatrixXd> lltOfA(A); // compute the Cholesky decomposition of A
    if(lltOfA.info() == Eigen::NumericalIssue)
    {
        std::cout << "AYYYY" << A << std::endl;  //.isCompressed() << std::endl;
        //        std::cout << A.isVector() << std::endl;
        throw std::runtime_error("Possibly non semi-positive definite matrix!");
    }
    

    x = cg.solve(b);
    
    //x /= 10000000000000;
    // std::cout << "#iterations:     " << cg.iterations() << std::endl;
    std::cout << (cg.error() > 0) << std::endl;
    //std::cout << " determinant " << ((Eigen::MatrixXd(A).inverse()))<< std::endl;
    std::cout << "estimated error: " << cg.error()      << std::endl;
     // update b, and solve again
    std::cout << "solved and this is how it turned out " << x;
    
    //x = cg.solve(b);
    
    std::cout << std::endl << "solved and this is how it turned out " << x;
    //std::cout << "----- lol time for pressure update ---- " << std::endl;
    
    
    PressureUpdate(A, x, dt);
    
    
}

void FlipSolver::PressureUpdate(Eigen::SparseMatrix<float> &A, Eigen::VectorXf &p, float dt) {
    int mx = mGrid->dimX;
    int my = mGrid->dimY;
    int mz = mGrid->dimZ;
    float dx = mGrid->cellSidelength;
    float scale = dt/(dx * dx);
    
   /* float dx = mGrid->cellSidelength;
    float scale = dt/(dx * dx);
    
    for (int i = 0; i < mx; i++) {
        for (int j = 0; i < my; j++) {
             for (int k = 0; k < mz; k++) {
                 
                 int gridIndex = i + j * mx + k * mx * my;
                 mGrid->gridU->pressureUpdate(gridIndex, p[gridIndex]);
                 mGrid->gridV->pressureUpdate(gridIndex, p[gridIndex]);
                 mGrid->gridW->pressureUpdate(gridIndex, p[gridIndex]);
             }
        }
    }*/
    for (int k=0; k<A.outerSize(); ++k) {
        
       // int prevPressure = 0;
        
        for (Eigen::SparseMatrix<float>::InnerIterator it(A,k); it; ++it)
            {
                int count = it.value();
                int id = it.index();
                
               // prevPressure = p[id];
                
               // float pressureCorrection = 0;
                
              /* lol there must be a better way.
               //-x neighbor
                pressureCorrection += insertCoeff(id -1, coeffs);
                
                //+x neighbor
                fluidNeighborCount +=insertCoeff(id +1, coeffs);
                
                //-y neighbor
                fluidNeighborCount += insertCoeff(id - mx, coeffs);
                
                //+y neighbor
                fluidNeighborCount += insertCoeff(id + mx, coeffs);
                
                //-z neighbor
                fluidNeighborCount += insertCoeff(id + mx * my), coeffs);
                
                //+z neighbor
                fluidNeighborCount += insertCoeff(id - mx * my, coeffs);*/
                
                
                // row index; might be it.col() if not careful luls.
                //flkadsflkjasldfjaksldjfklasjfkla
                //keep it in a map outside?
                
                float pressureChange = p[id]; //- prevPressure;
               /* std::cout << "pressure at " << id << ": " << p[id] << " id - 1: " << prevPressure << std::endl;*/
                //std::cout << "update by " << (count * pressureChange * scale) << " lol" << std::endl;
                //fill pressure
                mGrid->gridP->setValueAt(count * pressureChange * scale, id);
               
            }
    }
    
    //update by central differences
    mGrid->gridU->pressureUpdate(mGrid->gridP);
    mGrid->gridV->pressureUpdate(mGrid->gridP);
    mGrid->gridW->pressureUpdate(mGrid->gridP);
}

//pressure solving things
void FlipSolver::buildA(Eigen::SparseMatrix<float>& A, std::vector<Eigen::Triplet<float> >& coeffs) {
    int mx = mGrid->dimX;
    int my = mGrid->dimY;
    int mz = mGrid->dimZ;
    
    
    std::cout << "A.ROWS " << A.rows() << std::endl;
    std::cout << "A.COLS " << A.cols() << std::endl;
    std::cout << mx << " x " <<  my << " x " << mz << std::endl;
    
    //loop over all cells to calculate the A matrix
    for (int i = 0; i < mx; i++) {
        for (int j = 0; j < my; j++) {
            for (int k=0; k < mz; k++) {
                
                int pressure_id = i + (j * mx) + (k * mx * my);
                
                //fluid cell at ijk
                if (isFluid(i, j, k)) {
                    int fluidNeighborCount = 6;
                    
                    //-x neighbor
                    fluidNeighborCount += insertCoeff(pressure_id, i-1, j, k, coeffs);
                   
                   // int tryMe = i -1 + (j * mx) + (k * mx * my);
                   // std::cout << coeffs[tryMe] std::endl;
                    
                    //+x neighbor
                    fluidNeighborCount +=insertCoeff(pressure_id, i+1,j, k, coeffs);
                    
                    
                    //-y neighbor
                    fluidNeighborCount +=
                    insertCoeff(pressure_id, i,j-1, k,coeffs);
                  
                    //+y neighbor
                    fluidNeighborCount += insertCoeff(pressure_id, i,j+1, k,  coeffs);
                
                    
                    //-z neighbor
                    fluidNeighborCount += insertCoeff(pressure_id, i,j, k-1, coeffs);
                    
                    
                    //+z neighbor
                    fluidNeighborCount += insertCoeff(pressure_id, i,j, k + 1, coeffs);
                
                    
                    //current
                    coeffs.push_back(Eigen::Triplet<float> (pressure_id, pressure_id, fluidNeighborCount));
                    
                    
                }
                else {
                    //was solid
                    //solid's div will be set to 0 in buildB
                    coeffs.push_back(Eigen::Triplet<float> (pressure_id, pressure_id, 1));
                }
            }
        }
    }
    
    A.setFromTriplets(coeffs.begin(), coeffs.end());
    
}



void FlipSolver::buildb(Eigen::VectorXf& b) {
    

    /*int mx = particleBoundX;
    int my = particleBoundY;
    int mz = particleBoundZ;*/
    
    int mx = mGrid->dimX;
    int my = mGrid->dimY;
    int mz = mGrid->dimZ;
    
    //loop over all cells to calculate the A matrix
    for (int i = 0; i < mx; i++) {
        for (int j = 0; j < my; j++) {
            for (int k=0; k < mz; k++) {
        
                float div = 0;
                
                int id = i + (j * mx) + (k * mx * my);
                //if fluid
                if ((*(mGrid->gridMarker))(i, j, k) > 0) {
                    //calculate divergences in each direction
                    div += mGrid->gridU->getDelta(i, j, k);
                    div += mGrid->gridV->getDelta(i, j, k);
                    div += mGrid->gridW->getDelta(i, j, k);
                }
                
                b(id) -= div;
            }
        }
    }
}

