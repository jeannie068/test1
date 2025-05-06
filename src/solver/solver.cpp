#include "../solver/solver.hpp"
#include <iostream>
#include <ctime>
#include <algorithm>
#include <limits>
using namespace std;

/**
 * Constructor
 */
PlacementSolver::PlacementSolver()
    : hbTree(nullptr),
      initialTemperature(1000.0),
      finalTemperature(0.1),
      coolingRate(0.95),
      iterationsPerTemperature(100),
      noImprovementLimit(1000),
      probRotate(0.3),
      probMove(0.3),
      probSwap(0.3),
      probChangeRep(0.05),
      probConvertSym(0.05),
      areaWeight(1.0),
      wirelengthWeight(0.0),
      randomSeed(static_cast<unsigned int>(time(nullptr))),
      totalArea(0) {
}

PlacementSolver::~PlacementSolver() {

}

void PlacementSolver::setTimeoutManager(std::shared_ptr<TimeoutManager> manager) {
    timeoutManager = manager;
}

/**
 * Loads modules and symmetry constraints
 */
void PlacementSolver::loadProblem(const map<string, shared_ptr<Module>>& modules,
                                 const vector<shared_ptr<SymmetryGroup>>& symmetryGroups) {
    this->modules = modules;
    this->symmetryGroups = symmetryGroups;
    
    // Create a new HB*-tree
    hbTree = make_shared<HBStarTree>();
    
    // Add modules and symmetry groups to the HB*-tree
    for (const auto& pair : modules) {
        hbTree->addModule(pair.second);
    }
    
    for (const auto& group : symmetryGroups) {
        hbTree->addSymmetryGroup(group);
    }
}

/**
 * Creates an initial placement solution
 */
void PlacementSolver::createInitialSolution() {
    // Check if modules and symmetry groups are loaded
    if (modules.empty()) {
        cerr << "Error: No modules loaded." << endl;
        return;
    }
    
    // Construct an improved initial HB*-tree - Use the new method instead of constructInitialTree
    hbTree->constructImprovedInitialTree();
    
    // Pack the tree to get initial coordinates
    hbTree->pack();
    
    // Output initial area
    cout << "Initial area: " << hbTree->getArea() << endl;
}

/**
 * Sets simulated annealing parameters
 */
void PlacementSolver::setAnnealingParameters(double initialTemp, double finalTemp, double coolRate, 
                                           int iterations, int noImprovementLimit) {
    initialTemperature = initialTemp;
    finalTemperature = finalTemp;
    coolingRate = coolRate;
    iterationsPerTemperature = iterations;
    this->noImprovementLimit = noImprovementLimit;
}

/**
 * Sets perturbation probabilities
 */
void PlacementSolver::setPerturbationProbabilities(double rotate, double move, double swap, 
                                                 double changeRep, double convertSym) {
    // Check if probabilities sum to 1.0
    double sum = rotate + move + swap + changeRep + convertSym;
    if (abs(sum - 1.0) > 1e-6) {
        // Normalize probabilities to sum to 1.0
        if (sum <= 0.0) {
            // Default values if all probabilities are zero or negative
            probRotate = 0.3;
            probMove = 0.3;
            probSwap = 0.3;
            probChangeRep = 0.05;
            probConvertSym = 0.05;
            return;
        }
        
        probRotate = rotate / sum;
        probMove = move / sum;
        probSwap = swap / sum;
        probChangeRep = changeRep / sum;
        probConvertSym = convertSym / sum;
    } else {
        probRotate = rotate;
        probMove = move;
        probSwap = swap;
        probChangeRep = changeRep;
        probConvertSym = convertSym;
    }
}

/**
 * Sets cost function weights
 */
void PlacementSolver::setCostWeights(double area, double wirelength) {
    areaWeight = area;
    wirelengthWeight = wirelength;
}

/**
 * Sets random seed for reproducibility
 */
void PlacementSolver::setRandomSeed(unsigned int seed) {
    randomSeed = seed;
}

/**
 * Solves the placement problem using simulated annealing
 */
bool PlacementSolver::solve() {
    // Create initial solution if not already created
    if (!hbTree || !hbTree->getRoot()) {
        createInitialSolution();
    }
    
    if (!hbTree || !hbTree->getRoot()) {
        std::cerr << "Error: Failed to create initial solution." << std::endl;
        return false;
    }
    
    std::cout << "Starting simulated annealing..." << std::endl;
    std::cout << "Initial temperature: " << initialTemperature << std::endl;
    std::cout << "Final temperature: " << finalTemperature << std::endl;
    std::cout << "Cooling rate: " << coolingRate << std::endl;
    std::cout << "Iterations per temperature: " << iterationsPerTemperature << std::endl;
    std::cout << "No improvement limit: " << noImprovementLimit << std::endl;
    
    // Create the simulated annealing solver with optimized parameters
    SimulatedAnnealing sa(hbTree, 
                          1000.0,     // Initial temperature
                          1.0,        // Higher final temperature to stop earlier
                          0.85,       // More aggressive cooling
                          250,        // More iterations at high temperatures
                          500);       // Lower no improvement limit
    
    // Let the adaptive perturbation handle probabilities
    // These are just initial values that will be tuned during the SA process
    sa.setPerturbationProbabilities(0.3, 0.4, 0.2, 0.05, 0.05);
    sa.setCostWeights(areaWeight, wirelengthWeight);
    sa.setSeed(randomSeed);
    
    // Pass the timeout manager
    if (timeoutManager) {
        sa.setTimeoutManager(timeoutManager);
    }
    
    // Check for timeout before starting
    if (timeoutManager && timeoutManager->hasTimedOut()) {
        std::cout << "Timeout detected before starting SA." << std::endl;
        return false;
    }
    
    // Run simulated annealing
    std::shared_ptr<HBStarTree> result = nullptr;
    try {
        result = sa.run();
        if (!result) {
            std::cerr << "Error: Simulated annealing failed to find a solution." << std::endl;
            return false;
        }
    }
    catch (const std::runtime_error& e) {
        std::string errorMsg = e.what();
        if (errorMsg.find("Timeout") != std::string::npos) {
            std::cout << "SA process was interrupted by timeout." << std::endl;
            // Use the best solution found so far
            result = sa.getBestSolution();
            if (!result) {
                std::cerr << "No solution available after timeout." << std::endl;
                return false;
            }
        } else {
            throw; // Re-throw other runtime errors
        }
    }
    
    // Update the HB*-tree with the best solution
    hbTree = result;
    
    // Ensure the solution is packed (should already be, but just to be safe)
    try {
        hbTree->pack();
    }
    catch (const std::runtime_error& e) {
        std::string errorMsg = e.what();
        if (errorMsg.find("Timeout") != std::string::npos) {
            std::cout << "Packing was interrupted by timeout. Using partial results." << std::endl;
        } else {
            throw; // Re-throw other runtime errors
        }
    }
    
    // Update statistics
    totalArea = hbTree->getArea();
    
    std::cout << "Simulated annealing completed." << std::endl;
    std::cout << "Final area: " << totalArea << std::endl;
    
    // Print statistics
    auto stats = sa.getStatistics();
    std::cout << "Total iterations: " << stats["totalIterations"] << std::endl;
    std::cout << "Accepted moves: " << stats["acceptedMoves"] << std::endl;
    std::cout << "Rejected moves: " << stats["rejectedMoves"] << std::endl;
    std::cout << "No improvement count: " << stats["noImprovementCount"] << std::endl;
    
    return true;
}

/**
 * Gets the solution area
 */
int PlacementSolver::getSolutionArea() const {
    return totalArea;
}

/**
 * Gets the solution modules with their positions
 */
map<string, shared_ptr<Module>> PlacementSolver::getSolutionModules() const {
    return modules;
}

/**
 * Gets placement solution statistics
 */
map<string, int> PlacementSolver::getStatistics() const {
    map<string, int> stats;
    stats["totalArea"] = totalArea;
    return stats;
}

/**
 * Finalizes the solution - ensures the HB*-tree is packed and area is calculated
 * This is crucial to call before getting solution area, especially after timeout
 */
void PlacementSolver::finalizeSolution() {
    // Make sure we have a valid HB*-tree
    if (!hbTree || !hbTree->getRoot()) {
        std::cerr << "Error: No solution to finalize" << std::endl;
        totalArea = 0;
        return;
    }
    
    try {
        // Ensure the tree is packed to get latest coordinates
        hbTree->pack();
        
        // Calculate the area
        totalArea = hbTree->getArea();
        
        // Debug output
        std::cout << "Solution finalized - Area: " << totalArea << std::endl;
    }
    catch (const std::exception& e) {
        std::cerr << "Error finalizing solution: " << e.what() << std::endl;
        
        // Try a more cautious approach to get some valid area
        // This is a fallback mechanism for when we have a solution but packing fails
        int minX = std::numeric_limits<int>::max();
        int minY = std::numeric_limits<int>::max();
        int maxX = 0;
        int maxY = 0;
        
        bool validCoordinates = false;
        
        // Calculate bounding box from module positions
        for (const auto& pair : modules) {
            const auto& module = pair.second;
            if (module) {
                int x = module->getX();
                int y = module->getY();
                int width = module->getWidth();
                int height = module->getHeight();
                
                if (x >= 0 && y >= 0) {  // Only consider valid coordinates
                    minX = std::min(minX, x);
                    minY = std::min(minY, y);
                    maxX = std::max(maxX, x + width);
                    maxY = std::max(maxY, y + height);
                    validCoordinates = true;
                }
            }
        }
        
        if (validCoordinates) {
            totalArea = (maxX - minX) * (maxY - minY);
            std::cout << "Estimated area from module positions: " << totalArea << std::endl;
        } else {
            // If all else fails, return the last known area
            std::cout << "Using last known area: " << totalArea << std::endl;
        }
    }
}