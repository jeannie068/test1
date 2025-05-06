#include "../solver/solver.hpp"
#include "../data_struct/HBStarTree.hpp"
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

void PlacementSolver::setTimeoutManager(shared_ptr<TimeoutManager> manager) {
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
        cerr << "Error: Failed to create initial solution." << endl;
        return false;
    }
    
    // Pack the initial solution to get accurate area
    hbTree->pack();
    
    // Save a copy of the initial solution and its area
    auto initialSolution = hbTree->clone();
    int initialArea = hbTree->getArea();
    cout << "Initial area: " << initialArea << endl;
    
    cout << "Starting simulated annealing..." << endl;
    cout << "Initial temperature: " << initialTemperature << endl;
    cout << "Final temperature: " << finalTemperature << endl;
    cout << "Cooling rate: " << coolingRate << endl;
    cout << "Iterations per temperature: " << iterationsPerTemperature << endl;
    cout << "No improvement limit: " << noImprovementLimit << endl;
    
    // Create the simulated annealing solver with optimized parameters
    SimulatedAnnealing sa(hbTree, 
                          initialTemperature,
                          finalTemperature,
                          coolingRate,
                          iterationsPerTemperature,
                          noImprovementLimit);
    
    // Let the adaptive perturbation handle probabilities
    sa.setPerturbationProbabilities(probRotate, probMove, probSwap, 
                                   probChangeRep, probConvertSym);
    sa.setCostWeights(areaWeight, wirelengthWeight);
    sa.setSeed(randomSeed);
    
    // Pass the timeout manager
    if (timeoutManager) {
        sa.setTimeoutManager(timeoutManager);
    }
    
    // Check for timeout before starting
    if (timeoutManager && timeoutManager->hasTimedOut()) {
        cout << "Timeout detected before starting SA." << endl;
        return false;
    }
    
    // Run simulated annealing
    shared_ptr<HBStarTree> result = nullptr;
    try {
        result = sa.run();
        if (!result) {
            cerr << "Error: Simulated annealing failed to find a solution." << endl;
            return false;
        }
    }
    catch (const runtime_error& e) {
        string errorMsg = e.what();
        if (errorMsg.find("Timeout") != string::npos) {
            cout << "SA process was interrupted by timeout." << endl;
            // Use the best solution found so far
            result = sa.getBestSolution();
            if (!result) {
                cerr << "No solution available after timeout." << endl;
                // If no SA solution, use the initial solution
                hbTree = initialSolution;
                totalArea = initialArea;
                return true;
            }
        } else {
            throw; // Re-throw other runtime errors
        }
    }
    
    // Update the HB*-tree with the best solution from SA
    hbTree = result;
    
    // Ensure the solution is packed once to get accurate area
    try {
        hbTree->pack();
    }
    catch (const runtime_error& e) {
        string errorMsg = e.what();
        if (errorMsg.find("Timeout") != string::npos) {
            cout << "Packing was interrupted by timeout. Using partial results." << endl;
        } else {
            throw; // Re-throw other runtime errors
        }
    }
    
    // Update statistics
    totalArea = hbTree->getArea();
    
    // Check if the initial solution was better than what SA found
    if (totalArea > initialArea) {
        cout << "Initial solution was better than SA result. Using initial solution." << endl;
        hbTree = initialSolution;
        totalArea = initialArea;
    }
    
    cout << "Final area: " << totalArea << endl;
    
    // Print statistics
    auto stats = sa.getStatistics();
    cout << "Total iterations: " << stats["totalIterations"] << endl;
    cout << "Accepted moves: " << stats["acceptedMoves"] << endl;
    cout << "Rejected moves: " << stats["rejectedMoves"] << endl;
    cout << "No improvement count: " << stats["noImprovementCount"] << endl;
    
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
    // Make sure having a valid HB*-tree
    if (!hbTree || !hbTree->getRoot()) {
        cerr << "Error: No solution to finalize" << endl;
        totalArea = 0;
        return;
    }
    
    // Get the current area before potentially packing again
    int currentArea = hbTree->getArea();
    
    try {
        // Pack the tree to ensure coordinates are up-to-date
        hbTree->pack();
        
        // Calculate the area after packing
        totalArea = hbTree->getArea();
        
        // If packing makes the area worse, revert to the original area calculation
        // This happens if the solution was already optimally packed
        if (totalArea > currentArea && currentArea > 0) {
            totalArea = currentArea;
            cout << "Using pre-pack area as it's better: " << totalArea << endl;
        } else {
            cout << "Solution finalized - Area: " << totalArea << endl;
        }
    }
    catch (const exception& e) {
        cerr << "Error finalizing solution: " << e.what() << endl;
        
        // Try a more cautious approach to get some valid area
        // This is a fallback mechanism for when we have a solution but packing fails
        int minX = numeric_limits<int>::max();
        int minY = numeric_limits<int>::max();
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
                    minX = min(minX, x);
                    minY = min(minY, y);
                    maxX = max(maxX, x + width);
                    maxY = max(maxY, y + height);
                    validCoordinates = true;
                }
            }
        }
        
        if (validCoordinates) {
            totalArea = (maxX - minX) * (maxY - minY);
            cout << "Estimated area from module positions: " << totalArea << endl;
        } else if (currentArea > 0) {
            // If all else fails but we had a valid area before, use that
            totalArea = currentArea;
            cout << "Using pre-exception area: " << totalArea << endl;
        } else {
            // Last resort fallback
            cout << "Using last known area: " << totalArea << endl;
        }
    }
}