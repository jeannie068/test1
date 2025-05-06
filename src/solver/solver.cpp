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
      totalArea(0),
      timeoutManager(nullptr) {
}

PlacementSolver::~PlacementSolver() {
    // Clean up any resources if needed
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
        if (pair.second) {
            hbTree->addModule(pair.second);
        } else {
            cerr << "Warning: Null module skipped: " << pair.first << endl;
        }
    }
    
    for (const auto& group : symmetryGroups) {
        if (group) {
            hbTree->addSymmetryGroup(group);
        } else {
            cerr << "Warning: Null symmetry group skipped" << endl;
        }
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
    
    // Attempt to construct an improved initial HB*-tree
    try {
        hbTree->constructImprovedInitialTree();
        
        // Pack the tree to get initial coordinates
        hbTree->pack();
        
        // Output initial area
        cout << "Initial area: " << hbTree->getArea() << endl;
    } 
    catch (const exception& e) {
        cerr << "Error while creating initial solution: " << e.what() << endl;
        
        // Fallback to basic construction method
        cerr << "Attempting fallback to basic construction method..." << endl;
        hbTree->constructInitialTree();
        hbTree->pack();
        
        cout << "Initial area (fallback method): " << hbTree->getArea() << endl;
    }
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
    
    // Create the simulated annealing solver
    SimulatedAnnealing sa(hbTree, 
                          initialTemperature,
                          finalTemperature,
                          coolingRate,
                          iterationsPerTemperature,
                          noImprovementLimit);
    
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
    
    // Always track the best solution we found
    int bestAreaFound = sa.getBestCost();
    
    // If we have a valid result from SA, use it
    if (result) {
        // Update the HB*-tree with the best solution from SA
        hbTree = result;
        
        // Ensure the solution is packed
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
        
        // Print the area found by SA and the actual area after packing
        cout << "Best area found by SA: " << bestAreaFound << endl;
        cout << "Actual area after final packing: " << totalArea << endl;
        
        // Check if the SA solution is valid
        if (totalArea <= 0 || totalArea > initialArea * 2) {
            cout << "SA solution appears invalid. Using initial solution." << endl;
            hbTree = initialSolution;
            hbTree->pack();
            totalArea = hbTree->getArea();
        }
        // Check if the initial solution was better than what SA found
        else if (totalArea > initialArea) {
            cout << "Initial solution was better than SA result. Using initial solution." << endl;
            hbTree = initialSolution;
            hbTree->pack();
            totalArea = initialArea;
        }
    } else {
        // If no result from SA, use the initial solution
        cout << "No valid solution from SA. Using initial solution." << endl;
        hbTree = initialSolution;
        hbTree->pack();
        totalArea = hbTree->getArea();
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
    if (!hbTree) return map<string, shared_ptr<Module>>();
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
        if (totalArea > currentArea && currentArea > 0) {
            cerr << "Warning: Final packing increased area from " << currentArea
                 << " to " << totalArea << endl;
            
            // We'll still use the updated placement for consistency
            cout << "Solution finalized - Area: " << totalArea << endl;
        } else {
            cout << "Solution finalized - Area: " << totalArea << endl;
        }
        
        // Perform a final check for overlaps
        validateFinalPlacement();
    }
    catch (const exception& e) {
        cerr << "Error finalizing solution: " << e.what() << endl;
        
        // Try a more cautious approach to get some valid area
        // This is a fallback mechanism for when we have a solution but packing fails
        calculateAreaFromModules();
    }
}

/**
 * Calculate area directly from module positions as a fallback
 */
void PlacementSolver::calculateAreaFromModules() {
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
    } else {
        // If all else fails, use the last known area if it's reasonable
        if (totalArea > 0) {
            cout << "Using last known area: " << totalArea << endl;
        } else {
            cerr << "Unable to determine a valid area" << endl;
            totalArea = 0;
        }
    }
}

/**
 * Validates and fixes the final placement if necessary
 */
void PlacementSolver::validateFinalPlacement() {
    // Check for overlapping modules and try to fix them if found
    bool hasOverlap = false;
    
    for (auto it1 = modules.begin(); it1 != modules.end(); ++it1) {
        const auto& module1 = it1->second;
        if (!module1) continue;
        
        for (auto it2 = next(it1); it2 != modules.end(); ++it2) {
            const auto& module2 = it2->second;
            if (!module2) continue;
            
            // Check for overlap
            if (module1->getX() < module2->getX() + module2->getWidth() &&
                module1->getX() + module1->getWidth() > module2->getX() &&
                module1->getY() < module2->getY() + module2->getHeight() &&
                module1->getY() + module1->getHeight() > module2->getY()) {
                
                cerr << "Overlap detected in final placement between " << it1->first 
                     << " and " << it2->first << endl;
                hasOverlap = true;
                
                // Emergency fix: move the second module below the first one
                module2->setPosition(module2->getX(), module1->getY() + module1->getHeight());
                cerr << "Emergency fix: moved " << it2->first << " below " << it1->first << endl;
            }
        }
    }
    
    if (hasOverlap) {
        cerr << "Fixed overlaps in final placement - recalculating area" << endl;
        calculateAreaFromModules();
    }
}