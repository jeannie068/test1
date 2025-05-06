#include "../utils/SA.hpp"
#include <iostream>
#include <algorithm>
#include <ctime>
#include <limits>
#include <cmath>
#include <chrono>
using namespace std;

void SimulatedAnnealing::setTimeoutManager(shared_ptr<TimeoutManager> manager) {
    timeoutManager = manager;
}

bool SimulatedAnnealing::checkTimeout() const {
    if (timeoutManager && timeoutManager->hasTimedOut()) {
        return true;
    }
    return false;
}

/**
 * Constructor
 */
SimulatedAnnealing::SimulatedAnnealing(shared_ptr<HBStarTree> initialSolution,
                                     double initialTemp,
                                     double finalTemp,
                                     double coolRate,
                                     int iterations,
                                     int noImprovementLimit)
    : currentSolution(initialSolution),
      bestSolution(nullptr),
      initialTemperature(initialTemp),
      finalTemperature(finalTemp),
      coolingRate(coolRate),
      iterationsPerTemperature(iterations),
      noImprovementLimit(noImprovementLimit),
      uniformDist(0.0, 1.0),
      probRotate(0.3),
      probMove(0.3),
      probSwap(0.3),
      probChangeRepresentative(0.05),
      probConvertSymmetryType(0.05),
      totalIterations(0),
      acceptedMoves(0),
      rejectedMoves(0),
      noImprovementCount(0),
      areaWeight(1.0),
      wirelengthWeight(0.0),
      startTime(chrono::steady_clock::now()),
      timeoutSeconds(0),
      adaptivePerturbation(0.3, 0.3, 0.3, 0.05, 0.05),
      lastOperation("") {
    
    // Initialize random number generator with current time
    rng.seed(static_cast<unsigned int>(time(nullptr)));
    
    // Pack initial solution to get valid coordinates and area
    if (currentSolution) {
        currentSolution->pack();
        
        // Create a deep copy for the best solution
        bestSolution = currentSolution->clone();
        
        // Calculate initial cost
        currentCost = calculateCost(currentSolution);
        bestCost = currentCost;
    } else {
        cerr << "Warning: Null initial solution provided to SA" << endl;
        currentCost = numeric_limits<int>::max();
        bestCost = numeric_limits<int>::max();
    }
}

/**
 * Sets the perturbation probabilities
 */
void SimulatedAnnealing::setPerturbationProbabilities(double rotate, double move, double swap, 
                                                    double changeRep, double convertSym) {
    // Normalize probabilities to sum to 1.0
    double sum = rotate + move + swap + changeRep + convertSym;
    if (sum <= 0.0) {
        // Default values if all probabilities are zero or negative
        probRotate = 0.3;
        probMove = 0.3;
        probSwap = 0.3;
        probChangeRepresentative = 0.05;
        probConvertSymmetryType = 0.05;
        return;
    }
    
    probRotate = rotate / sum;
    probMove = move / sum;
    probSwap = swap / sum;
    probChangeRepresentative = changeRep / sum;
    probConvertSymmetryType = convertSym / sum;
    
    // Update adaptive perturbation with same values
    adaptivePerturbation = AdaptivePerturbation(probRotate, probMove, probSwap, 
                                                probChangeRepresentative, probConvertSymmetryType);
}

/**
 * Sets the cost function weights
 */
void SimulatedAnnealing::setCostWeights(double area, double wirelength) {
    areaWeight = area;
    wirelengthWeight = wirelength;
}

/**
 * Calculates the cost of a solution
 */
int SimulatedAnnealing::calculateCost(const shared_ptr<HBStarTree>& solution) const {
    if (!solution) return numeric_limits<int>::max();
    
    // Area cost - with penalty for invalid area
    int areaCost = solution->getArea();
    if (areaCost <= 0) {
        cerr << "Warning: Invalid area in cost calculation" << endl;
        return numeric_limits<int>::max();
    }
    
    // Wirelength cost
    int wirelengthCost = solution->getWireLength();
    
    // Weighted sum
    return static_cast<int>(areaWeight * areaCost + wirelengthWeight * wirelengthCost);
}

/**
 * Performs a random perturbation on the current solution
 */
bool SimulatedAnnealing::perturb() {
    // Check for timeout
    if (checkTimeout()) {
        throw runtime_error("Timeout during perturbation");
    }
    
    // Choose perturbation type based on adaptive probabilities
    double randVal = uniformDist(rng);
    bool result = false;
    const int maxAttempts = 3;  // Reduced from 5 to 3 for speed
    
    for (int attempt = 0; attempt < maxAttempts; attempt++) {
        if (randVal < adaptivePerturbation.getRotateProbability()) {
            lastOperation = "rotate";
            adaptivePerturbation.recordAttempt(lastOperation);
            result = perturbRotate();
        } else if (randVal < adaptivePerturbation.getRotateProbability() + 
                          adaptivePerturbation.getMoveProbability()) {
            lastOperation = "move";
            adaptivePerturbation.recordAttempt(lastOperation);
            result = perturbMove();
        } else if (randVal < adaptivePerturbation.getRotateProbability() + 
                          adaptivePerturbation.getMoveProbability() + 
                          adaptivePerturbation.getSwapProbability()) {
            lastOperation = "swap";
            adaptivePerturbation.recordAttempt(lastOperation);
            result = perturbSwap();
        } else if (randVal < adaptivePerturbation.getRotateProbability() + 
                          adaptivePerturbation.getMoveProbability() + 
                          adaptivePerturbation.getSwapProbability() + 
                          adaptivePerturbation.getChangeRepProbability()) {
            lastOperation = "changeRep";
            adaptivePerturbation.recordAttempt(lastOperation);
            result = perturbChangeRepresentative();
        } else {
            lastOperation = "convertSym";
            adaptivePerturbation.recordAttempt(lastOperation);
            result = perturbConvertSymmetryType();
        }
        
        if (result) break;  // If perturbation succeeded, stop trying
        
        // Try a different operation type for the next attempt
        randVal = uniformDist(rng);
    }
    
    return result;
}

/**
 * Rotates a random module
 */
bool SimulatedAnnealing::perturbRotate() {
    string moduleName = selectRandomModule();
    if (moduleName.empty()) return false;
    
    return currentSolution->rotateModule(moduleName);
}

/**
 * Moves a random node to a new position
 */
bool SimulatedAnnealing::perturbMove() {
    string nodeName = selectRandomNode();
    string newParentName = selectRandomNode();
    
    if (nodeName.empty() || newParentName.empty() || nodeName == newParentName) {
        return false;
    }
    
    // Randomly decide if the node should be a left or right child
    bool asLeftChild = (uniformDist(rng) < 0.5);
    
    return currentSolution->moveNode(nodeName, newParentName, asLeftChild);
}

/**
 * Swaps two random nodes
 */
bool SimulatedAnnealing::perturbSwap() {
    string nodeName1 = selectRandomNode();
    string nodeName2 = selectRandomNode();
    
    if (nodeName1.empty() || nodeName2.empty() || nodeName1 == nodeName2) {
        return false;
    }
    
    return currentSolution->swapNodes(nodeName1, nodeName2);
}

/**
 * Changes the representative of a symmetry pair in a random symmetry group
 */
bool SimulatedAnnealing::perturbChangeRepresentative() {
    string symmetryGroupName = selectRandomSymmetryGroup();
    if (symmetryGroupName.empty()) return false;
    
    // Get a random module from the symmetry group
    const auto& symmetryGroups = currentSolution->getSymmetryGroups();
    auto it = find_if(symmetryGroups.begin(), symmetryGroups.end(),
                      [&symmetryGroupName](const shared_ptr<SymmetryGroup>& group) {
                          return group && group->getName() == symmetryGroupName;
                      });
    
    if (it == symmetryGroups.end() || !(*it) || (*it)->getSymmetryPairs().empty()) {
        return false;
    }
    
    // Choose a random symmetry pair
    const auto& pairs = (*it)->getSymmetryPairs();
    uniform_int_distribution<int> pairDist(0, pairs.size() - 1);
    const auto& pair = pairs[pairDist(rng)];
    
    // Randomly choose one of the modules in the pair
    string moduleName = (uniformDist(rng) < 0.5) ? pair.first : pair.second;
    
    return currentSolution->changeRepresentative(symmetryGroupName, moduleName);
}

/**
 * Converts the symmetry type of a random symmetry group
 */
bool SimulatedAnnealing::perturbConvertSymmetryType() {
    string symmetryGroupName = selectRandomSymmetryGroup();
    if (symmetryGroupName.empty()) return false;
    
    return currentSolution->convertSymmetryType(symmetryGroupName);
}

/**
 * Select a random module
 */
string SimulatedAnnealing::selectRandomModule() const {
    if (!currentSolution) return "";
    
    const auto& modules = currentSolution->getModules();
    if (modules.empty()) return "";
    
    // Convert map to vector for random selection
    vector<string> moduleNames;
    moduleNames.reserve(modules.size());
    
    for (const auto& pair : modules) {
        if (pair.second) {
            moduleNames.push_back(pair.first);
        }
    }
    
    if (moduleNames.empty()) return "";
    
    uniform_int_distribution<int> dist(0, moduleNames.size() - 1);
    return moduleNames[dist(rng)];
}

/**
 * Select a random symmetry group
 */
string SimulatedAnnealing::selectRandomSymmetryGroup() const {
    if (!currentSolution) return "";
    
    const auto& symmetryGroups = currentSolution->getSymmetryGroups();
    if (symmetryGroups.empty()) return "";
    
    // Filter valid symmetry groups
    vector<string> validGroups;
    for (const auto& group : symmetryGroups) {
        if (group) {
            validGroups.push_back(group->getName());
        }
    }
    
    if (validGroups.empty()) return "";
    
    uniform_int_distribution<int> dist(0, validGroups.size() - 1);
    return validGroups[dist(rng)];
}

/**
 * Select a random node (module or symmetry group)
 */
string SimulatedAnnealing::selectRandomNode() const {
    if (!currentSolution) return "";
    
    // Decide whether to select a module or a symmetry group
    const auto& modules = currentSolution->getModules();
    const auto& symmetryGroups = currentSolution->getSymmetryGroups();
    
    // Build vectors of valid names
    vector<string> validModules;
    for (const auto& pair : modules) {
        if (pair.second) {
            validModules.push_back(pair.first);
        }
    }
    
    vector<string> validGroups;
    for (const auto& group : symmetryGroups) {
        if (group) {
            validGroups.push_back(group->getName());
        }
    }
    
    int totalNodes = validModules.size() + validGroups.size();
    if (totalNodes == 0) return "";
    
    uniform_int_distribution<int> dist(0, totalNodes - 1);
    int index = dist(rng);
    
    if (index < static_cast<int>(validModules.size())) {
        // Select a module
        return validModules[index];
    } else {
        // Select a symmetry group
        index -= validModules.size();
        if (index < static_cast<int>(validGroups.size())) {
            return validGroups[index];
        }
    }
    
    return "";
}

/**
 * Decides whether to accept a move based on the cost difference and temperature
 */
bool SimulatedAnnealing::acceptMove(int costDifference, double temperature) const {
    // Always accept moves that improve the solution
    if (costDifference <= 0) {
        return true;
    }
    
    // For moves that worsen the solution, accept with a probability
    double probability = exp(-costDifference / temperature);
    return uniformDist(rng) < probability;
}

/**
 * Validates the best solution for overlaps
 */
void SimulatedAnnealing::validateBestSolution() {
    if (!bestSolution) return;
    
    // Get all modules
    const auto& modules = bestSolution->getModules();
    
    // Check for overlaps
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
                
                cerr << "Overlap detected in best solution between " << it1->first 
                     << " and " << it2->first << endl;
                hasOverlap = true;
                
                // Try to fix by moving module2 below module1
                module2->setPosition(module2->getX(), module1->getY() + module1->getHeight());
            }
        }
    }
    
    if (hasOverlap) {
        cerr << "Fixed overlaps in best solution - repacking" << endl;
        bestSolution->pack();
    }
}

/**
 * Runs the SA
 */
shared_ptr<HBStarTree> SimulatedAnnealing::run() {
    try {
        double temperature = initialTemperature;
        
        // Reset statistics
        totalIterations = 0;
        acceptedMoves = 0;
        rejectedMoves = 0;
        noImprovementCount = 0;
        
        // Make sure initial solution is valid
        if (currentSolution) {
            currentSolution->pack();
            currentCost = calculateCost(currentSolution);
            
            // Clone best solution
            if (!bestSolution) {
                bestSolution = currentSolution->clone();
                bestCost = currentCost;
            }
        } else {
            cerr << "Error: No valid initial solution for SA" << endl;
            return bestSolution;
        }
        
        // Calculate total iterations before we'd reach final temperature
        int totalTemperatureSteps = ceil(log(finalTemperature/initialTemperature) / log(coolingRate));
        int estimatedTotalIterations = totalTemperatureSteps * iterationsPerTemperature;
        
        cout << "Starting SA with estimated " << estimatedTotalIterations 
             << " iterations over " << totalTemperatureSteps << " temperature steps" << endl;
        
        // Main annealing loop
        while (temperature > finalTemperature && noImprovementCount < noImprovementLimit) {
            // Check for timeout at the beginning of each temperature
            if (checkTimeout()) {
                cout << "Timeout detected at temperature " << temperature 
                     << ". Returning best solution so far." << endl;
                validateBestSolution();
                return bestSolution;
            }
            
            // Perform iterations at current temperature
            for (int i = 0; i < iterationsPerTemperature; ++i) {
                // Check for timeout every few iterations
                if (i % 5 == 0 && checkTimeout()) {
                    cout << "Timeout detected during iteration " << i 
                         << " at temperature " << temperature << "." << endl;
                    validateBestSolution();
                    return bestSolution;
                }
                
                try {
                    // Create a copy of the current solution
                    auto tempSolution = currentSolution->clone();
                    
                    // Perturb the solution
                    bool perturbSuccess = perturb();
                    if (!perturbSuccess) {
                        continue;  // Skip this iteration if perturbation failed
                    }
                    
                    // Pack the solution to get updated coordinates and area
                    currentSolution->pack();
                    
                    // Calculate new cost
                    int newCost = calculateCost(currentSolution);
                    
                    // Decide whether to accept the move
                    int costDifference = newCost - currentCost;
                    if (acceptMove(costDifference, temperature)) {
                        // Accept the move
                        currentCost = newCost;
                        acceptedMoves++;
                        
                        // Record success and improvement for adaptive perturbation
                        if (costDifference < 0) {
                            adaptivePerturbation.recordSuccess(lastOperation, -costDifference);
                            
                            // Update best solution if improved
                            if (newCost < bestCost) {
                                // Create a deep copy of the current solution
                                bestSolution = currentSolution->clone();
                                bestCost = newCost;
                                
                                cout << "New best solution found: " << bestCost << endl;
                                noImprovementCount = 0;
                            } else {
                                noImprovementCount++;
                            }
                        } else {
                            noImprovementCount++;
                        }
                    } else {
                        // Reject the move
                        currentSolution = tempSolution;
                        rejectedMoves++;
                        noImprovementCount++;
                    }
                    
                    totalIterations++;
                    
                    // Update perturbation probabilities periodically
                    if (totalIterations % 50 == 0) {
                        adaptivePerturbation.updateProbabilities();
                    }
                } 
                catch (const runtime_error& e) {
                    if (string(e.what()).find("Timeout") != string::npos) {
                        // Timeout detected during perturbation or packing
                        validateBestSolution();
                        return bestSolution;
                    } else {
                        // Other runtime errors - log and continue
                        cerr << "Error during SA iteration: " << e.what() << endl;
                        continue;
                    }
                }
            }
            
            // Check for timeout after a temperature cycle
            if (checkTimeout()) {
                cout << "Timeout detected after completing temperature " << temperature << "." << endl;
                validateBestSolution();
                return bestSolution;
            }
            
            // Cool down
            temperature *= coolingRate;
            
            // Print progress including adaptive perturbation statistics every 5 temperature steps
            static int tempSteps = 0;
            tempSteps++;
            
            if (tempSteps % 5 == 0) {
                cout << "Temperature: " << temperature 
                     << ", Best cost: " << bestCost 
                     << ", Current cost: " << currentCost 
                     << ", No improvement: " << noImprovementCount 
                     << endl;
                
                // Print adaptive perturbation statistics
                adaptivePerturbation.printStats();
            } else {
                cout << "Temperature: " << temperature 
                     << ", Best cost: " << bestCost 
                     << ", Current cost: " << currentCost 
                     << ", No improvement: " << noImprovementCount 
                     << endl;
            }
            
            // Check if we need to accelerate cooling
            if (noImprovementCount > noImprovementLimit / 2) {
                // No improvement for a while, cool faster
                temperature *= coolingRate; // Double the cooling effect
                cout << "Accelerating cooling - no improvement for " << noImprovementCount << " iterations" << endl;
            }
        }
        
        cout << "SA completed normally. Best cost: " << bestCost << endl;
        
        // Validate the best solution before returning
        validateBestSolution();
        
        // Return the best solution found
        return bestSolution;
    }
    catch (const exception& e) {
        cout << "Exception in SA::run(): " << e.what() << endl;
        cout << "Returning best solution found so far." << endl;
        
        // Try to validate best solution even after exception
        try {
            validateBestSolution();
        } catch (...) {
            cerr << "Error during best solution validation after exception" << endl;
        }
        
        return bestSolution;
    }
}

shared_ptr<HBStarTree> SimulatedAnnealing::getBestSolution() const {
    return bestSolution;
}

int SimulatedAnnealing::getBestCost() const {
    return bestCost;
}

/**
 * Gets statistics about the annealing process
 */
map<string, int> SimulatedAnnealing::getStatistics() const {
    map<string, int> stats;
    stats["totalIterations"] = totalIterations;
    stats["acceptedMoves"] = acceptedMoves;
    stats["rejectedMoves"] = rejectedMoves;
    stats["noImprovementCount"] = noImprovementCount;
    return stats;
}

/**
 * Sets the random seed for reproducible results
 */
void SimulatedAnnealing::setSeed(unsigned int seed) {
    rng.seed(seed);
}