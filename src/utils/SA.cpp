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
    currentSolution->pack();
    
    // Calculate initial cost
    currentCost = calculateCost(currentSolution);
    
    // Initialize best solution
    bestSolution = currentSolution->clone();
    bestCost = currentCost;
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
    // Area cost
    int areaCost = solution->getArea();
    
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
                              return group->getName() == symmetryGroupName;
                          });
    
    if (it == symmetryGroups.end() || (*it)->getSymmetryPairs().empty()) {
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
    const auto& modules = currentSolution->getModules();
    if (modules.empty()) return "";
    
    // Convert map to vector for random selection
    vector<string> moduleNames;
    for (const auto& pair : modules) {
        moduleNames.push_back(pair.first);
    }
    
    uniform_int_distribution<int> dist(0, moduleNames.size() - 1);
    return moduleNames[dist(rng)];
}

/**
 * Select a random symmetry group
 */
string SimulatedAnnealing::selectRandomSymmetryGroup() const {
    const auto& symmetryGroups = currentSolution->getSymmetryGroups();
    if (symmetryGroups.empty()) return "";
    
    uniform_int_distribution<int> dist(0, symmetryGroups.size() - 1);
    return symmetryGroups[dist(rng)]->getName();
}

/**
 * Select a random node (module or symmetry group)
 */
string SimulatedAnnealing::selectRandomNode() const {
    // Decide whether to select a module or a symmetry group
    const auto& modules = currentSolution->getModules();
    const auto& symmetryGroups = currentSolution->getSymmetryGroups();
    
    int totalNodes = modules.size() + symmetryGroups.size();
    if (totalNodes == 0) return "";
    
    uniform_int_distribution<int> dist(0, totalNodes - 1);
    int index = dist(rng);
    
    if (index < static_cast<int>(modules.size())) {
        // Select a module
        auto it = modules.begin();
        advance(it, index);
        return it->first;
    } else {
        // Select a symmetry group
        index -= modules.size();
        return symmetryGroups[index]->getName();
    }
}

/**
 * Decides whether to accept a move based on the cost difference and temperature
 */
bool SimulatedAnnealing::acceptMove(int costDifference, double temperature) const {
    // Always accept moves that improve the solution
    if (costDifference <= 0) {
        return true;
    }
    
    // For moves that worsen the solution, accept with a probability based on temperature
    double probability = exp(-costDifference / temperature);
    return uniformDist(rng) < probability;
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
        
        // Main annealing loop
        while (temperature > finalTemperature && noImprovementCount < noImprovementLimit) {
            // Check for timeout at the beginning of each temperature
            if (checkTimeout()) {
                cout << "Timeout detected at temperature " << temperature 
                          << ". Returning best solution so far." << endl;
                return bestSolution;
            }
            
            // Perform iterations at current temperature
            for (int i = 0; i < iterationsPerTemperature; ++i) {
                // Check for timeout frequently during iterations
                if (i % 10 == 0 && checkTimeout()) {
                    cout << "Timeout detected during iteration " << i 
                              << " at temperature " << temperature << "." << endl;
                    return bestSolution;
                }
                
                try {
                    // Create a copy of the current solution
                    auto tempSolution = currentSolution->clone();
                    
                    // Perturb the temporary solution
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
                        }
                        
                        // Update best solution if improved
                        if (newCost < bestCost) {
                            bestSolution = currentSolution->clone();
                            bestCost = newCost;
                            noImprovementCount = 0;
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
                    if (totalIterations % 100 == 0) {
                        adaptivePerturbation.updateProbabilities();
                    }
                } 
                catch (const runtime_error& e) {
                    if (string(e.what()).find("Timeout") != string::npos) {
                        // Timeout detected during perturbation or packing
                        return bestSolution;
                    } else {
                        // Re-throw other runtime errors
                        throw;
                    }
                }
            }
            
            // Check for timeout after a temperature cycle
            if (checkTimeout()) {
                cout << "Timeout detected after completing temperature " << temperature << "." << endl;
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
        }
        
        // Return the best solution found
        return bestSolution;
    }
    catch (const exception& e) {
        cout << "Exception in SA::run(): " << e.what() << endl;
        cout << "Returning best solution found so far." << endl;
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