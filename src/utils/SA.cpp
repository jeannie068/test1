#include "../utils/SA.hpp"
#include <iostream>
#include <algorithm>
#include <ctime>
#include <limits>
#include <cmath>
#include <chrono>
using namespace std;

// MovePool implementation
MovePool::MovePool() {
    // Preallocate the first block
    allocateBlock();
    cout << "MovePool initialized with block size: " << BLOCK_SIZE << endl;
}

MovePool::~MovePool() {
    // Clean up all allocated blocks
    for (auto& block : blocks) {
        for (auto* move : block) {
            delete move;
        }
    }
}

void MovePool::allocateBlock() {
    blocks.emplace_back();
    auto& newBlock = blocks.back();
    newBlock.reserve(BLOCK_SIZE);
    
    // Create BLOCK_SIZE new Move objects
    for (size_t i = 0; i < BLOCK_SIZE; ++i) {
        Move* move = new Move("none");
        newBlock.push_back(move);
        freeList.push_back(move);
    }
    
    allocations++;
    if (allocations % 5 == 0) {
        cout << "WARNING: MovePool created " << allocations 
             << " blocks (" << allocations * BLOCK_SIZE << " moves) - possible memory issue" << endl;
    }
}

// Update the MovePool::createMove function to initialize the new fields
Move* MovePool::createMove(const std::string& type, 
                          const std::string& param1, 
                          const std::string& param2, 
                          bool boolParam) {
    // If no free moves, allocate a new block
    if (freeList.empty()) {
        allocateBlock();
    }
    
    // Get a move from the free list
    Move* move = freeList.back();
    freeList.pop_back();
    
    // Initialize the move (as implemented earlier)
    move->operationType = type;
    move->param1 = param1;
    move->param2 = param2;
    move->boolParam = boolParam;
    move->originalParent = "";
    move->wasLeftChild = false;
    move->originalRepresentative = "";
    
    return move;
}

void MovePool::releaseMove(Move* move) {
    // Reset the move
    move->reset(); // Use the reset method implemented earlier
    
    // Return to the free list
    freeList.push_back(move);
}

// Move implementation
Move::Move(const std::string& type, 
           const std::string& p1, 
           const std::string& p2, 
           bool bp) 
    : operationType(type), param1(p1), param2(p2), boolParam(bp) {
}

void SimulatedAnnealing::applyMove(Move* move) {
    if (!move) return;
    
    const string& operation = move->getType();
    
    if (operation == "rotate") {
        const string& moduleName = move->getParam1();
        currentSolution->rotateModule(moduleName);
    }
    else if (operation == "move") {
        const string& nodeName = move->getParam1();
        const string& newParentName = move->getParam2();
        bool asLeftChild = move->getBoolParam();
        
        // Store original state for undoing
        auto node = currentSolution->findNode(nodeName);
        if (node) {
            auto parent = node->getParent();
            if (parent) {
                move->originalParent = parent->getModuleName();
                move->wasLeftChild = node->isLeftChild();
            }
        }
        
        currentSolution->moveNode(nodeName, newParentName, asLeftChild);
    }
    else if (operation == "swap") {
        const string& nodeName1 = move->getParam1();
        const string& nodeName2 = move->getParam2();
        currentSolution->swapNodes(nodeName1, nodeName2);
    }
    else if (operation == "changeRep") {
        const string& symmetryGroupName = move->getParam1();
        const string& moduleName = move->getParam2();
        
        // Store the original representative for undoing
        // This is simplified - in a real implementation, you would
        // need to get the actual current representative from the group
        move->originalRepresentative = moduleName;
        
        currentSolution->changeRepresentative(symmetryGroupName, moduleName);
    }
    else if (operation == "convertSym") {
        const string& symmetryGroupName = move->getParam1();
        
        // Find the symmetry group to store its original type
        const auto& symmetryGroups = currentSolution->getSymmetryGroups();
        for (const auto& group : symmetryGroups) {
            if (group && group->getName() == symmetryGroupName) {
                move->originalSymType = group->getType();
                break;
            }
        }
        
        currentSolution->convertSymmetryType(symmetryGroupName);
    }
}

void SimulatedAnnealing::undoMove(Move* move) {
    if (!move) return;
    
    const string& operation = move->getType();
    
    if (operation == "rotate") {
        // Rotating again undoes the rotation
        const string& moduleName = move->getParam1();
        currentSolution->rotateModule(moduleName);
    }
    else if (operation == "move") {
        // Move back to original parent
        if (!move->originalParent.empty()) {
            const string& nodeName = move->getParam1();
            currentSolution->moveNode(nodeName, move->originalParent, move->wasLeftChild);
        }
        else {
            // If we don't have original parent info, fallback to using best solution
            cout << "Warning: Missing original parent info for move undo - using fallback method" << endl;
            auto originalSolution = bestSolution->clone();
            currentSolution = originalSolution;
        }
    }
    else if (operation == "swap") {
        // Swapping again undoes the swap
        const string& nodeName1 = move->getParam1();
        const string& nodeName2 = move->getParam2();
        currentSolution->swapNodes(nodeName1, nodeName2);
    }
    else if (operation == "changeRep") {
        // Change back to original representative
        if (!move->originalRepresentative.empty()) {
            const string& symmetryGroupName = move->getParam1();
            currentSolution->changeRepresentative(symmetryGroupName, move->originalRepresentative);
        }
        else {
            // Fallback
            cout << "Warning: Missing original representative info for changeRep undo - using fallback method" << endl;
            auto originalSolution = bestSolution->clone();
            currentSolution = originalSolution;
        }
    }
    else if (operation == "convertSym") {
        // Just convert again to undo
        const string& symmetryGroupName = move->getParam1();
        currentSolution->convertSymmetryType(symmetryGroupName);
    }
}

// SimulatedAnnealing implementation
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
      movesPerTemperature(iterations),
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
    
    // Initialize temperature based on average cost delta
    initializeTemperature();
}

void SimulatedAnnealing::setTimeoutManager(shared_ptr<TimeoutManager> manager) {
    timeoutManager = manager;
}

bool SimulatedAnnealing::checkTimeout() const {
    if (timeoutManager && timeoutManager->hasTimedOut()) {
        return true;
    }
    return false;
}

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

void SimulatedAnnealing::setCostWeights(double area, double wirelength) {
    areaWeight = area;
    wirelengthWeight = wirelength;
}

void SimulatedAnnealing::setSeed(unsigned int seed) {
    rng.seed(seed);
}

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

Move* SimulatedAnnealing::generateMove() {
    // Try multiple times to generate a valid move
    for (int attempt = 0; attempt < 5; attempt++) {
        // Choose perturbation type based on probabilities
        double randVal = uniformDist(rng);
        
        if (randVal < probRotate) {
            // Rotate a random module
            string moduleName = selectRandomRepresentativeModule();
            if (moduleName.empty()) continue;
            
            lastOperation = "rotate";
            return movePool.createMove("rotate", moduleName);
        } 
        else if (randVal < probRotate + probMove) {
            // Move a node to a new position
            string nodeName = selectRandomNode();
            string newParentName = selectRandomNode();
            
            if (nodeName.empty() || newParentName.empty() || nodeName == newParentName) {
                continue;
            }
            
            // Randomly decide if the node should be a left or right child
            bool asLeftChild = (uniformDist(rng) < 0.5);
            
            lastOperation = "move";
            return movePool.createMove("move", nodeName, newParentName, asLeftChild);
        } 
        else if (randVal < probRotate + probMove + probSwap) {
            // Swap two nodes
            string nodeName1 = selectRandomNode();
            string nodeName2 = selectRandomNode();
            
            if (nodeName1.empty() || nodeName2.empty() || nodeName1 == nodeName2) {
                continue;
            }
            
            lastOperation = "swap";
            return movePool.createMove("swap", nodeName1, nodeName2);
        } 
        else if (randVal < probRotate + probMove + probSwap + probChangeRepresentative) {
            // Change the representative of a symmetry pair
            string symmetryGroupName = selectRandomSymmetryGroup();
            if (symmetryGroupName.empty()) continue;
            
            // Get a random module from the symmetry group
            const auto& symmetryGroups = currentSolution->getSymmetryGroups();
            auto it = find_if(symmetryGroups.begin(), symmetryGroups.end(),
                             [&symmetryGroupName](const shared_ptr<SymmetryGroup>& group) {
                                 return group && group->getName() == symmetryGroupName;
                             });
            
            if (it == symmetryGroups.end() || !(*it) || (*it)->getSymmetryPairs().empty()) {
                continue;
            }
            
            // Choose a random symmetry pair
            const auto& pairs = (*it)->getSymmetryPairs();
            uniform_int_distribution<int> pairDist(0, pairs.size() - 1);
            const auto& pair = pairs[pairDist(rng)];
            
            // Randomly choose one of the modules in the pair
            string moduleName = (uniformDist(rng) < 0.5) ? pair.first : pair.second;
            
            lastOperation = "changeRep";
            return movePool.createMove("changeRep", symmetryGroupName, moduleName);
        } 
        else {
            // Convert the symmetry type of a symmetry group
            string symmetryGroupName = selectRandomSymmetryGroup();
            if (symmetryGroupName.empty()) continue;
            
            lastOperation = "convertSym";
            return movePool.createMove("convertSym", symmetryGroupName);
        }
    }
    
    // Could not generate a valid move after multiple attempts
    cerr << "Warning: Failed to generate a valid move after 5 attempts" << endl;
    return nullptr;
}

string SimulatedAnnealing::selectRandomRepresentativeModule() const {
    if (!currentSolution) return "";
    
    // Collect valid representative modules
    vector<string> representativeModules;
    
    const auto& modules = currentSolution->getModules();
    const auto& symmetryGroups = currentSolution->getSymmetryGroups();
    
    for (const auto& pair : modules) {
        const string& moduleName = pair.first;
        bool isRepresentative = false;
        bool inAnyGroup = false;
        
        // Check if this module is in any symmetry group
        for (const auto& group : symmetryGroups) {
            if (!group) continue;
            
            bool isInThisGroup = false;
            
            // Check if module is in symmetry pairs
            for (const auto& symmPair : group->getSymmetryPairs()) {
                if (moduleName == symmPair.first || moduleName == symmPair.second) {
                    isInThisGroup = true;
                    inAnyGroup = true;
                    
                    // By convention, the second module in the pair is the representative
                    if (moduleName == symmPair.second) {
                        isRepresentative = true;
                    }
                    break;
                }
            }
            
            if (!isInThisGroup) {
                // Check if module is a self-symmetric module
                for (const auto& selfSym : group->getSelfSymmetric()) {
                    if (moduleName == selfSym) {
                        isInThisGroup = true;
                        inAnyGroup = true;
                        isRepresentative = true;  // Self-symmetric modules are always representatives
                        break;
                    }
                }
            }
            
            if (isRepresentative) break;  // No need to check other groups
        }
        
        // If not in any symmetry group, it is considered a representative
        if (!inAnyGroup) {
            isRepresentative = true;
        }
        
        if (isRepresentative) {
            representativeModules.push_back(moduleName);
        }
    }
    
    if (representativeModules.empty()) {
        cerr << "Warning: No representative modules found!" << endl;
        return "";
    }
    
    // Select a random representative module
    uniform_int_distribution<int> dist(0, representativeModules.size() - 1);
    return representativeModules[dist(rng)];
}

bool SimulatedAnnealing::acceptMove(int costDifference, double temperature) const {
    // Always accept moves that improve the solution
    if (costDifference <= 0) {
        return true;
    }
    
    // For moves that worsen the solution, accept with a probability
    double probability = exp(-costDifference / temperature);
    return uniformDist(rng) < probability;
}

// In SA.cpp
string SimulatedAnnealing::selectRandomModule() const {
    if (!currentSolution) return "";
    
    // Only select representative modules for operations
    vector<string> representativeModules;
    
    // Get all symmetry groups to check for representatives
    const auto& symmetryGroups = currentSolution->getSymmetryGroups();
    const auto& modules = currentSolution->getModules();
    
    for (const auto& pair : modules) {
        const string& moduleName = pair.first;
        bool isRepresentative = false;
        
        // Check if this module is a representative in any symmetry group
        for (const auto& group : symmetryGroups) {
            if (!group) continue;
            
            // Check if the module is a representative in this group
            for (const auto& symmPair : group->getSymmetryPairs()) {
                // For each symmetry pair, determine the representative
                string repName = (symmPair.first < symmPair.second) ? 
                                  symmPair.second : symmPair.first;
                
                if (moduleName == repName) {
                    isRepresentative = true;
                    break;
                }
            }
            
            // Also check self-symmetric modules
            for (const auto& selfSym : group->getSelfSymmetric()) {
                if (moduleName == selfSym) {
                    isRepresentative = true;
                    break;
                }
            }
            
            if (isRepresentative) break;
        }
        
        // For modules not in any symmetry group, they are also representatives
        if (!isRepresentative) {
            bool inAnyGroup = false;
            for (const auto& group : symmetryGroups) {
                if (!group) continue;
                
                if (group->isInGroup(moduleName)) {
                    inAnyGroup = true;
                    break;
                }
            }
            
            if (!inAnyGroup) {
                isRepresentative = true;
            }
        }
        
        // Add to the list if it's a representative
        if (isRepresentative) {
            representativeModules.push_back(moduleName);
        }
    }
    
    // If no representatives found, try a different approach or return empty
    if (representativeModules.empty()) {
        // Fallback: assume all non-symmetry modules are representatives
        for (const auto& pair : modules) {
            bool inAnyGroup = false;
            for (const auto& group : symmetryGroups) {
                if (!group) continue;
                
                if (group->isInGroup(pair.first)) {
                    inAnyGroup = true;
                    break;
                }
            }
            
            if (!inAnyGroup) {
                representativeModules.push_back(pair.first);
            }
        }
    }
    
    // If still no representatives, return empty
    if (representativeModules.empty()) return "";
    
    // Select a random representative module
    uniform_int_distribution<int> dist(0, representativeModules.size() - 1);
    return representativeModules[dist(rng)];
}

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

void SimulatedAnnealing::initializeTemperature() {
    // Measure average delta cost over random moves
    const int sampleSize = 500;
    double totalDelta = 0.0;
    int validSamples = 0;
    
    cout << "Sampling " << sampleSize << " random moves to initialize temperature..." << endl;
    
    for (int i = 0; i < sampleSize; ++i) {
        auto move = generateMove();
        if (!move) continue;
        
        int costBefore = calculateCost(currentSolution);
        applyMove(move);
        currentSolution->pack();
        int costAfter = calculateCost(currentSolution);
        
        // Undo the move
        undoMove(move);
        currentSolution->pack();
        
        // Release the move back to the pool
        movePool.releaseMove(move);
        
        // Accumulate the absolute difference, but check for valid values
        int deltaCost = abs(costAfter - costBefore);
        if (deltaCost > 0 && deltaCost < INT_MAX / 2) {
            totalDelta += deltaCost;
            validSamples++;
        }
    }
    
    // Calculate average delta, with safeguards
    double avgDelta = (validSamples > 0) ? totalDelta / validSamples : 1000.0;
    
    // Apply reasonable caps to the temperature
    const double MIN_TEMPERATURE = 100.0;
    const double MAX_TEMPERATURE = 10000.0;  // 1 million - much lower than 40 million
    
    // Calculate temperature according to formula, but with bounds
    double calculatedTemp = -avgDelta / log(0.8);
    initialTemperature = std::max(MIN_TEMPERATURE, std::min(calculatedTemp, MAX_TEMPERATURE));
    
    cout << "Calculated temperature from " << validSamples << " samples:" << endl;
    cout << "  Average cost delta: " << avgDelta << endl;
    cout << "  Raw calculated temperature: " << calculatedTemp << endl;
    cout << "  Capped initial temperature: " << initialTemperature << endl;
    
    // Make sure we have a valid best solution
    if (!bestSolution) {
        bestSolution = currentSolution->clone();
        bestCost = calculateCost(currentSolution);
    }
}

bool SimulatedAnnealing::processTemperature(double temperature) {
    bool improved = false;
    
    // Clear accepted move history for this temperature
    for (auto* move : acceptedMoveHistory) {
        movePool.releaseMove(move);
    }
    acceptedMoveHistory.clear();
    
    for (int i = 0; i < movesPerTemperature; ++i) {
        // Check for timeout periodically
        if (i % 100 == 0 && checkTimeout()) {
            throw runtime_error("Timeout occurred during temperature processing");
        }
        
        // Generate a random move
        auto move = generateMove();
        if (!move) continue;
        
        // Calculate cost before the move
        int costBefore = calculateCost(currentSolution);
        
        // Apply the move
        applyMove(move);
        
        // Pack the solution to get updated coordinates
        currentSolution->pack();
        
        // Calculate new cost
        int costAfter = calculateCost(currentSolution);
        int costDifference = costAfter - costBefore;
        
        // Decide whether to accept the move
        if (acceptMove(costDifference, temperature)) {
            // Accept the move
            acceptedMoves++;
            acceptedMoveHistory.push_back(move);
            
            // Update best solution if improved
            if (costAfter < bestCost) {
                bestSolution = currentSolution->clone();
                bestCost = costAfter;
                improved = true;
                noImprovementCount = 0;
            } else {
                noImprovementCount++;
            }
        } else {
            // Reject the move
            undoMove(move);
            currentSolution->pack();
            movePool.releaseMove(move);
            rejectedMoves++;
        }
        
        totalIterations++;
    }
    
    return improved;
}

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
        
        cout << "Starting SA with initial temperature " << temperature << endl;
        cout << "Initial cost: " << currentCost << endl;
        
        // Log MovePool stats
        cout << "MovePool status: " << movePool.getAllocatedBlocks() << " blocks, " 
             << movePool.getFreeListSize() << " free moves" << endl;
        
        int consecNoImprovement = 0;
        int lastReportedCost = currentCost;
        int passCount = 0;
        
        // Main annealing loop
        while (temperature > finalTemperature) {
            passCount++;
            // Check for timeout at the beginning of each temperature
            if (checkTimeout()) {
                cout << "Timeout detected at temperature " << temperature 
                     << ". Returning best solution so far." << endl;
                validateBestSolution();
                return bestSolution;
            }
            
            // Process one temperature level
            cout << "Starting temperature pass " << passCount 
                 << " at T=" << temperature << endl;
            
            bool improved = processTemperature(temperature);
            
            // Report detailed stats for this temperature level
            double acceptRate = (acceptedMoves > 0) ? 
                static_cast<double>(acceptedMoves) / (acceptedMoves + rejectedMoves) * 100.0 : 0.0;
            
            cout << "Temperature: " << temperature 
                 << ", Best cost: " << bestCost 
                 << ", Current cost: " << currentCost 
                 << ", Delta: " << (lastReportedCost - currentCost)
                 << ", Accept rate: " << acceptRate << "%" 
                 << ", Consec no improvement: " << consecNoImprovement 
                 << endl;
                 
            lastReportedCost = currentCost;
            
            // Check for stagnation
            if (!improved) {
                consecNoImprovement++;
                
                // If stagnant for 3 consecutive temperatures, apply extra cooling
                if (consecNoImprovement >= noImprovementLimit) {
                    double oldTemp = temperature;
                    temperature *= 0.5;  // Extra cooling
                    consecNoImprovement = 0;
                    cout << "Applying extra cooling due to stagnation. " 
                         << "Temperature: " << oldTemp << " -> " << temperature << endl;
                }
            } else {
                consecNoImprovement = 0;
                cout << "Solution improved! New best cost: " << bestCost << endl;
            }
            
            // Regular cooling
            temperature *= coolingRate;
            
            // Log MovePool stats periodically
            if (passCount % 10 == 0) {
                cout << "MovePool status: " << movePool.getAllocatedBlocks() << " blocks, " 
                     << movePool.getFreeListSize() << " free moves" << endl;
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

map<string, int> SimulatedAnnealing::getStatistics() const {
    map<string, int> stats;
    stats["totalIterations"] = totalIterations;
    stats["acceptedMoves"] = acceptedMoves;
    stats["rejectedMoves"] = rejectedMoves;
    stats["noImprovementCount"] = noImprovementCount;
    return stats;
}