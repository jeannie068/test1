// SA.hpp - Optimized Simulated Annealing for Analog Placement
#pragma once

#include <memory>
#include <random>
#include <cmath>
#include <chrono>
#include <functional>
#include <vector>
#include <string>
#include <unordered_map>
#include <limits>
#include "../data_struct/HBStarTree.hpp"
#include "../utils/TimeoutManager.hpp"

// Forward declarations
class Move;
class MovePool;

// Object pool for Move objects to reduce memory allocations
class MovePool {
private:
    static constexpr size_t BLOCK_SIZE = 1024; // Increase block size from 256 to 1024
    std::vector<std::vector<Move*>> blocks;
    std::vector<Move*> freeList;
    int allocations = 0; // Track number of blocks allocated

public:
    MovePool();
    ~MovePool();

    // Create a new move from the pool
    Move* createMove(const std::string& type, 
                    const std::string& param1 = "", 
                    const std::string& param2 = "", 
                    bool boolParam = false);

    // Return a move to the pool
    void releaseMove(Move* move);

    // Allocate a new block of moves
    void allocateBlock();
    
    // Get stats about the pool
    int getAllocatedBlocks() const { return allocations; }
    int getFreeListSize() const { return freeList.size(); }
};

// Lightweight Move class to represent a single perturbation
// In SA.hpp - Update Move class
class Move {
    friend class MovePool;
    friend class SimulatedAnnealing;

private:
    std::string operationType;      // "rotate", "move", "swap", "changeRep", "convertSym"
    std::string param1;             // First parameter (e.g., moduleName, nodeName1)
    std::string param2;             // Second parameter (e.g., newParentName, nodeName2)
    bool boolParam;                 // Boolean parameter (e.g., asLeftChild)
    
    // State for undoing moves
    std::string originalParent;     // Original parent for move operations
    bool wasLeftChild;              // Whether the node was a left child
    std::string originalRepresentative; // Original representative for changeRep
    SymmetryType originalSymType;   // Original symmetry type

    // Private constructor - only MovePool can create Moves
    Move(const std::string& type, 
         const std::string& p1 = "", 
         const std::string& p2 = "", 
         bool bp = false);

public:
    // No copy/move constructors - managed by the pool
    Move(const Move&) = delete;
    Move& operator=(const Move&) = delete;
    Move(Move&&) = delete;
    Move& operator=(Move&&) = delete;

    // Getters
    const std::string& getType() const { return operationType; }
    const std::string& getParam1() const { return param1; }
    const std::string& getParam2() const { return param2; }
    bool getBoolParam() const { return boolParam; }
    
    // Reset the move state
    void reset() {
        operationType = "none";
        param1 = "";
        param2 = "";
        boolParam = false;
        originalParent = "";
        wasLeftChild = false;
        originalRepresentative = "";
    }
};

class SimulatedAnnealing {
private:
    // The current state of the placement solution
    std::shared_ptr<HBStarTree> currentSolution;
    
    // The best solution found so far
    std::shared_ptr<HBStarTree> bestSolution;
    
    // Cost of the current solution
    int currentCost;
    
    // Cost of the best solution
    int bestCost;
    
    // SA parameters
    double initialTemperature;
    double finalTemperature;
    double coolingRate;
    int movesPerTemperature;
    int noImprovementLimit;
    
    // Random number generation - mutable to allow usage in const methods
    mutable std::mt19937 rng;
    mutable std::uniform_real_distribution<double> uniformDist;
    
    // Perturbation probabilities
    double probRotate;
    double probMove;
    double probSwap;
    double probChangeRepresentative;
    double probConvertSymmetryType;
    
    // Statistics
    int totalIterations;
    int acceptedMoves;
    int rejectedMoves;
    int noImprovementCount;
    
    // Cost function weight parameters
    double areaWeight;
    double wirelengthWeight;

    // Timeout related
    std::chrono::steady_clock::time_point startTime;
    std::chrono::seconds timeoutSeconds;
    std::shared_ptr<TimeoutManager> timeoutManager;
    bool checkTimeout() const;

    // Adaptive perturbation system
    std::string lastOperation;
    
    // Move pool for memory efficiency
    MovePool movePool;
    
    // Move history for the current temperature
    std::vector<Move*> acceptedMoveHistory;
    
    /**
     * Calculates the cost of a solution
     */
    int calculateCost(const std::shared_ptr<HBStarTree>& solution) const;
    
    /**
     * Generates a random perturbation move
     */
    Move* generateMove();
    
    /**
     * Applies a move to the current solution
     */
    void applyMove(Move* move);
    
    /**
     * Undoes a move on the current solution
     */
    void undoMove(Move* move);
    
    /**
     * Decides whether to accept a move based on the cost difference and temperature
     * 
     * @param costDifference Difference in cost (new - old)
     * @param temperature Current temperature
     * @return True if the move should be accepted, false otherwise
     */
    bool acceptMove(int costDifference, double temperature) const;
    
    /**
     * Select a random module
     * 
     * @return Name of the selected module
     */
    std::string selectRandomModule() const;
    
    /**
     * Select a random symmetry group
     * 
     * @return Name of the selected symmetry group
     */
    std::string selectRandomSymmetryGroup() const;
    
    /**
     * Select a random node (module or symmetry group)
     * 
     * @return Name of the selected node
     */
    std::string selectRandomNode() const;
    
    /**
     * Initializes the temperature based on average cost delta
     */
    void initializeTemperature();
    
    /**
     * Process one temperature level
     * 
     * @param temperature Current temperature
     * @return True if any improvement was made
     */
    bool processTemperature(double temperature);

public:
    /**
     * Constructor
     */
    SimulatedAnnealing(std::shared_ptr<HBStarTree> initialSolution,
                      double initialTemp = 1000.0,
                      double finalTemp = 0.1,
                      double coolingRate = 0.90,
                      int iterations = 1500,
                      int noImprovementLimit = 3);
    
    /**
     * Sets the perturbation probabilities
     * 
     * @param rotate Probability of rotation operation
     * @param move Probability of move operation
     * @param swap Probability of swap operation
     * @param changeRep Probability of change representative operation
     * @param convertSym Probability of convert symmetry type operation
     */
    void setPerturbationProbabilities(double rotate, double move, double swap, 
                                     double changeRep, double convertSym);
    
    void validateBestSolution();
    string selectRandomRepresentativeModule() const;

    /**
     * Sets the cost function weights
     * 
     * @param area Weight for area term
     * @param wirelength Weight for wirelength term
     */
    void setCostWeights(double area, double wirelength);
    
    /**
     * Runs the simulated annealing algorithm
     * 
     * @return Best solution found
     */
    std::shared_ptr<HBStarTree> run();
    
    std::shared_ptr<HBStarTree> getBestSolution() const;
    int getBestCost() const;
    
    /**
     * Gets statistics about the annealing process
     * 
     * @return Map of statistic name to value
     */
    std::map<std::string, int> getStatistics() const;
    
    /**
     * Sets the random seed for reproducible results
     * 
     * @param seed Random seed
     */
    void setSeed(unsigned int seed);

    void setTimeoutManager(std::shared_ptr<TimeoutManager> manager);
};