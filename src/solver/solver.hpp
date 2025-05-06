#pragma once
#include <memory>
#include <string>
#include <vector>
#include <map>
#include "../data_struct/HBStarTree.hpp"
#include "../data_struct/Module.hpp"
#include "../data_struct/SymmetryConstraint.hpp"
#include "../utils/SA.hpp"
#include "../utils/TimeoutManager.hpp"

/**
 * PlacementSolver class for solving the analog device placement problem
 * with symmetry constraints.
 * 
 * This class uses the HB*-tree representation and simulated annealing
 * to find an optimized placement solution.
 */
class PlacementSolver {
private:
    // The HB*-tree representation of the placement
    std::shared_ptr<HBStarTree> hbTree;
    
    // Modules in the placement
    std::map<std::string, std::shared_ptr<Module>> modules;
    
    // Symmetry constraints
    std::vector<std::shared_ptr<SymmetryGroup>> symmetryGroups;
    
    // Simulated annealing parameters
    double initialTemperature;
    double finalTemperature;
    double coolingRate;
    int iterationsPerTemperature;
    int noImprovementLimit;
    
    // Perturbation probabilities
    double probRotate;
    double probMove;
    double probSwap;
    double probChangeRep;
    double probConvertSym;
    
    // Cost function weights
    double areaWeight;
    double wirelengthWeight;
    
    // Random seed for reproducibility
    unsigned int randomSeed;
    
    // Statistics
    int totalArea;

    std::shared_ptr<TimeoutManager> timeoutManager;
    
    /**
     * Creates an initial placement solution
     */
    void createInitialSolution();
    
public:
    /**
     * Constructor
     */
    PlacementSolver();
    ~PlacementSolver();

    void setTimeoutManager(std::shared_ptr<TimeoutManager> manager);
    
    /**
     * Loads modules and symmetry constraints
     * 
     * @param modules Map of module names to modules
     * @param symmetryGroups Vector of symmetry groups
     */
    void loadProblem(const std::map<std::string, std::shared_ptr<Module>>& modules,
                    const std::vector<std::shared_ptr<SymmetryGroup>>& symmetryGroups);
    
    /**
     * Sets simulated annealing parameters
     * 
     * @param initialTemp Initial temperature
     * @param finalTemp Final temperature
     * @param coolRate Cooling rate
     * @param iterations Number of iterations per temperature
     * @param noImprovementLimit Maximum number of iterations without improvement
     */
    void setAnnealingParameters(double initialTemp, double finalTemp, double coolRate, 
                               int iterations, int noImprovementLimit);
    
    /**
     * Sets perturbation probabilities
     * 
     * @param rotate Probability of rotation operation
     * @param move Probability of move operation
     * @param swap Probability of swap operation
     * @param changeRep Probability of change representative operation
     * @param convertSym Probability of convert symmetry type operation
     */
    void setPerturbationProbabilities(double rotate, double move, double swap, 
                                     double changeRep, double convertSym);
    
    /**
     * Sets cost function weights
     * 
     * @param area Weight for area term
     * @param wirelength Weight for wirelength term
     */
    void setCostWeights(double area, double wirelength);
    
    /**
     * Sets random seed for reproducibility
     * 
     * @param seed Random seed
     */
    void setRandomSeed(unsigned int seed);
    
    /**
     * Solves the placement problem using simulated annealing
     * 
     * @return True if a valid solution was found, false otherwise
     */
    bool solve();
    
    /**
     * Finalizes the solution - ensures the HB*-tree is packed and area is calculated
     * This is crucial to call before getting solution area, especially after timeout
     */
    void finalizeSolution();
    
    /**
     * Gets the solution area
     * 
     * @return Total area of the placement
     */
    int getSolutionArea() const;
    
    /**
     * Gets the solution modules with their positions
     * 
     * @return Map of module names to modules
     */
    std::map<std::string, std::shared_ptr<Module>> getSolutionModules() const;
    
    /**
     * Gets placement solution statistics
     * 
     * @return Map of statistic name to value
     */
    std::map<std::string, int> getStatistics() const;
};