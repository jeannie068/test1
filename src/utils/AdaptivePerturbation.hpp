#pragma once

#include <string>
#include <map>
#include <iostream>

class AdaptivePerturbation {
private:
    // Track performance of different operations
    struct OperationStats {
        int attempts = 0;
        int successes = 0;
        double totalImprovement = 0.0;
        double averageImprovement = 0.0;
    };
    
    // Operation statistics
    std::map<std::string, OperationStats> opStats;
    
    // Current probabilities
    double probRotate;
    double probMove;
    double probSwap;
    double probChangeRep;
    double probConvertSym;
    
    // Learning rate for probability updates
    double learningRate = 0.1;
    
    // Minimum probabilities to ensure all operations have a chance
    double minProbRotate = 0.1;
    double minProbMove = 0.3;
    double minProbSwap = 0.1;
    double minProbChangeRep = 0.02;
    double minProbConvertSym = 0.02;
    
public:
    /**
     * Constructor
     * 
     * @param rotate Initial probability for rotate operation
     * @param move Initial probability for move operation
     * @param swap Initial probability for swap operation
     * @param changeRep Initial probability for change representative operation
     * @param convertSym Initial probability for convert symmetry type operation
     */
    AdaptivePerturbation(double rotate, double move, double swap, 
                        double changeRep, double convertSym);
    
    /**
     * Record an attempt of an operation
     * 
     * @param operation Name of the operation
     */
    void recordAttempt(const std::string& operation);
    
    /**
     * Record a successful operation
     * 
     * @param operation Name of the operation
     * @param improvement Amount of improvement in cost
     */
    void recordSuccess(const std::string& operation, double improvement);
    
    /**
     * Update probabilities based on operation success and improvement
     */
    void updateProbabilities();
    
    /**
     * Get current probability for rotate operation
     * 
     * @return Probability value [0.0-1.0]
     */
    double getRotateProbability() const;
    
    /**
     * Get current probability for move operation
     * 
     * @return Probability value [0.0-1.0]
     */
    double getMoveProbability() const;
    
    /**
     * Get current probability for swap operation
     * 
     * @return Probability value [0.0-1.0]
     */
    double getSwapProbability() const;
    
    /**
     * Get current probability for change representative operation
     * 
     * @return Probability value [0.0-1.0]
     */
    double getChangeRepProbability() const;
    
    /**
     * Get current probability for convert symmetry type operation
     * 
     * @return Probability value [0.0-1.0]
     */
    double getConvertSymProbability() const;
    
    /**
     * Print statistics about operation performance
     */
    void printStats() const;
};