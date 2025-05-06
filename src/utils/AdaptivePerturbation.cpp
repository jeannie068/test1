#include "../utils/AdaptivePerturbation.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

AdaptivePerturbation::AdaptivePerturbation(double rotate, double move, double swap, 
                                     double changeRep, double convertSym) {
    // Initialize with provided probabilities
    probRotate = rotate;
    probMove = move;
    probSwap = swap;
    probChangeRep = changeRep;
    probConvertSym = convertSym;
    
    // Normalize probabilities to sum to 1.0
    double sum = probRotate + probMove + probSwap + probChangeRep + probConvertSym;
    if (sum > 0.0) {
        probRotate /= sum;
        probMove /= sum;
        probSwap /= sum;
        probChangeRep /= sum;
        probConvertSym /= sum;
    } else {
        // Default values if all probabilities are zero or negative
        probRotate = 0.3;
        probMove = 0.4;
        probSwap = 0.2;
        probChangeRep = 0.05;
        probConvertSym = 0.05;
    }
    
    // Initialize statistics
    opStats["rotate"] = OperationStats();
    opStats["move"] = OperationStats();
    opStats["swap"] = OperationStats();
    opStats["changeRep"] = OperationStats();
    opStats["convertSym"] = OperationStats();
}

void AdaptivePerturbation::recordAttempt(const std::string& operation) {
    // Make sure the operation exists in our stats
    if (opStats.find(operation) == opStats.end()) {
        opStats[operation] = OperationStats();
    }
    
    opStats[operation].attempts++;
}

void AdaptivePerturbation::recordSuccess(const std::string& operation, double improvement) {
    // Make sure the operation exists in our stats
    if (opStats.find(operation) == opStats.end()) {
        opStats[operation] = OperationStats();
        opStats[operation].attempts = 1; // Set at least one attempt if not recorded
    }
    
    opStats[operation].successes++;
    opStats[operation].totalImprovement += improvement;
    
    // Update average improvement
    if (opStats[operation].successes > 0) {
        opStats[operation].averageImprovement = 
            opStats[operation].totalImprovement / opStats[operation].successes;
    }
}

void AdaptivePerturbation::updateProbabilities() {
    // Calculate total weighted improvement
    double totalWeightedImprovement = 0.0;
    double totalSuccessRate = 0.0;
    
    for (auto& pair : opStats) {
        const std::string& op = pair.first;
        OperationStats& stats = pair.second;
        
        // Skip operations with no attempts
        if (stats.attempts == 0) continue;
        
        // Calculate success rate
        double successRate = static_cast<double>(stats.successes) / stats.attempts;
        
        // Add to total success rate for normalization
        totalSuccessRate += successRate;
        
        // Calculate weighted improvement (success rate * average improvement)
        if (stats.successes > 0) {
            double weightedImprovement = successRate * stats.averageImprovement;
            totalWeightedImprovement += weightedImprovement;
        }
    }
    
    // Skip update if no improvements or success rates
    if (totalWeightedImprovement <= 0.0 || totalSuccessRate <= 0.0) {
        std::cout << "Skipping probability update - no improvements or successes" << std::endl;
        return;
    }
    
    // Calculate new probabilities based on success rates and weighted improvements
    double newProbRotate = calculateProbability("rotate", totalWeightedImprovement, totalSuccessRate);
    double newProbMove = calculateProbability("move", totalWeightedImprovement, totalSuccessRate);
    double newProbSwap = calculateProbability("swap", totalWeightedImprovement, totalSuccessRate);
    double newProbChangeRep = calculateProbability("changeRep", totalWeightedImprovement, totalSuccessRate);
    double newProbConvertSym = calculateProbability("convertSym", totalWeightedImprovement, totalSuccessRate);
    
    // Apply minimum probabilities
    newProbRotate = std::max(newProbRotate, minProbRotate);
    newProbMove = std::max(newProbMove, minProbMove);
    newProbSwap = std::max(newProbSwap, minProbSwap);
    newProbChangeRep = std::max(newProbChangeRep, minProbChangeRep);
    newProbConvertSym = std::max(newProbConvertSym, minProbConvertSym);
    
    // Normalize to sum to 1.0
    double sum = newProbRotate + newProbMove + newProbSwap + 
                 newProbChangeRep + newProbConvertSym;
    
    if (sum > 0.0) {
        newProbRotate /= sum;
        newProbMove /= sum;
        newProbSwap /= sum;
        newProbChangeRep /= sum;
        newProbConvertSym /= sum;
    }
    
    // Update probabilities with learning rate
    probRotate = (1 - learningRate) * probRotate + learningRate * newProbRotate;
    probMove = (1 - learningRate) * probMove + learningRate * newProbMove;
    probSwap = (1 - learningRate) * probSwap + learningRate * newProbSwap;
    probChangeRep = (1 - learningRate) * probChangeRep + learningRate * newProbChangeRep;
    probConvertSym = (1 - learningRate) * probConvertSym + learningRate * newProbConvertSym;
    
    // Renormalize (just in case)
    sum = probRotate + probMove + probSwap + probChangeRep + probConvertSym;
    if (sum > 0.0) {
        probRotate /= sum;
        probMove /= sum;
        probSwap /= sum;
        probChangeRep /= sum;
        probConvertSym /= sum;
    }
    
    // Reset statistics periodically to adapt to changing landscape
    // Instead of halving, use a decay factor to avoid numerical issues
    for (auto& pair : opStats) {
        OperationStats& stats = pair.second;
        
        // We want to decay the values while preserving the ratio
        const double decayFactor = 0.7; // 70% of original value
        
        if (stats.attempts > 0) {
            int newAttempts = std::max(1, static_cast<int>(stats.attempts * decayFactor));
            int newSuccesses = std::max(0, static_cast<int>(stats.successes * decayFactor));
            
            stats.attempts = newAttempts;
            stats.successes = newSuccesses;
            
            if (stats.successes > 0) {
                // Keep the same average improvement
                stats.totalImprovement = stats.averageImprovement * stats.successes;
            } else {
                stats.totalImprovement = 0.0;
                stats.averageImprovement = 0.0;
            }
        }
    }
}

double AdaptivePerturbation::calculateProbability (
    const std::string& operation, 
    double totalWeightedImprovement,
    double totalSuccessRate) {
    
    auto it = opStats.find(operation);
    if (it == opStats.end() || it->second.attempts == 0) {
        // If operation not found or has no attempts, use minimum probability
        if (operation == "rotate") return minProbRotate;
        if (operation == "move") return minProbMove;
        if (operation == "swap") return minProbSwap;
        if (operation == "changeRep") return minProbChangeRep;
        if (operation == "convertSym") return minProbConvertSym;
        return 0.1; // Default fallback
    }
    
    const OperationStats& stats = it->second;
    
    // Calculate success rate
    double successRate = static_cast<double>(stats.successes) / stats.attempts;
    
    // Calculate probability based on both success rate and improvement
    double probability = 0.0;
    
    if (stats.successes > 0) {
        // Factor in both success rate and average improvement
        double weightedImprovement = successRate * stats.averageImprovement;
        probability = (0.3 * successRate / totalSuccessRate) + 
                     (0.7 * weightedImprovement / totalWeightedImprovement);
    } else {
        // If no successes, just use a scaled success rate
        probability = 0.3 * successRate / totalSuccessRate;
    }
    
    return probability;
}

double AdaptivePerturbation::getRotateProbability() const {
    return probRotate;
}

double AdaptivePerturbation::getMoveProbability() const {
    return probMove;
}

double AdaptivePerturbation::getSwapProbability() const {
    return probSwap;
}

double AdaptivePerturbation::getChangeRepProbability() const {
    return probChangeRep;
}

double AdaptivePerturbation::getConvertSymProbability() const {
    return probConvertSym;
}

void AdaptivePerturbation::printStats() const {
    std::cout << "Operation Statistics:" << std::endl;
    
    for (const auto& pair : opStats) {
        const std::string& op = pair.first;
        const OperationStats& stats = pair.second;
        
        double successRate = stats.attempts > 0 ? 
            static_cast<double>(stats.successes) / stats.attempts : 0.0;
        
        std::cout << "  " << op << ": " 
             << "Attempts: " << stats.attempts 
             << ", Successes: " << stats.successes 
             << ", Rate: " << (successRate * 100.0) << "%" 
             << ", Avg Improvement: " << stats.averageImprovement 
             << std::endl;
    }
    
    std::cout << "Current Probabilities:" << std::endl;
    std::cout << "  Rotate: " << (probRotate * 100.0) << "%" << std::endl;
    std::cout << "  Move: " << (probMove * 100.0) << "%" << std::endl;
    std::cout << "  Swap: " << (probSwap * 100.0) << "%" << std::endl;
    std::cout << "  ChangeRep: " << (probChangeRep * 100.0) << "%" << std::endl;
    std::cout << "  ConvertSym: " << (probConvertSym * 100.0) << "%" << std::endl;
}