#include "../utils/AdaptivePerturbation.hpp"
#include <algorithm>
#include <cmath>

AdaptivePerturbation::AdaptivePerturbation(double rotate, double move, double swap, 
                                     double changeRep, double convertSym) {
    // Initialize with provided probabilities
    probRotate = rotate;
    probMove = move;
    probSwap = swap;
    probChangeRep = changeRep;
    probConvertSym = convertSym;
    
    // Initialize statistics
    opStats["rotate"] = OperationStats();
    opStats["move"] = OperationStats();
    opStats["swap"] = OperationStats();
    opStats["changeRep"] = OperationStats();
    opStats["convertSym"] = OperationStats();
}

void AdaptivePerturbation::recordAttempt(const std::string& operation) {
    opStats[operation].attempts++;
}

void AdaptivePerturbation::recordSuccess(const std::string& operation, double improvement) {
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
    
    for (auto& pair : opStats) {
        const std::string& op = pair.first;
        OperationStats& stats = pair.second;
        
        // Skip operations with no attempts
        if (stats.attempts == 0) continue;
        
        // Calculate success rate and weighted improvement
        double successRate = static_cast<double>(stats.successes) / stats.attempts;
        double weightedImprovement = successRate * stats.averageImprovement;
        
        // Add to total
        totalWeightedImprovement += weightedImprovement;
    }
    
    // Skip update if no improvements
    if (totalWeightedImprovement <= 0.0) return;
    
    // Calculate new probabilities based on weighted improvements
    double newProbRotate = opStats["rotate"].attempts > 0 ? 
        (opStats["rotate"].successes * opStats["rotate"].averageImprovement) / 
         totalWeightedImprovement : minProbRotate;
    
    double newProbMove = opStats["move"].attempts > 0 ? 
        (opStats["move"].successes * opStats["move"].averageImprovement) / 
         totalWeightedImprovement : minProbMove;
    
    double newProbSwap = opStats["swap"].attempts > 0 ? 
        (opStats["swap"].successes * opStats["swap"].averageImprovement) / 
         totalWeightedImprovement : minProbSwap;
    
    double newProbChangeRep = opStats["changeRep"].attempts > 0 ? 
        (opStats["changeRep"].successes * opStats["changeRep"].averageImprovement) / 
         totalWeightedImprovement : minProbChangeRep;
    
    double newProbConvertSym = opStats["convertSym"].attempts > 0 ? 
        (opStats["convertSym"].successes * opStats["convertSym"].averageImprovement) / 
         totalWeightedImprovement : minProbConvertSym;
    
    // Apply minimum probabilities
    newProbRotate = std::max(newProbRotate, minProbRotate);
    newProbMove = std::max(newProbMove, minProbMove);
    newProbSwap = std::max(newProbSwap, minProbSwap);
    newProbChangeRep = std::max(newProbChangeRep, minProbChangeRep);
    newProbConvertSym = std::max(newProbConvertSym, minProbConvertSym);
    
    // Normalize to sum to 1.0
    double sum = newProbRotate + newProbMove + newProbSwap + 
                 newProbChangeRep + newProbConvertSym;
    
    newProbRotate /= sum;
    newProbMove /= sum;
    newProbSwap /= sum;
    newProbChangeRep /= sum;
    newProbConvertSym /= sum;
    
    // Update probabilities with learning rate
    probRotate = (1 - learningRate) * probRotate + learningRate * newProbRotate;
    probMove = (1 - learningRate) * probMove + learningRate * newProbMove;
    probSwap = (1 - learningRate) * probSwap + learningRate * newProbSwap;
    probChangeRep = (1 - learningRate) * probChangeRep + learningRate * newProbChangeRep;
    probConvertSym = (1 - learningRate) * probConvertSym + learningRate * newProbConvertSym;
    
    // Renormalize (just in case)
    sum = probRotate + probMove + probSwap + probChangeRep + probConvertSym;
    probRotate /= sum;
    probMove /= sum;
    probSwap /= sum;
    probChangeRep /= sum;
    probConvertSym /= sum;
    
    // Reset statistics periodically to adapt to changing landscape
    for (auto& pair : opStats) {
        OperationStats& stats = pair.second;
        stats.attempts = std::max(1, stats.attempts / 2);
        stats.successes = std::max(0, stats.successes / 2);
        if (stats.successes > 0) {
            stats.totalImprovement = stats.averageImprovement * stats.successes;
        } else {
            stats.totalImprovement = 0.0;
        }
    }
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