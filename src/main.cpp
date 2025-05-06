#include <iostream>
#include <string>
#include <memory>
#include <ctime>
#include <cstdlib>
#include <chrono>
#include <thread>
#include <atomic>

#include "parser/Parser.hpp"
#include "solver/solver.hpp"
#include "data_struct/Module.hpp"
#include "data_struct/SymmetryConstraint.hpp"
#include "utils/TimeoutManager.hpp"
using namespace std;

void printUsage(const char* programName) {
    cout << "Usage: " << programName << " <input_file> <output_file> [area_ratio]" << endl;
    cout << "  input_file: Path to the input .txt file" << endl;
    cout << "  output_file: Path to the output .out file" << endl;
    cout << "  area_ratio: Optional parameter for area vs. wirelength weight ratio (default 1.0)" << endl;
}

int main(int argc, char* argv[]) {
    // Check command line arguments
    if (argc < 3 || argc > 4) {
        printUsage(argv[0]);
        return 1;
    }

    string inputFile = argv[1];
    string outputFile = argv[2];
    double areaRatio = 1.0;  // Default area weight ratio
    
    // Parse optional area ratio parameter
    if (argc == 4) {
        try {
            areaRatio = stod(argv[3]);
            if (areaRatio < 0.0) {
                cerr << "Error: Area ratio must be non-negative" << endl;
                return 1;
            }
        } catch (const exception& e) {
            cerr << "Error parsing area ratio: " << e.what() << endl;
            return 1;
        }
    }
    
    // Record start time
    auto startTime = chrono::steady_clock::now();
    
    // Parse input file first to get module data
    map<string, shared_ptr<Module>> modules;
    vector<shared_ptr<SymmetryGroup>> symmetryGroups;
    
    cout << "Parsing input file: " << inputFile << endl;
    if (!Parser::parseInputFile(inputFile, modules, symmetryGroups)) {
        cerr << "Error parsing input file" << endl;
        return 1;
    }
    
    // Create solver with parsed data
    PlacementSolver solver;
    solver.loadProblem(modules, symmetryGroups);
    
    // Configure solver parameters
    solver.setAnnealingParameters(
        1000.0,     // Initial temperature
        1.0,        // Final temperature
        0.85,       // Cooling rate
        250,        // Iterations per temperature
        500        // No improvement limit
    );
    
    solver.setPerturbationProbabilities(
        0.3,        // Rotate probability
        0.3,        // Move probability
        0.3,        // Swap probability
        0.05,       // Change representative probability
        0.05        // Convert symmetry type probability
    );
    
    solver.setCostWeights(
        areaRatio,      // Area weight
        1.0 - areaRatio // Wirelength weight (complementary to area weight)
    );
    
    solver.setRandomSeed(static_cast<unsigned int>(time(nullptr)));
    
    // Create the timeout manager with emergency shutdown after 10 seconds
    // Main timeout is set to 290 seconds (4m 50s)
    auto timeoutManager = make_shared<TimeoutManager>(240, 10);
    
    // Set a custom emergency callback that writes the output before exiting
    timeoutManager->setEmergencyCallback([&]() {
        cout << "\nEmergency shutdown activated. Writing best solution found so far." << endl;
        
        // Make sure we calculate the area of the solution
        solver.finalizeSolution();
        
        int solutionArea = solver.getSolutionArea();
        auto solutionModules = solver.getSolutionModules();
        
        // Write output file
        cout << "Writing emergency output file: " << outputFile << endl;
        if (!Parser::writeOutputFile(outputFile, solutionModules, solutionArea)) {
            cerr << "Error writing output file during emergency shutdown" << endl;
        }
        exit(0); // Clean exit
    });
    
    // Start the watchdog
    timeoutManager->startWatchdog();
    
    // Pass the timeout manager to the solver
    solver.setTimeoutManager(timeoutManager);
    
    try {
        // Solve the placement problem
        cout << "Solving placement problem..." << endl;
        bool success = solver.solve();
        
        // If we've reached here without timeout, write the output
        if (success || timeoutManager->hasTimedOut()) {
            // Make sure we calculate the area of the solution
            solver.finalizeSolution();
            
            int solutionArea = solver.getSolutionArea();
            auto solutionModules = solver.getSolutionModules();
            
            if (timeoutManager->hasTimedOut()) {
                cout << "Writing best solution found before timeout..." << endl;
            } else {
                cout << "Writing final solution..." << endl;
            }
            
            // Write output file
            cout << "Writing output file: " << outputFile << endl;
            if (!Parser::writeOutputFile(outputFile, solutionModules, solutionArea)) {
                cerr << "Error writing output file" << endl;
                return 1;
            }
            
            // Display execution time
            auto endTime = chrono::steady_clock::now();
            auto executionTime = chrono::duration_cast<chrono::seconds>(endTime - startTime).count();
            cout << "Execution time: " << executionTime << " seconds" << endl;
            cout << "Final area: " << solutionArea << endl;
            
            return 0;
        } else {
            cerr << "Error solving placement problem" << endl;
            return 1;
        }
    }
    catch (const runtime_error& e) {
        string errorMsg = e.what();
        if (errorMsg.find("Timeout") != string::npos) {
            cout << "Caught timeout exception: " << errorMsg << endl;
            
            // Make sure we calculate the area of the solution
            solver.finalizeSolution();
            
            // Try to write the output with the best solution so far
            int solutionArea = solver.getSolutionArea();
            auto solutionModules = solver.getSolutionModules();
            
            cout << "Writing output file after timeout exception: " << outputFile << endl;
            Parser::writeOutputFile(outputFile, solutionModules, solutionArea);
            
            return 0;
        } else {
            cerr << "Unexpected runtime error: " << errorMsg << endl;
            return 1;
        }
    }
    catch (const exception& e) {
        cerr << "Unexpected exception: " << e.what() << endl;
        return 1;
    }
}