// TimeoutManager.hpp
#pragma once

#include <atomic>
#include <thread>
#include <chrono>
#include <iostream>
#include <csignal>
#include <functional>

class TimeoutManager {
private:
    std::atomic<bool> timeoutOccurred;
    std::thread watchdogThread;
    std::chrono::steady_clock::time_point startTime;
    int timeoutSeconds;
    
    // Emergency timer and callback
    std::thread emergencyThread;
    int emergencyTimeoutSeconds;
    std::function<void()> emergencyCallback;

public:
    TimeoutManager(int seconds = 300, int emergencySeconds = 10) : 
        timeoutOccurred(false),
        startTime(std::chrono::steady_clock::now()),
        timeoutSeconds(seconds),
        emergencyTimeoutSeconds(emergencySeconds) {
        
        // Set default emergency callback to forcefully exit
        emergencyCallback = []() {
            std::cerr << "\nEmergency shutdown activated. Forcing exit.\n";
            std::exit(1); // Hard exit
        };
    }

    ~TimeoutManager() {
        if (watchdogThread.joinable()) {
            watchdogThread.join();
        }
        if (emergencyThread.joinable()) {
            emergencyThread.join();
        }
    }

    void startWatchdog() {
        // Reset the start time when the watchdog is started
        startTime = std::chrono::steady_clock::now();
        timeoutOccurred = false;  // Also reset the timeout flag
        
        watchdogThread = std::thread([this]() {
            while (!timeoutOccurred) {
                auto currentTime = std::chrono::steady_clock::now();
                auto elapsedTime = std::chrono::duration_cast<std::chrono::seconds>(
                    currentTime - startTime).count();
                
                if (elapsedTime >= timeoutSeconds) {
                    timeoutOccurred = true;
                    std::cout << "\nProgram timeout reached! Forcing termination...\n" << std::endl;
                    
                    // Start emergency shutdown timer
                    // If program doesn't exit gracefully, force it to exit
                    emergencyThread = std::thread([this]() {
                        std::this_thread::sleep_for(std::chrono::seconds(emergencyTimeoutSeconds));
                        if (emergencyCallback) {
                            emergencyCallback();
                        }
                    });
                    emergencyThread.detach(); // Let it run independently
                    break;
                }
                
                // Check every second
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        });
    }


    bool hasTimedOut() const {
        return timeoutOccurred;
    }

    void checkTimeout() {
        if (timeoutOccurred) {
            throw std::runtime_error("Timeout occurred");
        }
    }
    
    // Set a custom emergency callback
    void setEmergencyCallback(std::function<void()> callback) {
        emergencyCallback = callback;
    }
};