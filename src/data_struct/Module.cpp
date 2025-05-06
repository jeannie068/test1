#include "Module.hpp"
#include <iostream>
#include <algorithm>



// Constructors
Module::Module(const std::string& name, int width, int height)
    : name(name), width(width), height(height), x(0), y(0), isRotated(false) {
    if (width <= 0 || height <= 0) {
        std::cerr << "Warning: Created module '" << name 
                  << "' with invalid dimensions: " << width << "x" << height << std::endl;
    }
}

Module::Module(const Module& other)
    : name(other.name), width(other.width), height(other.height),
      x(other.x), y(other.y), isRotated(other.isRotated) {
}

// Getters
const std::string& Module::getName() const {
    return name;
}

int Module::getWidth() const {
    return isRotated ? height : width;
}

int Module::getHeight() const {
    return isRotated ? width : height;
}

int Module::getOriginalWidth() const {
    return width;
}

int Module::getOriginalHeight() const {
    return height;
}

int Module::getX() const {
    return x;
}

int Module::getY() const {
    return y;
}

bool Module::getRotated() const {
    return isRotated;
}

// Setters
void Module::setPosition(int x, int y) {
    if (x < 0 || y < 0) {
        std::cerr << "Warning: Setting negative position (" << x << "," << y 
                  << ") for module '" << name << "'" << std::endl;
    }
    this->x = std::max(0, x); // Ensure non-negative coordinates
    this->y = std::max(0, y);
}

void Module::rotate() {
    isRotated = !isRotated;
}

void Module::setRotation(bool rotate) {
    isRotated = rotate;
}

// Utility functions
int Module::getArea() const {
    return width * height; // Area doesn't change with rotation
}

bool Module::overlaps(const Module& other) const {
    // Check if two modules overlap
    if (x + getWidth() <= other.x || other.x + other.getWidth() <= x) {
        return false; // No horizontal overlap
    }
    if (y + getHeight() <= other.y || other.y + other.getHeight() <= y) {
        return false; // No vertical overlap
    }
    return true; // There is overlap
}

/**
 * Returns the distance between the modules
 * Using minimum distance between edges
 */
int Module::getDistance(const Module& other) const {
    // Check if modules overlap
    if (overlaps(other)) {
        return 0;
    }
    
    // Calculate horizontal distance
    int horizontalDistance = 0;
    if (x + getWidth() <= other.x) {
        // This module is to the left of other
        horizontalDistance = other.x - (x + getWidth());
    } else if (other.x + other.getWidth() <= x) {
        // Other module is to the left of this
        horizontalDistance = x - (other.x + other.getWidth());
    }
    
    // Calculate vertical distance
    int verticalDistance = 0;
    if (y + getHeight() <= other.y) {
        // This module is below other
        verticalDistance = other.y - (y + getHeight());
    } else if (other.y + other.getHeight() <= y) {
        // Other module is below this
        verticalDistance = y - (other.y + other.getHeight());
    }
    
    // Return Manhattan distance (sum of horizontal and vertical distances)
    return horizontalDistance + verticalDistance;
}

/**
 * Gets the area of overlap with another module
 * Returns 0 if no overlap
 */
int Module::getOverlapArea(const Module& other) const {
    if (!overlaps(other)) {
        return 0;
    }
    
    int overlapWidth = std::min(x + getWidth(), other.x + other.getWidth()) - 
                      std::max(x, other.x);
    int overlapHeight = std::min(y + getHeight(), other.y + other.getHeight()) - 
                       std::max(y, other.y);
    
    return overlapWidth * overlapHeight;
}

/**
 * Attempts to resolve overlap with another module
 * Returns true if successful
 */
bool Module::resolveOverlap(Module& other) {
    if (!overlaps(other)) {
        return true; // No overlap to resolve
    }
    
    // Calculate overlap amounts in each direction
    int overlapRight = (x + getWidth()) - other.x;
    int overlapLeft = (other.x + other.getWidth()) - x;
    int overlapTop = (y + getHeight()) - other.y;
    int overlapBottom = (other.y + other.getHeight()) - y;
    
    // Find the smallest overlap direction
    int minOverlap = std::min({overlapRight, overlapLeft, overlapTop, overlapBottom});
    
    // Move the module with the smallest distance
    if (minOverlap == overlapRight) {
        // Move other to the right
        other.setPosition(x + getWidth(), other.y);
    } else if (minOverlap == overlapLeft) {
        // Move this to the right
        setPosition(other.x + other.getWidth(), y);
    } else if (minOverlap == overlapTop) {
        // Move other up
        other.setPosition(other.x, y + getHeight());
    } else {
        // Move this up
        setPosition(x, other.y + other.getHeight());
    }
    
    return true;
}

int Module::getRight() const {
    return x + getWidth();
}

int Module::getTop() const {
    return y + getHeight();
}

void Module::print() const {
    std::cout << "Module: " << name << std::endl;
    std::cout << "  Position: (" << x << ", " << y << ")" << std::endl;
    std::cout << "  Dimensions: " << getWidth() << " x " << getHeight() << std::endl;
    std::cout << "  Rotated: " << (isRotated ? "Yes" : "No") << std::endl;
}