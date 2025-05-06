/**
 * Contour.hpp
 * 
 * Implementation of the Contour class for efficient packing in the ASF-B*-tree
 * placement algorithm. The contour data structure represents the skyline profile
 * of the currently placed modules, allowing for efficient height queries and updates.
 * Optimized implementation using doubly-linked list.
 */

#pragma once

#include <vector>
#include <utility>
#include <algorithm>
#include <limits>
#include <memory>
#include <functional>

/**
 * Contour segment representing a horizontal or vertical segment in the placement
 */
struct ContourSegment {
    int start;   // Start coordinate (x for horizontal, y for vertical)
    int end;     // End coordinate (x for horizontal, y for vertical)
    int height;  // Height (y for horizontal, x for vertical)
    
    ContourSegment(int start, int end, int height)
        : start(start), end(end), height(height) {}
        
    bool operator<(const ContourSegment& other) const {
        return start < other.start;
    }
};

/**
 * Node in the doubly-linked list representing a contour segment
 */
struct ContourNode {
    int start;   // Start coordinate
    int end;     // End coordinate
    int height;  // Height
    
    ContourNode* prev;  // Pointer to previous node
    ContourNode* next;  // Pointer to next node
    
    ContourNode(int s, int e, int h) : start(s), end(e), height(h), prev(nullptr), next(nullptr) {}
};

/**
 * Optimized Contour class using a doubly-linked list
 * for faster segment insertion and merging
 */
class Contour {
private:
    ContourNode* head;  // Head of the doubly-linked list
    ContourNode* tail;  // Tail of the doubly-linked list
    
    // Maintain max coordinates and heights for quick access
    int maxCoordinate;
    int maxHeight;
    
    // Find the node containing a coordinate
    ContourNode* findNode(int coordinate) const;
    
    // Merge adjacent nodes with the same height
    void mergeNodes();
    
public:
    Contour();
    Contour(const Contour& other);  // Copy constructor
    ~Contour();
    void clear();
    
    /**
     * Adds a segment to the contour
     */
    void addSegment(int start, int end, int height);
    
    /**
     * Gets the height of the contour at a specific range
     */
    int getHeight(int start, int end) const;
    
    /**
     * Gets all contour segments
     */
    std::vector<ContourSegment> getSegments() const;
    
    /**
     * Merges this contour with another contour
     */
    void merge(const Contour& other);
    
    /**
     * Gets the maximum coordinate value in the contour
     */
    int getMaxCoordinate() const;
    
    /**
     * Gets the maximum height value in the contour
     */
    int getMaxHeight() const;
    
    /**
     * Checks if the contour is empty
     */
    bool isEmpty() const;
};