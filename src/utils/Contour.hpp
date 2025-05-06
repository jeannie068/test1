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
 * Optimized Contour class using sorted vector of segments
 * with binary search for faster queries instead of map-based lookup
 */
class Contour {
private:
    // Using a sorted vector of segments for better cache locality
    std::vector<ContourSegment> segments;
    
    // Maintain max coordinates and heights for quick access
    int maxCoordinate;
    int maxHeight;
    
    // Binary search to find segment containing a point
    int findSegmentIndex(int coordinate) const;
    
    // Merge overlapping segments with same height
    void mergeSegments();
    
public:
    Contour();
    Contour(const Contour& other);
    ~Contour();
    void clear();
    
    /**
     * Adds a segment to the contour - O(log n) with binary search
     */
    void addSegment(int start, int end, int height);
    
    /**
     * Gets the height of the contour at a specific range - O(log n)
     */
    int getHeight(int start, int end) const;
    
    /**
     * Gets all contour segments
     */
    const std::vector<ContourSegment>& getSegments() const;
    
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