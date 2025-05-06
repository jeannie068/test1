/**
 * Contour.cpp
 * 
 * Implementation of the Contour class for efficient packing in the ASF-B*-tree
 * placement algorithm. The contour data structure represents the skyline profile
 * of the currently placed modules, allowing for O(log n) height queries and updates.
 * Optimized implementation using sorted vector and binary search
 */

#include "Contour.hpp"
#include <algorithm>
#include <cassert>

/**
 * Constructor
 */
Contour::Contour() : maxCoordinate(0), maxHeight(0) {
    // Initialize with empty contour
}

/**
 * Copy constructor
 */
Contour::Contour(const Contour& other) 
    : segments(other.segments),
      maxCoordinate(other.maxCoordinate),
      maxHeight(other.maxHeight) {
}

/**
 * Destructor
 */
Contour::~Contour() {
    // No manual cleanup needed
}

/**
 * Clears the contour
 */
void Contour::clear() {
    segments.clear();
    maxCoordinate = 0;
    maxHeight = 0;
}

/**
 * Binary search to find segment containing a coordinate
 * Returns the index of the segment, or -1 if not found
 */
int Contour::findSegmentIndex(int coordinate) const {
    if (segments.empty()) return -1;
    
    // Binary search optimized for faster lookup using upper_bound
    auto it = std::upper_bound(segments.begin(), segments.end(), coordinate,
                              [](int coord, const ContourSegment& seg) {
                                  return coord < seg.start;
                              });
    
    // If we're at the beginning, there's no segment containing coordinate
    if (it == segments.begin()) return -1;
    
    // Check the previous segment
    --it;
    if (it->start <= coordinate && coordinate < it->end) {
        return std::distance(segments.begin(), it);
    }
    
    return -1;
}

/**
 * Merge overlapping segments with the same height
 */
void Contour::mergeSegments() {
    if (segments.size() <= 1) return;
    
    std::vector<ContourSegment> mergedSegments;
    mergedSegments.reserve(segments.size());
    
    // Add the first segment
    mergedSegments.push_back(segments[0]);
    
    // Try to merge each subsequent segment
    for (size_t i = 1; i < segments.size(); ++i) {
        ContourSegment& last = mergedSegments.back();
        const ContourSegment& current = segments[i];
        
        // If the current segment can be merged with the last one
        if (last.end == current.start && last.height == current.height) {
            last.end = current.end;
        } else {
            mergedSegments.push_back(current);
        }
    }
    
    // Replace segments with merged segments
    segments = std::move(mergedSegments);
}

/**
 * Adds a segment to the contour
 * Optimized for O(log n) insertion time
 */
void Contour::addSegment(int start, int end, int height) {
    if (start >= end) return;  // Invalid segment
    
    // Update max values
    maxCoordinate = std::max(maxCoordinate, end);
    maxHeight = std::max(maxHeight, height);
    
    // Special case: empty contour
    if (segments.empty()) {
        segments.emplace_back(start, end, height);
        return;
    }
    
    // Find the first segment that starts after or at the new segment's start
    auto startIt = std::lower_bound(segments.begin(), segments.end(), 
                                  ContourSegment(start, start, 0),
                                  [](const ContourSegment& a, const ContourSegment& b) {
                                      return a.start < b.start;
                                  });
    
    // Find the first segment that starts after the new segment's end
    auto endIt = std::lower_bound(segments.begin(), segments.end(), 
                                ContourSegment(end, end, 0),
                                [](const ContourSegment& a, const ContourSegment& b) {
                                    return a.start < b.start;
                                });
    
    // If there's a segment that ends at start with the same height, extend it
    if (startIt != segments.begin()) {
        auto prevIt = startIt - 1;
        if (prevIt->end == start && prevIt->height == height) {
            start = prevIt->start;  // Extend the segment
            startIt = prevIt;       // Start from this segment
        }
    }
    
    // If there's a segment that starts at end with the same height, extend it
    if (endIt != segments.end() && endIt->start == end && endIt->height == height) {
        end = endIt->end;  // Extend the segment
        ++endIt;          // Include this segment in the range to remove
    }
    
    // Create a new segment with the extended range
    ContourSegment newSegment(start, end, height);
    
    // Replace the range of segments that overlap with the new segment
    // with the new segment itself
    segments.erase(startIt, endIt);
    segments.insert(startIt, newSegment);
    
    // No need to sort or merge - the segments are already in order and merged
}

/**
 * Gets the height of the contour at a specific range
 * Optimized for O(log n) query time
 */
int Contour::getHeight(int start, int end) const {
    if (start >= end) return 0;
    if (segments.empty()) return 0;
    
    // Find the first segment that might contain start
    auto it = std::lower_bound(segments.begin(), segments.end(),
                              ContourSegment(start, start, 0),
                              [](const ContourSegment& a, const ContourSegment& b) {
                                  return a.start < b.start;
                              });
    
    // If we found a segment that starts after start, check the previous segment
    if (it != segments.begin() && (it == segments.end() || it->start > start)) {
        --it;
        if (it->end <= start) {
            // This segment ends before start, move to the next one
            ++it;
        }
    }
    
    // Find maximum height in the range
    int maxHeight = 0;
    
    // Iterate through segments that intersect the given range
    while (it != segments.end() && it->start < end) {
        // Only consider the segment if it actually overlaps the range
        if (it->end > start) {
            maxHeight = std::max(maxHeight, it->height);
        }
        ++it;
    }
    
    return maxHeight;
}

/**
 * Gets all contour segments
 */
const std::vector<ContourSegment>& Contour::getSegments() const {
    return segments;
}

/**
 * Merges this contour with another contour
 */
void Contour::merge(const Contour& other) {
    // If other contour is empty, nothing to do
    if (other.segments.empty()) return;
    
    // If this contour is empty, just copy the other one
    if (segments.empty()) {
        segments = other.segments;
        maxCoordinate = other.maxCoordinate;
        maxHeight = other.maxHeight;
        return;
    }
    
    // Preallocate vector for all combined segments
    std::vector<ContourSegment> result;
    result.reserve(segments.size() + other.segments.size());
    
    // Merge the two sorted segment lists
    auto it1 = segments.begin();
    auto it2 = other.segments.begin();
    
    while (it1 != segments.end() && it2 != other.segments.end()) {
        // Choose the segment with earlier start
        if (it1->start < it2->start) {
            result.push_back(*it1);
            ++it1;
        } else {
            result.push_back(*it2);
            ++it2;
        }
    }
    
    // Add remaining segments
    while (it1 != segments.end()) {
        result.push_back(*it1);
        ++it1;
    }
    
    while (it2 != other.segments.end()) {
        result.push_back(*it2);
        ++it2;
    }
    
    // Replace segments with the merged list
    segments = std::move(result);
    
    // Update max values
    maxCoordinate = std::max(maxCoordinate, other.maxCoordinate);
    maxHeight = std::max(maxHeight, other.maxHeight);
    
    // Merge overlapping segments with the same height
    mergeSegments();
}

/**
 * Gets the maximum coordinate value in the contour
 */
int Contour::getMaxCoordinate() const {
    return maxCoordinate;
}

/**
 * Gets the maximum height value in the contour
 */
int Contour::getMaxHeight() const {
    return maxHeight;
}

/**
 * Checks if the contour is empty
 */
bool Contour::isEmpty() const {
    return segments.empty();
}