/**
 * Contour.cpp
 * 
 * Implementation of the Contour class for efficient packing in the ASF-B*-tree
 * placement algorithm. The contour data structure represents the skyline profile
 * of the currently placed modules, allowing for efficient height queries and updates.
 * Optimized implementation using doubly-linked list.
 */

#include "Contour.hpp"
#include <algorithm>
#include <cassert>

/**
 * Constructor
 */
Contour::Contour() : head(nullptr), tail(nullptr), maxCoordinate(0), maxHeight(0) {
    // Initialize with empty contour
}

/**
 * Copy constructor
 */
Contour::Contour(const Contour& other) : head(nullptr), tail(nullptr), maxCoordinate(other.maxCoordinate), maxHeight(other.maxHeight) {
    // Deep copy of the linked list
    ContourNode* current = other.head;
    while (current) {
        addSegment(current->start, current->end, current->height);
        current = current->next;
    }
}

/**
 * Destructor
 */
Contour::~Contour() {
    clear();
}

/**
 * Clears the contour
 */
void Contour::clear() {
    // Delete all nodes in the linked list
    ContourNode* current = head;
    while (current) {
        ContourNode* next = current->next;
        delete current;
        current = next;
    }
    head = nullptr;
    tail = nullptr;
    maxCoordinate = 0;
    maxHeight = 0;
}

/**
 * Finds the node containing a coordinate
 */
ContourNode* Contour::findNode(int coordinate) const {
    ContourNode* current = head;
    while (current) {
        if (current->start <= coordinate && coordinate < current->end) {
            return current;
        }
        current = current->next;
    }
    return nullptr;
}

/**
 * Merges adjacent nodes with the same height
 */
void Contour::mergeNodes() {
    if (!head) return;
    
    ContourNode* current = head;
    while (current && current->next) {
        // If the current node's end equals the next node's start and they have the same height
        if (current->end == current->next->start && current->height == current->next->height) {
            // Extend the current node's end to the next node's end
            current->end = current->next->end;
            
            // Remove the next node
            ContourNode* nextNode = current->next;
            current->next = nextNode->next;
            
            if (nextNode->next) nextNode->next->prev = current;
            else tail = current;
            
            delete nextNode;
            
            // Continue checking for more merges without advancing current
            continue;
        }
        
        current = current->next;
    }
}

/**
 * Adds a segment to the contour
 */
void Contour::addSegment(int start, int end, int height) {
    if (start >= end) return;  // Invalid segment
    
    // Update maximum values
    maxCoordinate = std::max(maxCoordinate, end);
    maxHeight = std::max(maxHeight, height);
    
    // Special case: empty contour
    if (!head) {
        head = new ContourNode(start, end, height);
        tail = head;
        return;
    }
    
    // Find the insertion point and handle overlapping segments
    
    // Find the first node that overlaps with the new segment
    ContourNode* current = head;
    
    // Skip nodes that end before the new segment starts
    while (current && current->end <= start) {
        current = current->next;
    }
    
    // No overlap - insert at the end or between nodes
    if (!current) {
        // Check if we can extend the tail
        if (tail && tail->end == start && tail->height == height) {
            tail->end = end;
        } else {
            // Insert a new node at the end
            ContourNode* newNode = new ContourNode(start, end, height);
            newNode->prev = tail;
            if (tail) tail->next = newNode;
            else head = newNode;
            tail = newNode;
        }
        return;
    }
    
    // If the new segment starts before the current node
    if (start < current->start) {
        // Check if we can extend the previous node
        if (current->prev && current->prev->end == start && current->prev->height == height) {
            // Extend previous node up to the current node
            current->prev->end = std::min(end, current->start);
            
            // If the new segment extends beyond the current node's start, continue processing
            if (end > current->start) {
                start = current->start;
            } else {
                return;  // Done
            }
        } else {
            // Insert a new node before the current one
            ContourNode* newNode = new ContourNode(start, std::min(end, current->start), height);
            newNode->next = current;
            newNode->prev = current->prev;
            if (current->prev) current->prev->next = newNode;
            else head = newNode;
            current->prev = newNode;
            
            // If the new segment ends before the current node starts, we're done
            if (end <= current->start) {
                return;
            }
            
            // Otherwise, continue processing from the current node's start
            start = current->start;
        }
    }
    
    // Process overlapping nodes
    while (current && start < end) {
        // If the new segment completely covers the current node
        if (end >= current->end) {
            current->height = height;  // Update height
            start = current->end;      // Move start to the end of this node
            current = current->next;   // Move to next node
        } else {
            // The new segment ends within the current node - split the node
            ContourNode* newNode = new ContourNode(end, current->end, current->height);
            newNode->next = current->next;
            if (current->next) current->next->prev = newNode;
            else tail = newNode;
            
            current->next = newNode;
            newNode->prev = current;
            
            current->end = end;
            current->height = height;
            
            break;  // Done
        }
    }
    
    // Merge adjacent nodes with the same height
    mergeNodes();
}

/**
 * Gets the height of the contour at a specific range
 */
int Contour::getHeight(int start, int end) const {
    if (start >= end) return 0;
    if (!head) return 0;
    
    // Find the maximum height in the range
    int maxHeight = 0;
    
    // Find the first node that might contain or come after start
    ContourNode* current = head;
    while (current && current->end <= start) {
        current = current->next;
    }
    
    // Iterate through nodes that intersect with the given range
    while (current && current->start < end) {
        // Only consider the node if it actually overlaps the range
        if (current->end > start) {
            maxHeight = std::max(maxHeight, current->height);
        }
        current = current->next;
    }
    
    return maxHeight;
}

/**
 * Gets all contour segments
 */
std::vector<ContourSegment> Contour::getSegments() const {
    std::vector<ContourSegment> segments;
    
    ContourNode* current = head;
    while (current) {
        segments.push_back(ContourSegment(current->start, current->end, current->height));
        current = current->next;
    }
    
    return segments;
}

/**
 * Merges this contour with another contour
 */
void Contour::merge(const Contour& other) {
    // If other contour is empty, nothing to do
    if (!other.head) return;
    
    // If this contour is empty, just copy the other one
    if (!head) {
        *this = other;
        return;
    }
    
    // Merge the two contours by adding each segment from the other contour
    ContourNode* current = other.head;
    while (current) {
        addSegment(current->start, current->end, current->height);
        current = current->next;
    }
    
    // Update maximum values
    maxCoordinate = std::max(maxCoordinate, other.maxCoordinate);
    maxHeight = std::max(maxHeight, other.maxHeight);
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
    return head == nullptr;
}