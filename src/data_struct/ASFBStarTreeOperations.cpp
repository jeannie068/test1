/**
 * ASFBStarTreeOperations.cpp
 * 
 * Implementation of packing, symmetry handling, and tree manipulation
 * operations for the ASFBStarTree class.
 */

#include "ASFBStarTree.hpp"
#include <algorithm>
#include <queue>
#include <iostream>
#include <limits>
#include <cmath>
using namespace std;

/**
 * Initialize contours for packing
 */
void ASFBStarTree::initializeContours() {
    horizontalContour->clear();
    verticalContour->clear();
    
    // Initialize horizontal contour with a segment at y=0
    horizontalContour->addSegment(0, numeric_limits<int>::max(), 0);
    
    // Initialize vertical contour with a segment at x=0
    verticalContour->addSegment(0, numeric_limits<int>::max(), 0);
}

/**
 * Update contour with a module
 */
void ASFBStarTree::updateContourWithModule(const shared_ptr<Module>& module) {
    if (!module) return;
    
    int x = module->getX();
    int y = module->getY();
    int width = module->getWidth();
    int height = module->getHeight();
    
    // Update horizontal contour
    horizontalContour->addSegment(x, x + width, y + height);
    
    // Update vertical contour
    verticalContour->addSegment(y, y + height, x + width);
}

/**
 * Pack a node in the ASF-B*-tree
 */
void ASFBStarTree::packNode(const shared_ptr<BStarTreeNode>& node) {
    if (!node) return;
    
    const string& moduleName = node->getModuleName();
    auto it = modules.find(moduleName);
    if (it == modules.end() || !it->second) {
        cerr << "Error: Module " << moduleName << " not found in modules map" << endl;
        return;
    }
    
    auto module = it->second;
    
    int x = 0, y = 0;
    
    // Calculate x-coordinate based on B*-tree rules
    if (node->getParent()) {
        auto parent = modules[node->getParent()->getModuleName()];
        if (!parent) {
            cerr << "Error: Parent module not found" << endl;
            return;
        }
        
        if (node->isLeftChild()) {
            // Left child: place to the right of parent
            x = parent->getX() + parent->getWidth();
        } else {
            // Right child: same x-coordinate as parent
            x = parent->getX();
        }
    }
    
    // Calculate y-coordinate using the horizontal contour
    y = horizontalContour->getHeight(x, x + module->getWidth());
    
    // Special handling for self-symmetric modules
    if (find(selfSymmetricModules.begin(), selfSymmetricModules.end(), moduleName) != selfSymmetricModules.end()) {
        // For vertical symmetry, center on the axis
        if (symmetryGroup->getType() == SymmetryType::VERTICAL) {
            // Calculate the center position
            x = static_cast<int>(symmetryAxisPosition - module->getWidth() / 2.0);
        }
        // For horizontal symmetry, center on the axis
        else {
            // Calculate the center position
            y = static_cast<int>(symmetryAxisPosition - module->getHeight() / 2.0);
        }
    }
    
    // Set the module's position
    module->setPosition(x, y);
    
    // Update contours
    updateContourWithModule(module);
}

/**
 * Calculate positions for self-symmetric modules
 */
void ASFBStarTree::calculateSelfSymmetricModulePositions() {
    // Process self-symmetric modules
    for (const auto& moduleName : selfSymmetricModules) {
        auto it = modules.find(moduleName);
        if (it == modules.end() || !it->second) continue;
        
        auto module = it->second;
        
        // For self-symmetric modules, make sure they are centered on the symmetry axis
        if (symmetryGroup->getType() == SymmetryType::VERTICAL) {
            // Center the module on the vertical symmetry axis
            int newX = static_cast<int>(symmetryAxisPosition - module->getWidth() / 2.0);
            module->setPosition(newX, module->getY());
        } else {
            // Center the module on the horizontal symmetry axis
            int newY = static_cast<int>(symmetryAxisPosition - module->getHeight() / 2.0);
            module->setPosition(module->getX(), newY);
        }
    }
}

/**
 * Mirror non-representative modules based on the representatives' positions
 */
void ASFBStarTree::mirrorNonRepresentativeModules() {
    if (symmetryGroup->getType() == SymmetryType::VERTICAL) {
        // For each symmetry pair, mirror the non-representative
        for (const auto& pair : symmetryGroup->getSymmetryPairs()) {
            // Find the representative module
            string repName = getRepresentative(pair.first);
            if (repName.empty()) continue;
            
            // Get the representative and non-representative modules
            string nonRepName = (pair.first == repName) ? pair.second : pair.first;
            
            auto repIt = modules.find(repName);
            auto nonRepIt = modules.find(nonRepName);
            
            if (repIt == modules.end() || nonRepIt == modules.end() || 
                !repIt->second || !nonRepIt->second) {
                continue;
            }
            
            auto rep = repIt->second;
            auto nonRep = nonRepIt->second;
            
            // Copy rotation status
            nonRep->setRotation(rep->getRotated());
            
            // Mirror position around the symmetry axis
            int repWidth = rep->getWidth();
            int repX = rep->getX();
            int repY = rep->getY();
            
            // Calculate the reflected x-coordinate
            double repCenterX = repX + repWidth / 2.0;
            double reflectedCenterX = 2 * symmetryAxisPosition - repCenterX;
            
            // Calculate new x-coordinate for non-representative
            int nonRepX = static_cast<int>(reflectedCenterX - nonRep->getWidth() / 2.0);
            
            // Place the non-representative module
            nonRep->setPosition(nonRepX, repY);
        }
    } else { // HORIZONTAL symmetry
        for (const auto& pair : symmetryGroup->getSymmetryPairs()) {
            // Find the representative module
            string repName = getRepresentative(pair.first);
            if (repName.empty()) continue;
            
            // Get the representative and non-representative modules
            string nonRepName = (pair.first == repName) ? pair.second : pair.first;
            
            auto repIt = modules.find(repName);
            auto nonRepIt = modules.find(nonRepName);
            
            if (repIt == modules.end() || nonRepIt == modules.end() || 
                !repIt->second || !nonRepIt->second) {
                continue;
            }
            
            auto rep = repIt->second;
            auto nonRep = nonRepIt->second;
            
            // Copy rotation status
            nonRep->setRotation(rep->getRotated());
            
            // Mirror position around the horizontal symmetry axis
            int repHeight = rep->getHeight();
            int repX = rep->getX();
            int repY = rep->getY();
            
            // Calculate the reflected y-coordinate
            double repCenterY = repY + repHeight / 2.0;
            double reflectedCenterY = 2 * symmetryAxisPosition - repCenterY;
            
            // Calculate new y-coordinate for non-representative
            int nonRepY = static_cast<int>(reflectedCenterY - nonRep->getHeight() / 2.0);
            
            // Place the non-representative module
            nonRep->setPosition(repX, nonRepY);
        }
    }
}

/**
 * Repack only the modified nodes and their subtrees
 */
void ASFBStarTree::repackModifiedNodes() {
    if (modifiedNodes.empty()) return;
    
    // Initialize contours
    initializeContours();
    
    // Sort modified nodes by depth (deepest first)
    std::vector<shared_ptr<BStarTreeNode>> sortedNodes(modifiedNodes.begin(), modifiedNodes.end());
    std::sort(sortedNodes.begin(), sortedNodes.end(), 
             [](const shared_ptr<BStarTreeNode>& a, const shared_ptr<BStarTreeNode>& b) {
                 int depthA = 0, depthB = 0;
                 auto currA = a, currB = b;
                 
                 while (currA->getParent()) {
                     depthA++;
                     currA = currA->getParent();
                 }
                 
                 while (currB->getParent()) {
                     depthB++;
                     currB = currB->getParent();
                 }
                 
                 return depthA > depthB;
             });
    
    // Pack each modified node
    for (const auto& node : sortedNodes) {
        packNode(node);
    }
    
    // Apply special positioning for self-symmetric modules
    calculateSelfSymmetricModulePositions();
    
    // Mirror non-representative modules
    mirrorNonRepresentativeModules();
    
    // Clear modified nodes
    modifiedNodes.clear();
}

/**
 * Calculates the coordinates of all modules in the symmetry group
 * by packing the ASF-B*-tree
 */
bool ASFBStarTree::pack() {
    if (!root) return false;
    
    // If there are modified nodes, only repack those
    if (!modifiedNodes.empty()) {
        repackModifiedNodes();
        return true;
    }
    
    // Initialize contours
    initializeContours();
    
    // Traverse the tree in pre-order and pack each node
    queue<shared_ptr<BStarTreeNode>> nodeQueue;
    nodeQueue.push(root);
    
    while (!nodeQueue.empty()) {
        auto currentNode = nodeQueue.front();
        nodeQueue.pop();
        
        if (!currentNode) continue;
        
        // Pack the current node
        packNode(currentNode);
        
        // Add children to the queue
        if (currentNode->getLeftChild()) {
            nodeQueue.push(currentNode->getLeftChild());
        }
        if (currentNode->getRightChild()) {
            nodeQueue.push(currentNode->getRightChild());
        }
    }
    
    // Apply special positioning for self-symmetric modules
    calculateSelfSymmetricModulePositions();
    
    // Mirror non-representative modules
    mirrorNonRepresentativeModules();
    
    return true;
}

/**
 * Checks if the tree satisfies the symmetric-feasible condition
 */
bool ASFBStarTree::isSymmetricFeasible() const {
    // Check self-symmetric modules
    for (const auto& moduleName : selfSymmetricModules) {
        auto node = findNode(moduleName);
        if (!node) continue;
        
        // For vertical symmetry, self-symmetric modules must be on the rightmost branch
        if (symmetryGroup->getType() == SymmetryType::VERTICAL) {
            // Traverse up the tree from the node
            auto current = node;
            while (current && current->getParent()) {
                // If this node is a left child of its parent, it's not on the rightmost branch
                if (current->getParent()->getLeftChild() == current) {
                    return false;
                }
                current = current->getParent();
            }
        }
        // For horizontal symmetry, self-symmetric modules must be on the leftmost branch
        else {
            // Traverse up the tree from the node
            auto current = node;
            while (current && current->getParent()) {
                // If this node is a right child of its parent, it's not on the leftmost branch
                if (current->getParent()->getRightChild() == current) {
                    return false;
                }
                current = current->getParent();
            }
        }
    }
    
    return true;
}

/**
 * Rotates a module in the symmetry group
 */
bool ASFBStarTree::rotateModule(const string& moduleName) {
    // Check if the module exists
    auto it = modules.find(moduleName);
    if (it == modules.end() || !it->second) {
        cerr << "Error: Module " << moduleName << " not found" << endl;
        return false;
    }
    
    // Check if this is a representative module
    if (!isRepresentative(moduleName)) {
        cerr << "Error: Can only rotate representative modules" << endl;
        return false;
    }
    
    // Get the representative module
    auto module = it->second;
    
    // For rotation, just swap width and height; don't modify coordinates
    // The coordinates will be recalculated during packing
    module->rotate();
    
    // Mark the module's node for repacking
    auto node = findNode(moduleName);
    if (node) {
        markNodeForRepack(node);
    }
    
    return true;
}

/**
 * Moves a node to a new position in the tree
 */
bool ASFBStarTree::moveNode(const string& nodeName, 
                           const string& newParentName, 
                           bool asLeftChild) {
    // Check if the nodes are representatives
    if (!isRepresentative(nodeName) || !isRepresentative(newParentName)) {
        cerr << "Error: Can only move representative nodes" << endl;
        return false;
    }
    
    // Use direct lookup instead of traversing the tree
    auto node = findNode(nodeName);
    auto newParent = findNode(newParentName);
    
    if (!node || !newParent) {
        cerr << "Error: Node or parent not found" << endl;
        return false;
    }
    
    // Check if the move is valid
    if (!canMoveNode(node, newParent, asLeftChild)) {
        cerr << "Error: Invalid move - would violate symmetry constraints" << endl;
        return false;
    }
    
    // Remove the node from its current parent
    auto oldParent = node->getParent();
    if (oldParent) {
        if (oldParent->getLeftChild() == node) {
            oldParent->setLeftChild(nullptr);
        } else if (oldParent->getRightChild() == node) {
            oldParent->setRightChild(nullptr);
        }
    }
    
    // Add the node to its new parent
    node->setParent(newParent);
    if (asLeftChild) {
        // If there's already a left child, handle it
        auto existingChild = newParent->getLeftChild();
        if (existingChild) {
            // Try to find a place for the existing child
            node->setLeftChild(existingChild);
            existingChild->setParent(node);
        }
        newParent->setLeftChild(node);
    } else {
        // If there's already a right child, handle it
        auto existingChild = newParent->getRightChild();
        if (existingChild) {
            // Try to find a place for the existing child
            node->setRightChild(existingChild);
            existingChild->setParent(node);
        }
        newParent->setRightChild(node);
    }
    
    markNodeForRepack(node);
    markNodeForRepack(newParent);
    if (oldParent) {
        markNodeForRepack(oldParent);
    }
    
    return true;
}

/**
 * Swaps two nodes in the tree
 */
bool ASFBStarTree::swapNodes(const string& nodeName1, const string& nodeName2) {
    // Check if the nodes are representatives
    if (!isRepresentative(nodeName1) || !isRepresentative(nodeName2)) {
        cerr << "Error: Can only swap representative nodes" << endl;
        return false;
    }
    
    // Use direct lookup instead of traversing the tree
    auto node1 = findNode(nodeName1);
    auto node2 = findNode(nodeName2);
    
    if (!node1 || !node2) {
        cerr << "Error: Node not found" << endl;
        return false;
    }
    
    // Check if both nodes can be swapped
    // Self-symmetric modules have special restrictions
    bool node1OnBoundary = isOnBoundary(node1->getModuleName());
    bool node2OnBoundary = isOnBoundary(node2->getModuleName());
    
    // If either node is a self-symmetric module, they must stay on the boundary
    if (node1OnBoundary || node2OnBoundary) {
        // If both are on the boundary, we can swap them
        if (node1OnBoundary && node2OnBoundary) {
            // Swap is allowed
        } else {
            // One is on the boundary, one is not - swap not allowed
            cerr << "Error: Cannot swap self-symmetric and non-self-symmetric modules" << endl;
            return false;
        }
    }
    
    // Swap the module names instead of the nodes themselves
    node1->swapModuleName(node2);
    
    // Mark both nodes for repacking
    markNodeForRepack(node1);
    markNodeForRepack(node2);
    
    return true;
}

/**
 * Changes the representative of a symmetry pair
 */
bool ASFBStarTree::changeRepresentative(const string& moduleName) {
    // Find the symmetry pair
    auto pairIt = symmetricPairMap.find(moduleName);
    if (pairIt == symmetricPairMap.end()) {
        cerr << "Error: Module " << moduleName << " is not part of a symmetry pair" << endl;
        return false;
    }
    
    string module1 = moduleName;
    string module2 = pairIt->second;
    
    // Determine current representative
    string currentRep = getRepresentative(module1);
    if (currentRep.empty()) {
        cerr << "Error: No representative found for " << module1 << endl;
        return false;
    }
    
    // Determine which module should become the new representative
    string newRep = (currentRep == module1) ? module2 : module1;
    string oldRep = currentRep;
    
    // Update the representative map
    representativeMap[module1] = newRep;
    representativeMap[module2] = newRep;
    
    // Update representative sets
    representativeModules.erase(oldRep);
    representativeModules.insert(newRep);
    nonRepresentativeModules.erase(newRep);
    nonRepresentativeModules.insert(oldRep);
    
    // Rebuild the tree with the new representative
    constructInitialTree();
    
    return true;
}

/**
 * Converts the symmetry type (vertical to horizontal or vice versa)
 */
bool ASFBStarTree::convertSymmetryType() {
    if (!symmetryGroup) return false;
    
    // Change the symmetry type
    if (symmetryGroup->getType() == SymmetryType::VERTICAL) {
        symmetryGroup->setType(SymmetryType::HORIZONTAL);
    } else {
        symmetryGroup->setType(SymmetryType::VERTICAL);
    }
    
    // Reset the axis lock
    axisPositionLocked = false;
    
    // Re-lock the axis with the new type
    lockSymmetryAxis();
    
    // Rotate all modules
    for (auto& pair : modules) {
        pair.second->rotate();
    }
    
    // Rebuild the tree with the new symmetry type
    constructInitialTree();
    
    return true;
}