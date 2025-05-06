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
    auto module = modules[moduleName];
    
    if (!module) return;
    
    int x = 0, y = 0;
    
    // Calculate x-coordinate based on B*-tree rules
    if (node->getParent()) {
        auto parent = modules[node->getParent()->getModuleName()];
        if (parent) {
            if (node->getParent()->getLeftChild() == node) {
                // Left child: place to the right of parent
                x = parent->getX() + parent->getWidth();
            } else {
                // Right child: same x-coordinate as parent
                x = parent->getX();
            }
        }
    }
    
    // Calculate y-coordinate using the horizontal contour
    y = horizontalContour->getHeight(x, x + module->getWidth());
    
    // Special handling for self-symmetric modules
    if (find(selfSymmetricModules.begin(), selfSymmetricModules.end(), 
                 moduleName) != selfSymmetricModules.end()) {
        // Self-symmetric modules must be on the boundary
        if (symmetryGroup->getType() == SymmetryType::VERTICAL) {
            // For vertical symmetry, the module must abut the symmetry axis
            x = static_cast<int>(symmetryAxisPosition) - module->getWidth() / 2;
        } else {
            // For horizontal symmetry, the module must be on the bottom
            y = static_cast<int>(symmetryAxisPosition) - module->getHeight() / 2;
        }
    }
    
    // Set the module's position
    module->setPosition(x, y);
    
    // Update contours
    updateContourWithModule(module);
}

/**
 * Calculate the positions of symmetric modules
 */
void ASFBStarTree::calculateSymmetricModulePositions() {
    // Process symmetry pairs
    for (const auto& pair : symmetryGroup->getSymmetryPairs()) {
        const string& module1 = pair.first;
        const string& module2 = pair.second;
        
        auto mod1 = modules[module1];
        auto mod2 = modules[module2];
        
        if (!mod1 || !mod2) continue;
        
        // Determine which module is the representative
        string representative = getRepresentative(module1);
        
        if (representative == module1) {
            // module1 is the representative, calculate position of module2
            if (symmetryGroup->getType() == SymmetryType::VERTICAL) {
                // For vertical symmetry, reflect center about the symmetry axis
                double center1_x = mod1->getX() + mod1->getWidth() / 2.0;
                double reflectedCenter_x = 2 * symmetryAxisPosition - center1_x;
                // Convert back to left edge
                int reflectedX = static_cast<int>(reflectedCenter_x - mod2->getWidth() / 2.0);
                mod2->setPosition(reflectedX, mod1->getY());
            } else {
                // For horizontal symmetry, reflect center about the symmetry axis
                double center1_y = mod1->getY() + mod1->getHeight() / 2.0;
                double reflectedCenter_y = 2 * symmetryAxisPosition - center1_y;
                // Convert back to bottom edge
                int reflectedY = static_cast<int>(reflectedCenter_y - mod2->getHeight() / 2.0);
                mod2->setPosition(mod1->getX(), reflectedY);
            }
        } else if (representative == module2) {
            // module2 is the representative, calculate position of module1
            if (symmetryGroup->getType() == SymmetryType::VERTICAL) {
                // For vertical symmetry, reflect center about the symmetry axis
                double center2_x = mod2->getX() + mod2->getWidth() / 2.0;
                double reflectedCenter_x = 2 * symmetryAxisPosition - center2_x;
                // Convert back to left edge
                int reflectedX = static_cast<int>(reflectedCenter_x - mod1->getWidth() / 2.0);
                mod1->setPosition(reflectedX, mod2->getY());
            } else {
                // For horizontal symmetry, reflect center about the symmetry axis
                double center2_y = mod2->getY() + mod2->getHeight() / 2.0;
                double reflectedCenter_y = 2 * symmetryAxisPosition - center2_y;
                // Convert back to bottom edge
                int reflectedY = static_cast<int>(reflectedCenter_y - mod1->getHeight() / 2.0);
                mod1->setPosition(mod2->getX(), reflectedY);
            }
        }
    }
    
    // Process self-symmetric modules
    for (const auto& moduleName : selfSymmetricModules) {
        auto module = modules[moduleName];
        if (module) {
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
    
    // Set the symmetry axis position
    if (symmetryGroup->getType() == SymmetryType::VERTICAL) {
        // For vertical symmetry, calculate proper axis position
        double sumX = 0.0;
        int count = 0;
        for (const auto& pair : modules) {
            if (isRepresentative(pair.first)) {
                double centerX = pair.second->getX() + pair.second->getWidth() / 2.0;
                sumX += centerX;
                count++;
            }
        }
        symmetryAxisPosition = (count > 0) ? (sumX / count) : 0.0;
    } else {
        // For horizontal symmetry, calculate proper axis position
        double sumY = 0.0;
        int count = 0;
        for (const auto& pair : modules) {
            if (isRepresentative(pair.first)) {
                double centerY = pair.second->getY() + pair.second->getHeight() / 2.0;
                sumY += centerY;
                count++;
            }
        }
        symmetryAxisPosition = (count > 0) ? (sumY / count) : 0.0;
    }
    
    // Calculate positions for symmetric modules
    calculateSymmetricModulePositions();
    
    return true;
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
    
    // Calculate positions for symmetric modules
    calculateSymmetricModulePositions();
    
    // Clear modified nodes
    modifiedNodes.clear();
}

/**
 * Checks if the tree satisfies the symmetric-feasible condition
 */
bool ASFBStarTree::isSymmetricFeasible() const {
    // Check self-symmetric modules
    for (const auto& moduleName : selfSymmetricModules) {
        auto node = [this, &moduleName]() -> shared_ptr<BStarTreeNode> {
            // Find the node for this module
            queue<shared_ptr<BStarTreeNode>> queue;
            if (root) queue.push(root);
            
            while (!queue.empty()) {
                auto current = queue.front();
                queue.pop();
                
                if (current->getModuleName() == moduleName) {
                    return current;
                }
                
                if (current->getLeftChild()) queue.push(current->getLeftChild());
                if (current->getRightChild()) queue.push(current->getRightChild());
            }
            
            return nullptr;
        }();
        
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
    auto it = modules.find(moduleName);
    if (it == modules.end()) return false;
    
    auto module = it->second;
    
    // Special handling for symmetry pairs and self-symmetric modules
    auto pairIt = symmetricPairMap.find(moduleName);
    auto selfIt = find(selfSymmetricModules.begin(), selfSymmetricModules.end(), moduleName);
    
    if (pairIt != symmetricPairMap.end()) {
        // For symmetry pairs, rotate both modules
        auto pairModule = modules[pairIt->second];
        if (pairModule) {
            pairModule->rotate();
        }
    } else if (selfIt != selfSymmetricModules.end()) {
        // For self-symmetric modules, update the shape of its representative
        // This is a simplification - in reality, more complex handling might be needed
    }
    
    // Rotate the module
    module->rotate();
    
    return true;
}

/**
 * Moves a node to a new position in the tree
 */
bool ASFBStarTree::moveNode(const string& nodeName, 
                           const string& newParentName, 
                           bool asLeftChild) {
    // Use direct lookup instead of traversing the tree
    auto node = findNode(nodeName);
    auto newParent = findNode(newParentName);
    
    if (!node || !newParent) return false;
    
    // Check if the move is valid
    if (!canMoveNode(node, newParent, asLeftChild)) {
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
            // This is a simplification - in practice, more sophisticated handling is needed
            node->setLeftChild(existingChild);
            existingChild->setParent(node);
        }
        newParent->setLeftChild(node);
    } else {
        // If there's already a right child, handle it
        auto existingChild = newParent->getRightChild();
        if (existingChild) {
            // Try to find a place for the existing child
            // This is a simplification - in practice, more sophisticated handling is needed
            node->setRightChild(existingChild);
            existingChild->setParent(node);
        }
        newParent->setRightChild(node);
    }
    
    markNodeForRepack(node);
    markNodeForRepack(newParent);
    if (node->getParent()) {
        markNodeForRepack(node->getParent());
    }
    
    // Pack the modified nodes
    repackModifiedNodes();
    
    return true;
}

/**
 * Swaps two nodes in the tree
 */
bool ASFBStarTree::swapNodes(const string& nodeName1, const string& nodeName2) {
    // Use direct lookup instead of traversing the tree
    auto node1 = findNode(nodeName1);
    auto node2 = findNode(nodeName2);
    
    if (!node1 || !node2) return false;
    
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
            return false;
        }
    }
    
    // Instead of setting module names (which we can't do), swap the parent-child relationships
    // Get parents and positions
    auto parent1 = node1->getParent();
    auto parent2 = node2->getParent();
    
    bool isLeftChild1 = parent1 && parent1->getLeftChild() == node1;
    bool isLeftChild2 = parent2 && parent2->getLeftChild() == node2;
    
    // Detach nodes from parents
    if (parent1) {
        if (isLeftChild1) parent1->setLeftChild(nullptr);
        else parent1->setRightChild(nullptr);
    }
    
    if (parent2) {
        if (isLeftChild2) parent2->setLeftChild(nullptr);
        else parent2->setRightChild(nullptr);
    }
    
    // Reattach nodes to opposite parents
    if (parent1) {
        if (isLeftChild1) parent1->setLeftChild(node2);
        else parent1->setRightChild(node2);
        node2->setParent(parent1);
    } else {
        // node1 was the root
        root = node2;
        node2->setParent(nullptr);
    }
    
    if (parent2) {
        if (isLeftChild2) parent2->setLeftChild(node1);
        else parent2->setRightChild(node1);
        node1->setParent(parent2);
    } else {
        // node2 was the root
        root = node1;
        node1->setParent(nullptr);
    }
    
    // Swap children too
    auto leftChild1 = node1->getLeftChild();
    auto rightChild1 = node1->getRightChild();
    auto leftChild2 = node2->getLeftChild();
    auto rightChild2 = node2->getRightChild();
    
    // Set children for node1
    node1->setLeftChild(leftChild2);
    node1->setRightChild(rightChild2);
    if (leftChild2) leftChild2->setParent(node1);
    if (rightChild2) rightChild2->setParent(node1);
    
    // Set children for node2
    node2->setLeftChild(leftChild1);
    node2->setRightChild(rightChild1);
    if (leftChild1) leftChild1->setParent(node2);
    if (rightChild1) rightChild1->setParent(node2);
    
    // Mark affected nodes for repacking
    markNodeForRepack(node1);
    markNodeForRepack(node2);
    if (node1->getParent()) {
        markNodeForRepack(node1->getParent());
    }
    if (node2->getParent()) {
        markNodeForRepack(node2->getParent());
    }
    
    // Pack the modified nodes
    repackModifiedNodes();
    
    return true;
}

/**
 * Changes the representative of a symmetry pair
 */
bool ASFBStarTree::changeRepresentative(const string& moduleName) {
    // Find the symmetry pair
    auto pairIt = symmetricPairMap.find(moduleName);
    if (pairIt == symmetricPairMap.end()) return false;
    
    string module1 = moduleName;
    string module2 = pairIt->second;
    
    // Update the representative map
    if (representativeMap[module1] == module2) {
        // Change representative from module2 to module1
        representativeMap[module1] = module1;
        representativeMap[module2] = module1;
    } else {
        // Change representative from module1 to module2
        representativeMap[module1] = module2;
        representativeMap[module2] = module2;
    }
    
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
    
    // Rotate all modules
    for (auto& pair : modules) {
        pair.second->rotate();
    }
    
    // Rebuild the tree with the new symmetry type
    constructInitialTree();
    
    return true;
}