/**
 * ASFBStarTreeCore.cpp
 * 
 * Core functionality of the ASFBStarTree class including initialization,
 * node management, and utility functions.
 */

#include "ASFBStarTree.hpp"
#include <algorithm>
#include <queue>
#include <iostream>
#include <limits>
#include <cmath>
using namespace std;

/**
 * Constructor
 */
ASFBStarTree::ASFBStarTree(shared_ptr<SymmetryGroup> symGroup)
    : root(nullptr),
      symmetryGroup(symGroup),
      horizontalContour(make_shared<Contour>()),
      verticalContour(make_shared<Contour>()),
      symmetryAxisPosition(0.0) {
    
    // Initialize representative and symmetry pair maps
    if (symmetryGroup) {
        // Process symmetry pairs
        for (const auto& pair : symmetryGroup->getSymmetryPairs()) {
            // For each symmetry pair, choose the second module as the representative
            representativeMap[pair.first] = pair.second;
            representativeMap[pair.second] = pair.second;  // Representative represents itself
            
            // Track the symmetric pair relationship
            symmetricPairMap[pair.first] = pair.second;
            symmetricPairMap[pair.second] = pair.first;
        }
        
        // Process self-symmetric modules
        for (const auto& moduleName : symmetryGroup->getSelfSymmetric()) {
            // For a self-symmetric module, it represents itself but half of it
            representativeMap[moduleName] = moduleName;
            selfSymmetricModules.push_back(moduleName);
        }
    }
}

ASFBStarTree::~ASFBStarTree() {
    // Clear node map
    nodeMap.clear();
}

/**
 * Adds a module to the tree
 */
void ASFBStarTree::addModule(shared_ptr<Module> module) {
    if (!module) return;
    
    // Add the module to our module map
    modules[module->getName()] = module;
}

/**
 * Constructs an initial ASF-B*-tree based on the symmetry group
 */
void ASFBStarTree::constructInitialTree() {
    // Clear any existing tree and node map
    root = nullptr;
    nodeMap.clear();
    
    // First, collect all representatives
    vector<string> representatives;
    for (const auto& pair : modules) {
        const string& moduleName = pair.first;
        if (isRepresentative(moduleName)) {
            representatives.push_back(moduleName);
        }
    }
    
    if (representatives.empty()) return;
    
    // Sort representatives by area (largest first)
    sort(representatives.begin(), representatives.end(), 
              [this](const string& a, const string& b) {
                  return modules[a]->getArea() > modules[b]->getArea();
              });
    
    // Create the root node with the first representative
    root = make_shared<BStarTreeNode>(representatives[0]);
    
    // Register the root node in the lookup map
    registerNodeInMap(root);
    
    // Add other representatives to the tree
    for (size_t i = 1; i < representatives.size(); ++i) {
        auto newNode = make_shared<BStarTreeNode>(representatives[i]);
        
        // Case: self-symmetric module - it must be on the boundary
        if (find(selfSymmetricModules.begin(), selfSymmetricModules.end(), 
                      representatives[i]) != selfSymmetricModules.end()) {
            
            // Place self-symmetric modules on the rightmost branch for vertical symmetry
            // or leftmost branch for horizontal symmetry
            if (symmetryGroup->getType() == SymmetryType::VERTICAL) {
                // Find the rightmost node
                auto current = root;
                while (current->getRightChild()) {
                    current = current->getRightChild();
                }
                current->setRightChild(newNode);
                newNode->setParent(current);
            } else {
                // Find the leftmost node
                auto current = root;
                while (current->getLeftChild()) {
                    current = current->getLeftChild();
                }
                current->setLeftChild(newNode);
                newNode->setParent(current);
            }
        } else {
            // Case: symmetry pairs - can place anywhere
            // For simplicity, place them as right children of the rightmost node
            auto current = root;
            while (current->getRightChild()) {
                current = current->getRightChild();
            }
            current->setRightChild(newNode);
            newNode->setParent(current);
        }
        
        // Register the new node in the lookup map
        registerNodeInMap(newNode);
    }
}

/**
 * Returns the bounding rectangle area of the symmetry island
 */
int ASFBStarTree::getArea() const {
    if (modules.empty()) return 0;
    
    int minX = numeric_limits<int>::max();
    int minY = numeric_limits<int>::max();
    int maxX = 0;
    int maxY = 0;
    
    // Find the bounding rectangle
    for (const auto& pair : modules) {
        const auto& module = pair.second;
        
        minX = min(minX, module->getX());
        minY = min(minY, module->getY());
        maxX = max(maxX, module->getX() + module->getWidth());
        maxY = max(maxY, module->getY() + module->getHeight());
    }
    
    return (maxX - minX) * (maxY - minY);
}

/**
 * Gets the contour of the symmetry island (for HB*-tree)
 */
pair<shared_ptr<Contour>, shared_ptr<Contour>> ASFBStarTree::getContours() const {
    return {horizontalContour, verticalContour};
}

/**
 * Helper function: checks if a module is on the boundary
 */
bool ASFBStarTree::isOnBoundary(const string& moduleName) const {
    return find(selfSymmetricModules.begin(), selfSymmetricModules.end(), moduleName) != selfSymmetricModules.end();
}

/**
 * Helper function: checks if a node can be moved to a new position
 */
bool ASFBStarTree::canMoveNode(const shared_ptr<BStarTreeNode>& node, 
                              const shared_ptr<BStarTreeNode>& newParent, 
                              bool asLeftChild) const {
    if (!node || !newParent) return false;
    
    // Self-symmetric modules have placement restrictions
    if (isOnBoundary(node->getModuleName())) {
        // For vertical symmetry, self-symmetric modules must be on the rightmost branch
        if (symmetryGroup->getType() == SymmetryType::VERTICAL) {
            if (asLeftChild) return false;  // Can't be a left child
            
            // Check if the new parent is on the rightmost branch
            auto current = newParent;
            while (current && current->getParent()) {
                if (current->getParent()->getLeftChild() == current) {
                    return false;  // Not on the rightmost branch
                }
                current = current->getParent();
            }
        }
        // For horizontal symmetry, self-symmetric modules must be on the leftmost branch
        else {
            if (!asLeftChild) return false;  // Can't be a right child
            
            // Check if the new parent is on the leftmost branch
            auto current = newParent;
            while (current && current->getParent()) {
                if (current->getParent()->getRightChild() == current) {
                    return false;  // Not on the leftmost branch
                }
                current = current->getParent();
            }
        }
    }
    
    return true;
}

// Direct node lookup by name
shared_ptr<BStarTreeNode> ASFBStarTree::findNode(const string& nodeName) const {
    auto it = nodeMap.find(nodeName);
    if (it != nodeMap.end()) {
        return it->second;
    }
    
    // If not found in the map, traverse the tree
    // This should rarely happen after initialization
    std::queue<shared_ptr<BStarTreeNode>> queue;
    if (root) queue.push(root);
    
    while (!queue.empty()) {
        auto current = queue.front();
        queue.pop();
        
        if (current->getModuleName() == nodeName) {
            return current;
        }
        
        if (current->getLeftChild()) queue.push(current->getLeftChild());
        if (current->getRightChild()) queue.push(current->getRightChild());
    }
    
    return nullptr;
}

// Register a node and its children in the lookup map
void ASFBStarTree::registerNodeInMap(shared_ptr<BStarTreeNode> node) {
    if (!node) return;
    
    // Add to node map
    nodeMap[node->getModuleName()] = node;
    
    // Recursively register children
    if (node->getLeftChild()) {
        registerNodeInMap(node->getLeftChild());
    }
    if (node->getRightChild()) {
        registerNodeInMap(node->getRightChild());
    }
}

// Unregister a node and its children from the lookup map
void ASFBStarTree::unregisterNodeFromMap(shared_ptr<BStarTreeNode> node) {
    if (!node) return;
    
    // Remove from node map
    nodeMap.erase(node->getModuleName());
    
    // Recursively unregister children
    if (node->getLeftChild()) {
        unregisterNodeFromMap(node->getLeftChild());
    }
    if (node->getRightChild()) {
        unregisterNodeFromMap(node->getRightChild());
    }
}

// Mark a node for repacking
void ASFBStarTree::markNodeForRepack(shared_ptr<BStarTreeNode> node) {
    if (!node) return;
    
    modifiedNodes.insert(node);
}

shared_ptr<BStarTreeNode> ASFBStarTree::getRoot() const {
    return root;
}

/**
 * Gets all modules in the symmetry group
 */
const map<string, shared_ptr<Module>>& ASFBStarTree::getModules() const {
    return modules;
}

shared_ptr<SymmetryGroup> ASFBStarTree::getSymmetryGroup() const {
    return symmetryGroup;
}

double ASFBStarTree::getSymmetryAxisPosition() const {
    return symmetryAxisPosition;
}

string ASFBStarTree::getRepresentative(const string& moduleName) const {
    auto it = representativeMap.find(moduleName);
    if (it == representativeMap.end()) return "";
    
    return it->second;
}

/**
 * Checks if a module is a representative
 */
bool ASFBStarTree::isRepresentative(const string& moduleName) const {
    auto it = representativeMap.find(moduleName);
    if (it == representativeMap.end()) return false;
    
    // A module is a representative if it represents itself
    return it->second == moduleName;
}

/**
 * Creates a deep copy of this ASF-B*-tree
 */
shared_ptr<ASFBStarTree> ASFBStarTree::clone() const {
    auto clone = std::make_shared<ASFBStarTree>(symmetryGroup);
    
    // Copy modules
    for (const auto& pair : modules) {
        auto moduleCopy = make_shared<Module>(*pair.second);
        clone->modules[pair.first] = moduleCopy;
    }
    
    // Copy representative map and symmetry pair map
    clone->representativeMap = representativeMap;
    clone->symmetricPairMap = symmetricPairMap;
    clone->selfSymmetricModules = selfSymmetricModules;
    
    // Copy symmetry axis position
    clone->symmetryAxisPosition = symmetryAxisPosition;
    
    // Clone the tree structure
    if (root) {
        function<shared_ptr<BStarTreeNode>(shared_ptr<BStarTreeNode>)> cloneNode;
        cloneNode = [&cloneNode](shared_ptr<BStarTreeNode> node) -> shared_ptr<BStarTreeNode> {
            if (!node) return nullptr;
            
            auto newNode = make_shared<BStarTreeNode>(node->getModuleName());
            
            if (node->getLeftChild()) {
                auto leftChild = cloneNode(node->getLeftChild());
                newNode->setLeftChild(leftChild);
                leftChild->setParent(newNode);
            }
            
            if (node->getRightChild()) {
                auto rightChild = cloneNode(node->getRightChild());
                newNode->setRightChild(rightChild);
                rightChild->setParent(newNode);
            }
            
            return newNode;
        };
        
        clone->root = cloneNode(root);
        
        // Build the node lookup map for the clone
        clone->registerNodeInMap(clone->root);
    }
    
    // Clone contours
    clone->horizontalContour = make_shared<Contour>(*horizontalContour);
    clone->verticalContour = make_shared<Contour>(*verticalContour);
    
    return clone;
}