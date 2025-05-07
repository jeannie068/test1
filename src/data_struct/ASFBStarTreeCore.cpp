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
      symmetryAxisPosition(0.0),
      axisPositionLocked(false) {
    
    if (!symmetryGroup) {
        cerr << "Error: Null symmetry group provided to ASF-B*-tree" << endl;
        return;
    }
    
    // Process symmetry pairs and determine representatives
    for (const auto& pair : symmetryGroup->getSymmetryPairs()) {
        const string& module1 = pair.first;
        const string& module2 = pair.second;
        
        // For each symmetry pair, choose the module with lexicographically larger name as representative
        string representative = (module1 < module2) ? module2 : module1;
        string nonRepresentative = (representative == module1) ? module2 : module1;
        
        // Track the representative relationship
        representativeMap[module1] = representative;
        representativeMap[module2] = representative;
        
        // Track the symmetric pair relationship
        symmetricPairMap[module1] = module2;
        symmetricPairMap[module2] = module1;
        
        // Add to appropriate sets
        if (representative == module1) {
            representativeModules.insert(module1);
            nonRepresentativeModules.insert(module2);
        } else {
            representativeModules.insert(module2);
            nonRepresentativeModules.insert(module1);
        }
    }
    
    // Process self-symmetric modules
    for (const auto& moduleName : symmetryGroup->getSelfSymmetric()) {
        // For a self-symmetric module, it represents itself
        representativeMap[moduleName] = moduleName;
        selfSymmetricModules.push_back(moduleName);
        representativeModules.insert(moduleName);
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
 * Locks the symmetry axis based on the symmetry type
 * This is called once after all modules are added
 */
void ASFBStarTree::lockSymmetryAxis() {
    if (axisPositionLocked) return;
    
    // Calculate a locked symmetry axis position based on initial module positions
    // For vertical symmetry, we start with a central position
    if (symmetryGroup->getType() == SymmetryType::VERTICAL) {
        // Calculate a reasonable x-axis position based on the width of modules
        int totalWidth = 0;
        int count = 0;
        
        for (const auto& moduleName : representativeModules) {
            auto it = modules.find(moduleName);
            if (it != modules.end() && it->second) {
                totalWidth += it->second->getWidth();
                count++;
            }
        }
        
        if (count > 0) {
            symmetryAxisPosition = totalWidth / count; // Simple average for initialization
        } else {
            symmetryAxisPosition = 0.0;
        }
    } else { // HORIZONTAL symmetry
        // Calculate a reasonable y-axis position
        int totalHeight = 0;
        int count = 0;
        
        for (const auto& moduleName : representativeModules) {
            auto it = modules.find(moduleName);
            if (it != modules.end() && it->second) {
                totalHeight += it->second->getHeight();
                count++;
            }
        }
        
        if (count > 0) {
            symmetryAxisPosition = totalHeight / count;
        } else {
            symmetryAxisPosition = 0.0;
        }
    }
    
    axisPositionLocked = true;
    
    cout << "Locked symmetry axis position: " << symmetryAxisPosition << endl;
}

/**
 * Constructs an initial ASF-B*-tree based on the symmetry group
 */
void ASFBStarTree::constructInitialTree() {
    // Clear any existing tree and node map
    root = nullptr;
    nodeMap.clear();
    
    // Lock the symmetry axis position if not already locked
    if (!axisPositionLocked) {
        lockSymmetryAxis();
    }
    
    // Collect all representatives for the B*-tree
    vector<string> representatives;
    for (const auto& moduleName : representativeModules) {
        auto it = modules.find(moduleName);
        if (it != modules.end() && it->second) {
            representatives.push_back(moduleName);
        }
    }
    
    if (representatives.empty()) {
        cerr << "Error: No representatives found for ASF-B*-tree" << endl;
        return;
    }
    
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
        
        // For self-symmetric modules, they must be on the boundary
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
            // For symmetry pairs, can place anywhere
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
        
        if (module) {
            minX = min(minX, module->getX());
            minY = min(minY, module->getY());
            maxX = max(maxX, module->getX() + module->getWidth());
            maxY = max(maxY, module->getY() + module->getHeight());
        }
    }
    
    // If minX/minY are still at max value, no valid modules were found
    if (minX == numeric_limits<int>::max() || minY == numeric_limits<int>::max()) {
        return 0;
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
    // Only look for representative nodes in the ASF-B*-tree
    if (!isRepresentative(nodeName)) {
        return nullptr;
    }
    
    auto it = nodeMap.find(nodeName);
    if (it != nodeMap.end()) {
        return it->second;
    }
    
    // If not found in the map, traverse the tree
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
    return representativeModules.count(moduleName) > 0;
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
    
    // Copy representative maps and sets
    clone->representativeMap = representativeMap;
    clone->symmetricPairMap = symmetricPairMap;
    clone->selfSymmetricModules = selfSymmetricModules;
    clone->representativeModules = representativeModules;
    clone->nonRepresentativeModules = nonRepresentativeModules;
    
    // Copy symmetry axis position and lock status
    clone->symmetryAxisPosition = symmetryAxisPosition;
    clone->axisPositionLocked = axisPositionLocked;
    
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