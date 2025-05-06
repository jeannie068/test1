/**
 * HBStarTreeUtils.cpp
 * 
 * Implementation of utility methods for the HBStarTree class.
 */

#include "HBStarTree.hpp"
#include <algorithm>
#include <queue>
#include <iostream>
#include <limits>
using namespace std;

/**
 * Finds the nearest contour node for a given node
 */
shared_ptr<HBStarTreeNode> HBStarTree::findNearestContourNode(shared_ptr<HBStarTreeNode> node) const {
    if (!node || !root) return nullptr;
    
    // Use BFS to find the nearest contour node
    queue<shared_ptr<HBStarTreeNode>> queue;
    queue.push(root);
    
    while (!queue.empty()) {
        auto current = queue.front();
        queue.pop();
        
        if (current->getType() == HBNodeType::CONTOUR) {
            return current;
        }
        
        if (current->getLeftChild()) queue.push(current->getLeftChild());
        if (current->getRightChild()) queue.push(current->getRightChild());
    }
    
    return nullptr;
}

/**
 * Finds the leftmost skewed child of a node
 */
shared_ptr<HBStarTreeNode> HBStarTree::findLeftmostSkewedChild(shared_ptr<HBStarTreeNode> node) const {
    if (!node) return nullptr;
    
    auto current = node;
    while (current->getLeftChild()) {
        current = current->getLeftChild();
    }
    
    return current;
}

/**
 * Register a node in the lookup maps
 */
void HBStarTree::registerNodeInMap(shared_ptr<HBStarTreeNode> node) {
    if (!node) return;
    
    // Add to node map by name
    nodeMap[node->getName()] = node;
    
    // Recursively register children
    if (node->getLeftChild()) {
        registerNodeInMap(node->getLeftChild());
    }
    if (node->getRightChild()) {
        registerNodeInMap(node->getRightChild());
    }
}

/**
 * Unregister a node from the lookup maps
 */
void HBStarTree::unregisterNodeFromMap(shared_ptr<HBStarTreeNode> node) {
    if (!node) return;
    
    // Remove from node map
    nodeMap.erase(node->getName());
    
    // Recursively unregister children
    if (node->getLeftChild()) {
        unregisterNodeFromMap(node->getLeftChild());
    }
    if (node->getRightChild()) {
        unregisterNodeFromMap(node->getRightChild());
    }
}

shared_ptr<HBStarTreeNode> HBStarTree::findNode(const string& nodeName) const {
    auto it = nodeMap.find(nodeName);
    if (it != nodeMap.end()) {
        return it->second;
    }
    return nullptr;
}

/**
 * Gets the root node of the HB*-tree
 */
shared_ptr<HBStarTreeNode> HBStarTree::getRoot() const {
    return root;
}

/**
 * Gets all modules in the design
 */
const map<string, shared_ptr<Module>>& HBStarTree::getModules() const {
    return modules;
}

/**
 * Gets all symmetry groups in the design
 */
const vector<shared_ptr<SymmetryGroup>>& HBStarTree::getSymmetryGroups() const {
    return symmetryGroups;
}

shared_ptr<HBStarTreeNode> HBStarTree::getModuleNode(const string& moduleName) const {
    auto it = moduleNodes.find(moduleName);
    if (it == moduleNodes.end()) return nullptr;
    
    return it->second;
}

shared_ptr<HBStarTreeNode> HBStarTree::getSymmetryGroupNode(const string& symmetryGroupName) const {
    auto it = symmetryGroupNodes.find(symmetryGroupName);
    if (it == symmetryGroupNodes.end()) return nullptr;
    
    return it->second;
}

/**
 * Returns the total area of the placement
 */
int HBStarTree::getArea() const {
    return totalArea;
}

/**
 * Returns the total wire length of the placement
 */
int HBStarTree::getWireLength() const {
    // Wire length calculation depends on netlist information
    // This is a simplified implementation
    return 0;
}

/**
 * Creates a deep copy of this HB*-tree
 */
shared_ptr<HBStarTree> HBStarTree::clone() const {
    auto clone = make_shared<HBStarTree>();
    
    // Copy modules
    for (const auto& pair : modules) {
        auto moduleCopy = make_shared<Module>(*pair.second);
        clone->modules[pair.first] = moduleCopy;
    }
    
    // Copy symmetry groups
    for (const auto& group : symmetryGroups) {
        // Deep copy of symmetry group
        auto groupCopy = make_shared<SymmetryGroup>(*group);
        clone->symmetryGroups.push_back(groupCopy);
    }
    
    // Reconstruct initial tree
    clone->constructInitialTree();
    
    // Copy packed state
    clone->isPacked = isPacked;
    clone->totalArea = totalArea;
    
    // Copy contours
    clone->horizontalContour = make_shared<Contour>(*horizontalContour);
    clone->verticalContour = make_shared<Contour>(*verticalContour);
    
    return clone;
}
