/**
 * HBStarTreePacking.cpp
 * 
 * Implementation of packing-related methods for the HBStarTree class.
 */

#include "HBStarTree.hpp"
#include <algorithm>
#include <queue>
#include <iostream>
#include <limits>
using namespace std;

/**
 * Calculates the coordinates of all modules by packing the HB*-tree
 */
bool HBStarTree::pack() {
    if (!root) return false;
    
    // If there are modified subtrees, only repack those
    if (!modifiedSubtrees.empty()) {
        repackAffectedSubtrees();
        return true;
    }
    
    // Reset contours
    horizontalContour->clear();
    verticalContour->clear();
    
    // Initialize horizontal contour with a segment at y=0
    horizontalContour->addSegment(0, std::numeric_limits<int>::max(), 0);
    
    // Initialize vertical contour with a segment at x=0
    verticalContour->addSegment(0, std::numeric_limits<int>::max(), 0);
    
    // Pack the entire tree
    packSubtree(root);
    
    // Calculate total area
    int minX = std::numeric_limits<int>::max(), minY = std::numeric_limits<int>::max();
    int maxX = 0, maxY = 0;
    
    // Find the minimum and maximum coordinates from all modules
    for (const auto& pair : modules) {
        const auto& module = pair.second;
        minX = std::min(minX, module->getX());
        minY = std::min(minY, module->getY());
        maxX = std::max(maxX, module->getX() + module->getWidth());
        maxY = std::max(maxY, module->getY() + module->getHeight());
    }
    
    // Update total area - FIXED to use (maxX - minX) * (maxY - minY)
    totalArea = (maxX - minX) * (maxY - minY);
    
    // Update contour nodes
    updateContourNodes();
    
    isPacked = true;
    
    return true;
}

/**
 * Pack a subtree starting from the given node
 */
void HBStarTree::packSubtree(shared_ptr<HBStarTreeNode> node) {
    if (!node) return;
    
    switch (node->getType()) {
        case HBNodeType::MODULE: {
            // Pack a regular module
            const string& moduleName = node->getModuleName();
            auto module = modules[moduleName];
            
            if (!module) return;
            
            int x = 0, y = 0;
            
            // Calculate x-coordinate based on B*-tree rules
            if (node->getParent()) {
                if (node->isLeftChild()) {
                    // Left child: place to the right of parent
                    if (node->getParent()->getType() == HBNodeType::MODULE) {
                        auto parentModule = modules[node->getParent()->getModuleName()];
                        if (parentModule) {
                            x = parentModule->getX() + parentModule->getWidth();
                        }
                    } else if (node->getParent()->getType() == HBNodeType::HIERARCHY) {
                        auto asfTree = node->getParent()->getASFTree();
                        if (asfTree) {
                            x = static_cast<int>(asfTree->getSymmetryAxisPosition());
                        }
                    } else if (node->getParent()->getType() == HBNodeType::CONTOUR) {
                        int x1, y1, x2, y2;
                        node->getParent()->getContour(x1, y1, x2, y2);
                        x = x2;
                    }
                } else {
                    // Right child: same x-coordinate as parent
                    if (node->getParent()->getType() == HBNodeType::MODULE) {
                        auto parentModule = modules[node->getParent()->getModuleName()];
                        if (parentModule) {
                            x = parentModule->getX();
                        }
                    } else if (node->getParent()->getType() == HBNodeType::HIERARCHY) {
                        x = 0;
                    } else if (node->getParent()->getType() == HBNodeType::CONTOUR) {
                        int x1, y1, x2, y2;
                        node->getParent()->getContour(x1, y1, x2, y2);
                        x = x1;
                    }
                }
            }
            
            // Calculate y-coordinate using the horizontal contour
            y = horizontalContour->getHeight(x, x + module->getWidth());
            
            // Set the module's position
            module->setPosition(x, y);
            
            // Update contours
            horizontalContour->addSegment(x, x + module->getWidth(), y + module->getHeight());
            verticalContour->addSegment(y, y + module->getHeight(), x + module->getWidth());
            
            break;
        }
        case HBNodeType::HIERARCHY: {
            // Pack a symmetry island
            auto asfTree = node->getASFTree();
            if (!asfTree) return;
            
            // Pack the ASF-B*-tree
            asfTree->pack();
            
            // Get the bounding rectangle of the symmetry island
            int minX = std::numeric_limits<int>::max();
            int minY = std::numeric_limits<int>::max();
            int symMaxX = 0;
            int symMaxY = 0;
            
            for (const auto& pair : asfTree->getModules()) {
                const auto& module = pair.second;
                
                minX = min(minX, module->getX());
                minY = min(minY, module->getY());
                symMaxX = max(symMaxX, module->getX() + module->getWidth());
                symMaxY = max(symMaxY, module->getY() + module->getHeight());
            }
            
            // Calculate the position for the symmetry island
            int x = 0, y = 0;
            
            // Calculate x-coordinate based on B*-tree rules
            if (node->getParent()) {
                if (node->isLeftChild()) {
                    // Left child: place to the right of parent
                    if (node->getParent()->getType() == HBNodeType::MODULE) {
                        auto parentModule = modules[node->getParent()->getModuleName()];
                        if (parentModule) {
                            x = parentModule->getX() + parentModule->getWidth();
                        }
                    } else if (node->getParent()->getType() == HBNodeType::HIERARCHY) {
                        auto parentAsfTree = node->getParent()->getASFTree();
                        if (parentAsfTree) {
                            x = static_cast<int>(parentAsfTree->getSymmetryAxisPosition());
                        }
                    } else if (node->getParent()->getType() == HBNodeType::CONTOUR) {
                        int x1, y1, x2, y2;
                        node->getParent()->getContour(x1, y1, x2, y2);
                        x = x2;
                    }
                } else {
                    // Right child: same x-coordinate as parent
                    if (node->getParent()->getType() == HBNodeType::MODULE) {
                        auto parentModule = modules[node->getParent()->getModuleName()];
                        if (parentModule) {
                            x = parentModule->getX();
                        }
                    } else if (node->getParent()->getType() == HBNodeType::HIERARCHY) {
                        x = 0;
                    } else if (node->getParent()->getType() == HBNodeType::CONTOUR) {
                        int x1, y1, x2, y2;
                        node->getParent()->getContour(x1, y1, x2, y2);
                        x = x1;
                    }
                }
            }
            
            // Calculate y-coordinate using the horizontal contour
            y = horizontalContour->getHeight(x, x + (symMaxX - minX));
            
            // Shift all modules in the symmetry island
            int deltaX = x - minX;
            int deltaY = y - minY;
            
            for (const auto& pair : asfTree->getModules()) {
                const auto& module = pair.second;
                module->setPosition(module->getX() + deltaX, module->getY() + deltaY);
            }
            
            // Update contours
            horizontalContour->addSegment(x, x + (symMaxX - minX), y + (symMaxY - minY));
            verticalContour->addSegment(y, y + (symMaxY - minY), x + (symMaxX - minX));
            
            break;
        }
        case HBNodeType::CONTOUR:
            // Contour nodes don't need to be packed
            break;
    }
    
    // Recursively pack children
    if (node->getLeftChild()) {
        packSubtree(node->getLeftChild());
    }
    if (node->getRightChild()) {
        packSubtree(node->getRightChild());
    }
}

/**
 * Updates contour nodes after changing the ASF-B*-tree of a symmetry group
 */
void HBStarTree::updateContourNodes() {
    // Process each hierarchy node
    for (const auto& pair : symmetryGroupNodes) {
        auto hierarchyNode = pair.second;
        auto asfTree = hierarchyNode->getASFTree();
        
        if (!asfTree) continue;
        
        // Get the contours of the symmetry island
        auto contours = asfTree->getContours();
        auto horizontalContour = contours.first;
        
        // Get the horizontal contour segments
        auto segments = horizontalContour->getSegments();
        
        // Clear existing contour nodes
        vector<shared_ptr<HBStarTreeNode>> existingContourNodes;
        queue<shared_ptr<HBStarTreeNode>> queue;
        
        if (hierarchyNode->getRightChild()) {
            queue.push(hierarchyNode->getRightChild());
        }
        
        while (!queue.empty()) {
            auto current = queue.front();
            queue.pop();
            
            if (current->getType() == HBNodeType::CONTOUR) {
                existingContourNodes.push_back(current);
                
                if (current->getLeftChild()) {
                    queue.push(current->getLeftChild());
                }
                if (current->getRightChild()) {
                    queue.push(current->getRightChild());
                }
            }
        }
        
        // Create new contour nodes
        vector<shared_ptr<HBStarTreeNode>> newContourNodes;
        for (size_t i = 0; i < segments.size(); ++i) {
            auto contourNode = make_shared<HBStarTreeNode>(HBNodeType::CONTOUR, 
                                                             pair.first + "_contour_" + to_string(i));
            contourNode->setContour(segments[i].start, segments[i].height, segments[i].end, segments[i].height);
            newContourNodes.push_back(contourNode);
        }
        
        // Connect contour nodes
        if (!newContourNodes.empty()) {
            // Connect the first contour node to the hierarchy node
            hierarchyNode->setRightChild(newContourNodes[0]);
            newContourNodes[0]->setParent(hierarchyNode);
            
            // Connect the rest of the contour nodes
            for (size_t i = 0; i < newContourNodes.size() - 1; ++i) {
                newContourNodes[i]->setLeftChild(newContourNodes[i + 1]);
                newContourNodes[i + 1]->setParent(newContourNodes[i]);
            }
        }
        
        // Find dangling nodes - nodes whose parents were contour nodes that no longer exist
        vector<shared_ptr<HBStarTreeNode>> danglingNodes;
        for (const auto& oldContourNode : existingContourNodes) {
            if (oldContourNode->getRightChild()) {
                danglingNodes.push_back(oldContourNode->getRightChild());
            }
        }
        
        // Reassign dangling nodes
        for (const auto& danglingNode : danglingNodes) {
            // Find the nearest contour node
            auto nearestContourNode = findNearestContourNode(danglingNode);
            
            if (nearestContourNode) {
                if (!nearestContourNode->getRightChild()) {
                    // Attach directly as right child
                    nearestContourNode->setRightChild(danglingNode);
                    danglingNode->setParent(nearestContourNode);
                } else {
                    // Find the leftmost skewed child
                    auto leftmostSkewedChild = findLeftmostSkewedChild(nearestContourNode->getRightChild());
                    
                    leftmostSkewedChild->setLeftChild(danglingNode);
                    danglingNode->setParent(leftmostSkewedChild);
                }
            }
        }
    }
}

/**
 * Validates that all symmetry islands are placed correctly
 */
bool HBStarTree::validateSymmetryIslandPlacement() const {
    // Check each symmetry group
    for (const auto& group : symmetryGroups) {
        auto it = symmetryGroupNodes.find(group->getName());
        if (it == symmetryGroupNodes.end()) continue;
        
        auto hierarchyNode = it->second;
        auto asfTree = hierarchyNode->getASFTree();
        
        if (!asfTree || !asfTree->isSymmetricFeasible()) {
            return false;
        }
    }
    
    return true;
}

/**
 * Repack only the affected subtrees
 */
void HBStarTree::repackAffectedSubtrees() {
    if (modifiedSubtrees.empty()) return;
    
    // Reset contours
    horizontalContour->clear();
    verticalContour->clear();
    
    // Initialize horizontal contour with a segment at y=0
    horizontalContour->addSegment(0, std::numeric_limits<int>::max(), 0);
    
    // Initialize vertical contour with a segment at x=0
    verticalContour->addSegment(0, std::numeric_limits<int>::max(), 0);
    
    // Use a map to efficiently track nodes by their height in the tree
    unordered_map<shared_ptr<HBStarTreeNode>, int> nodeDepths;
    vector<shared_ptr<HBStarTreeNode>> rootsToRepack;
    
    // Calculate depths and find roots in one pass
    for (const auto& node : modifiedSubtrees) {
        // Skip if already processed
        if (nodeDepths.find(node) != nodeDepths.end()) continue;
        
        // Calculate depth and check if it's a root
        int depth = 0;
        bool isRoot = true;
        auto parent = node->getParent();
        auto current = parent;
        
        while (current) {
            depth++;
            // If any ancestor is also modified, this is not a root
            if (modifiedSubtrees.find(current) != modifiedSubtrees.end()) {
                isRoot = false;
                break;
            }
            current = current->getParent();
        }
        
        nodeDepths[node] = depth;
        
        if (isRoot) {
            rootsToRepack.push_back(node);
        }
    }
    
    // Sort by depth (deeper nodes first)
    sort(rootsToRepack.begin(), rootsToRepack.end(), 
         [&nodeDepths](const shared_ptr<HBStarTreeNode>& a, const shared_ptr<HBStarTreeNode>& b) {
             return nodeDepths[a] > nodeDepths[b];
         });
    
    // Now we need to consider the placement order more carefully
    // Pack the tree in a bottom-up manner
    
    // First, check if the root node itself is modified
    bool rootModified = false;
    for (const auto& node : modifiedSubtrees) {
        if (node == root) {
            rootModified = true;
            break;
        }
    }
    
    if (rootModified) {
        // If root is modified, pack the entire tree
        packSubtree(root);
    } else {
        // Otherwise, pack only the modified subtrees
        for (const auto& node : rootsToRepack) {
            // Make sure to update contours with any already placed nodes above this subtree
            updateContourForSubtree(node);
            
            // Now pack the subtree
            packSubtree(node);
        }
    }
    
    // Calculate total area
    int minX = std::numeric_limits<int>::max(), minY = std::numeric_limits<int>::max();
    int maxX = 0, maxY = 0;
    
    for (const auto& pair : modules) {
        const auto& module = pair.second;
        minX = std::min(minX, module->getX());
        minY = std::min(minY, module->getY());
        maxX = std::max(maxX, module->getX() + module->getWidth());
        maxY = std::max(maxY, module->getY() + module->getHeight());
    }
    
    // Update total area
    totalArea = (maxX - minX) * (maxY - minY);
    
    // Update contour nodes
    updateContourNodes();
    
    // Clear the modified set
    modifiedSubtrees.clear();
}

void HBStarTree::updateContourForSubtree(shared_ptr<HBStarTreeNode> node) {
    if (!node) return;
    
    // Start from root and add all modules to contour until we reach this node
    queue<shared_ptr<HBStarTreeNode>> nodeQueue;
    nodeQueue.push(root);
    
    while (!nodeQueue.empty()) {
        auto current = nodeQueue.front();
        nodeQueue.pop();
        
        // Skip this subtree
        if (current == node) continue;
        
        // Process this node
        if (current->getType() == HBNodeType::MODULE) {
            const string& moduleName = current->getModuleName();
            auto module = modules[moduleName];
            
            if (module) {
                // Update contours with this module
                int x = module->getX();
                int y = module->getY();
                int width = module->getWidth();
                int height = module->getHeight();
                
                horizontalContour->addSegment(x, x + width, y + height);
                verticalContour->addSegment(y, y + height, x + width);
            }
        } else if (current->getType() == HBNodeType::HIERARCHY) {
            auto asfTree = current->getASFTree();
            
            if (asfTree) {
                // Update contours with all modules in this symmetry island
                for (const auto& pair : asfTree->getModules()) {
                    const auto& module = pair.second;
                    
                    int x = module->getX();
                    int y = module->getY();
                    int width = module->getWidth();
                    int height = module->getHeight();
                    
                    horizontalContour->addSegment(x, x + width, y + height);
                    verticalContour->addSegment(y, y + height, x + width);
                }
            }
        }
        
        // Enqueue children
        if (current->getLeftChild() && current->getLeftChild() != node) {
            nodeQueue.push(current->getLeftChild());
        }
        if (current->getRightChild() && current->getRightChild() != node) {
            nodeQueue.push(current->getRightChild());
        }
    }
}
