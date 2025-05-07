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
        if (module) {
            minX = std::min(minX, module->getX());
            minY = std::min(minY, module->getY());
            maxX = std::max(maxX, module->getX() + module->getWidth());
            maxY = std::max(maxY, module->getY() + module->getHeight());
        }
    }
    
    // Update total area - ensure we're using correct min/max values
    if (minX < maxX && minY < maxY) {
        totalArea = (maxX - minX) * (maxY - minY);
    } else {
        cerr << "Warning: Invalid area calculation (minX=" << minX << ", minY=" << minY 
             << ", maxX=" << maxX << ", maxY=" << maxY << ")" << endl;
        totalArea = 0;
    }
    
    // Update contour nodes
    updateContourNodes();
    
    isPacked = true;
    
    // Validate and fix any overlaps in the final placement
    bool valid = validateAndFixOverlaps();
    
    // If there were significant overlap issues, fix by shifting modules
    if (!valid) {
        cerr << "Warning: Overlaps detected after initial packing - attempting to fix" << endl;
        shiftOverlappingModules();
        
        // Recalculate area after fixing overlaps
        minX = std::numeric_limits<int>::max(), minY = std::numeric_limits<int>::max();
        maxX = 0, maxY = 0;
        
        for (const auto& pair : modules) {
            const auto& module = pair.second;
            if (module) {
                minX = std::min(minX, module->getX());
                minY = std::min(minY, module->getY());
                maxX = std::max(maxX, module->getX() + module->getWidth());
                maxY = std::max(maxY, module->getY() + module->getHeight());
            }
        }
        
        if (minX < maxX && minY < maxY) {
            totalArea = (maxX - minX) * (maxY - minY);
        }
    }
    
    return true;
}

bool HBStarTree::validateAndFixOverlaps() {
    bool valid = true;
    int fixedOverlaps = 0;
    
    // Check each pair of modules for overlap
    for (auto it1 = modules.begin(); it1 != modules.end(); ++it1) {
        const auto& module1 = it1->second;
        if (!module1) continue;
        
        for (auto it2 = next(it1); it2 != modules.end(); ++it2) {
            const auto& module2 = it2->second;
            if (!module2) continue;
            
            // Check for overlap using box overlap test
            bool overlaps = 
                module1->getX() < module2->getX() + module2->getWidth() &&
                module1->getX() + module1->getWidth() > module2->getX() &&
                module1->getY() < module2->getY() + module2->getHeight() &&
                module1->getY() + module1->getHeight() > module2->getY();
                
            if (overlaps) {
                valid = false;
                fixedOverlaps++;
                
                // Calculate overlap in each direction
                int overlapX = min(module1->getX() + module1->getWidth(), 
                                  module2->getX() + module2->getWidth()) - 
                               max(module1->getX(), module2->getX());
                
                int overlapY = min(module1->getY() + module1->getHeight(), 
                                  module2->getY() + module2->getHeight()) - 
                               max(module1->getY(), module2->getY());
                
                // Choose the direction with smaller overlap to fix
                if (overlapX <= overlapY) {
                    // Resolve horizontally
                    if (module1->getX() <= module2->getX()) {
                        // Move module2 to the right of module1
                        module2->setPosition(module1->getX() + module1->getWidth(), module2->getY());
                        cerr << "Fixed horizontal overlap: moved " << it2->first 
                             << " to the right of " << it1->first << endl;
                    } else {
                        // Move module1 to the right of module2
                        module1->setPosition(module2->getX() + module2->getWidth(), module1->getY());
                        cerr << "Fixed horizontal overlap: moved " << it1->first 
                             << " to the right of " << it2->first << endl;
                    }
                } else {
                    // Resolve vertically
                    if (module1->getY() <= module2->getY()) {
                        // Move module2 below module1
                        module2->setPosition(module2->getX(), module1->getY() + module1->getHeight());
                        cerr << "Fixed vertical overlap: moved " << it2->first 
                             << " below " << it1->first << endl;
                    } else {
                        // Move module1 below module2
                        module1->setPosition(module1->getX(), module2->getY() + module2->getHeight());
                        cerr << "Fixed vertical overlap: moved " << it1->first 
                             << " below " << it2->first << endl;
                    }
                }
            }
        }
    }
    
    if (fixedOverlaps > 0) {
        cerr << "Fixed " << fixedOverlaps << " overlaps in placement" << endl;
    }
    
    return valid;
}

void HBStarTree::shiftOverlappingModules() {
    // Create a grid of modules by x, y coordinates
    map<int, map<int, shared_ptr<Module>>> grid;
    
    // Place modules in the grid
    for (const auto& pair : modules) {
        const auto& module = pair.second;
        if (!module) continue;
        
        int x = module->getX();
        int y = module->getY();
        
        // Check if there's already a module at this position
        if (grid[x].find(y) != grid[x].end()) {
            // There's already a module here, shift this one
            int newY = y;
            bool placed = false;
            
            // Try to find a free spot by shifting down
            while (!placed) {
                newY += 10;  // Shift by a small amount
                if (grid[x].find(newY) == grid[x].end()) {
                    // Found a free spot
                    grid[x][newY] = module;
                    module->setPosition(x, newY);
                    placed = true;
                    cerr << "Shifted module " << pair.first << " down to y=" << newY << endl;
                }
            }
        } else {
            // Position is free, add it to the grid
            grid[x][y] = module;
        }
    }
    
    // Update contours after shifting
    horizontalContour->clear();
    verticalContour->clear();
    
    // Initialize horizontal contour with a segment at y=0
    horizontalContour->addSegment(0, std::numeric_limits<int>::max(), 0);
    
    // Initialize vertical contour with a segment at x=0
    verticalContour->addSegment(0, std::numeric_limits<int>::max(), 0);
    
    // Update contours with each module
    for (const auto& pair : modules) {
        const auto& module = pair.second;
        if (!module) continue;
        
        int x = module->getX();
        int y = module->getY();
        int width = module->getWidth();
        int height = module->getHeight();
        
        horizontalContour->addSegment(x, x + width, y + height);
        verticalContour->addSegment(y, y + height, x + width);
    }
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
            auto moduleIt = modules.find(moduleName);
            if (moduleIt == modules.end() || !moduleIt->second) {
                cerr << "Warning: Module " << moduleName << " not found or null" << endl;
                return;
            }
            
            auto module = moduleIt->second;
            
            int x = 0, y = 0;
            
            // Calculate x-coordinate based on B*-tree rules
            if (node->getParent()) {
                if (node->isLeftChild()) {
                    // Left child: place to the right of parent
                    if (node->getParent()->getType() == HBNodeType::MODULE) {
                        auto parentName = node->getParent()->getModuleName();
                        auto parentIt = modules.find(parentName);
                        if (parentIt != modules.end() && parentIt->second) {
                            auto parentModule = parentIt->second;
                            x = parentModule->getX() + parentModule->getWidth();
                        }
                    } else if (node->getParent()->getType() == HBNodeType::HIERARCHY) {
                        auto hierarchyNode = node->getParent();
                        auto asfTree = hierarchyNode->getASFTree();
                        if (asfTree) {
                            // Find rightmost module in the symmetry island
                            int rightmost = 0;
                            for (const auto& pair : asfTree->getModules()) {
                                const auto& m = pair.second;
                                rightmost = max(rightmost, m->getX() + m->getWidth());
                            }
                            x = rightmost;
                        }
                    } else if (node->getParent()->getType() == HBNodeType::CONTOUR) {
                        int x1, y1, x2, y2;
                        node->getParent()->getContour(x1, y1, x2, y2);
                        x = x2; // Use the right end of the contour
                    }
                } else {
                    // Right child: same x-coordinate as parent
                    if (node->getParent()->getType() == HBNodeType::MODULE) {
                        auto parentName = node->getParent()->getModuleName();
                        auto parentIt = modules.find(parentName);
                        if (parentIt != modules.end() && parentIt->second) {
                            auto parentModule = parentIt->second;
                            x = parentModule->getX();
                        }
                    } else if (node->getParent()->getType() == HBNodeType::HIERARCHY) {
                        auto hierarchyNode = node->getParent();
                        auto asfTree = hierarchyNode->getASFTree();
                        if (asfTree) {
                            // Use the leftmost x-coordinate of the hierarchy
                            int leftmost = numeric_limits<int>::max();
                            for (const auto& pair : asfTree->getModules()) {
                                const auto& m = pair.second;
                                leftmost = min(leftmost, m->getX());
                            }
                            x = (leftmost != numeric_limits<int>::max()) ? leftmost : 0;
                        } else {
                            x = 0; // Default if no ASF-B*-tree
                        }
                    } else if (node->getParent()->getType() == HBNodeType::CONTOUR) {
                        int x1, y1, x2, y2;
                        node->getParent()->getContour(x1, y1, x2, y2);
                        x = x1; // Use the left end of the contour
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
            
            // First, pack the ASF-B*-tree at origin (0,0)
            asfTree->pack();
            
            // Calculate the bounding box of the symmetry island
            int minX = numeric_limits<int>::max();
            int minY = numeric_limits<int>::max();
            int maxX = 0;
            int maxY = 0;
            
            for (const auto& pair : asfTree->getModules()) {
                const auto& module = pair.second;
                if (module) {
                    minX = min(minX, module->getX());
                    minY = min(minY, module->getY());
                    maxX = max(maxX, module->getX() + module->getWidth());
                    maxY = max(maxY, module->getY() + module->getHeight());
                }
            }
            
            // Calculate width and height of the symmetry island
            int width = maxX - minX;
            int height = maxY - minY;
            
            if (width <= 0 || height <= 0) {
                cerr << "Warning: Invalid symmetry island dimensions: " << width << "x" << height << endl;
                break;
            }
            
            // Calculate position for the symmetry island
            int x = 0, y = 0;
            
            // Calculate x-coordinate based on B*-tree rules
            if (node->getParent()) {
                if (node->isLeftChild()) {
                    // Left child: place to the right of parent
                    if (node->getParent()->getType() == HBNodeType::MODULE) {
                        auto parentName = node->getParent()->getModuleName();
                        auto parentIt = modules.find(parentName);
                        if (parentIt != modules.end() && parentIt->second) {
                            auto parentModule = parentIt->second;
                            x = parentModule->getX() + parentModule->getWidth();
                        }
                    } else if (node->getParent()->getType() == HBNodeType::HIERARCHY) {
                        auto parentNode = node->getParent();
                        auto parentAsfTree = parentNode->getASFTree();
                        if (parentAsfTree) {
                            // Find rightmost module in the parent hierarchy
                            int rightmost = 0;
                            for (const auto& pair : parentAsfTree->getModules()) {
                                const auto& m = pair.second;
                                rightmost = max(rightmost, m->getX() + m->getWidth());
                            }
                            x = rightmost;
                        }
                    } else if (node->getParent()->getType() == HBNodeType::CONTOUR) {
                        int x1, y1, x2, y2;
                        node->getParent()->getContour(x1, y1, x2, y2);
                        x = x2; // Use the right end of the contour
                    }
                } else {
                    // Right child: same x-coordinate as parent
                    if (node->getParent()->getType() == HBNodeType::MODULE) {
                        auto parentName = node->getParent()->getModuleName();
                        auto parentIt = modules.find(parentName);
                        if (parentIt != modules.end() && parentIt->second) {
                            auto parentModule = parentIt->second;
                            x = parentModule->getX();
                        }
                    } else if (node->getParent()->getType() == HBNodeType::HIERARCHY) {
                        auto parentNode = node->getParent();
                        auto parentAsfTree = parentNode->getASFTree();
                        if (parentAsfTree) {
                            // Use the leftmost x-coordinate of the parent hierarchy
                            int leftmost = numeric_limits<int>::max();
                            for (const auto& pair : parentAsfTree->getModules()) {
                                const auto& m = pair.second;
                                leftmost = min(leftmost, m->getX());
                            }
                            x = (leftmost != numeric_limits<int>::max()) ? leftmost : 0;
                        } else {
                            x = 0; // Default if no ASF-B*-tree
                        }
                    } else if (node->getParent()->getType() == HBNodeType::CONTOUR) {
                        int x1, y1, x2, y2;
                        node->getParent()->getContour(x1, y1, x2, y2);
                        x = x1; // Use the left end of the contour
                    }
                }
            }
            
            // Calculate y-coordinate using the horizontal contour
            y = horizontalContour->getHeight(x, x + width);
            
            // Shift all modules in the symmetry island
            int deltaX = x - minX;
            int deltaY = y - minY;
            
            for (const auto& pair : asfTree->getModules()) {
                const auto& module = pair.second;
                if (module) {
                    int newX = module->getX() + deltaX;
                    int newY = module->getY() + deltaY;
                    
                    // Ensure no negative coordinates
                    if (newX < 0) newX = 0;
                    if (newY < 0) newY = 0;
                    
                    module->setPosition(newX, newY);
                }
            }
            
            // Update contours with the actual module positions
            for (const auto& pair : asfTree->getModules()) {
                const auto& module = pair.second;
                if (module) {
                    int modX = module->getX();
                    int modY = module->getY();
                    int modWidth = module->getWidth();
                    int modHeight = module->getHeight();
                    
                    horizontalContour->addSegment(modX, modX + modWidth, modY + modHeight);
                    verticalContour->addSegment(modY, modY + modHeight, modX + modWidth);
                }
            }
            
            break;
        }
        case HBNodeType::CONTOUR:
            // Contour nodes don't need to be packed
            break;
    }
    
    // Recursively pack left child first, then right child
    if (node->getLeftChild()) {
        packSubtree(node->getLeftChild());
    }
    if (node->getRightChild()) {
        packSubtree(node->getRightChild());
    }
}

/**
 * Validates that the placement has no overlaps
 */
bool HBStarTree::validatePlacement() const {
    bool valid = true;
    int overlapsDetected = 0;
    
    // Check each pair of modules for overlap
    for (auto it1 = modules.begin(); it1 != modules.end(); ++it1) {
        const auto& module1 = it1->second;
        if (!module1) continue;
        
        for (auto it2 = next(it1); it2 != modules.end(); ++it2) {
            const auto& module2 = it2->second;
            if (!module2) continue;
            
            // Check for overlap using box overlap test
            bool overlaps = 
                module1->getX() < module2->getX() + module2->getWidth() &&
                module1->getX() + module1->getWidth() > module2->getX() &&
                module1->getY() < module2->getY() + module2->getHeight() &&
                module1->getY() + module1->getHeight() > module2->getY();
                
            if (overlaps) {
                cerr << "Overlap detected between " << it1->first 
                     << " (" << module1->getX() << "," << module1->getY() 
                     << "," << module1->getWidth() << "," << module1->getHeight() 
                     << ") and " << it2->first 
                     << " (" << module2->getX() << "," << module2->getY() 
                     << "," << module2->getWidth() << "," << module2->getHeight() 
                     << ")" << endl;
                
                valid = false;
                overlapsDetected++;
                
                // Try to resolve the overlap intelligently
                // Calculate overlap in each direction
                int overlapX = min(module1->getX() + module1->getWidth(), 
                                 module2->getX() + module2->getWidth()) - 
                              max(module1->getX(), module2->getX());
                
                int overlapY = min(module1->getY() + module1->getHeight(), 
                                 module2->getY() + module2->getHeight()) - 
                              max(module1->getY(), module2->getY());
                
                // Choose the direction with smaller overlap to resolve
                if (overlapX <= overlapY) {
                    // Resolve horizontally
                    if (module1->getX() <= module2->getX()) {
                        // Move module2 to the right of module1
                        module2->setPosition(module1->getX() + module1->getWidth(), module2->getY());
                    } else {
                        // Move module1 to the right of module2
                        module1->setPosition(module2->getX() + module2->getWidth(), module1->getY());
                    }
                } else {
                    // Resolve vertically
                    if (module1->getY() <= module2->getY()) {
                        // Move module2 below module1
                        module2->setPosition(module2->getX(), module1->getY() + module1->getHeight());
                    } else {
                        // Move module1 below module2
                        module1->setPosition(module1->getX(), module2->getY() + module2->getHeight());
                    }
                }
            }
        }
    }
    
    if (overlapsDetected > 0) {
        cerr << "Fixed " << overlapsDetected << " overlaps in placement" << endl;
        
        // If many overlaps were fixed, trigger a repack to maintain consistency
        if (overlapsDetected > 5) {
            cerr << "Too many overlaps detected - consider repacking" << endl;
        }
    }
    
    return valid;
}

/**
 * Updates contour nodes after changing the ASF-B*-tree of a symmetry group
 */
void HBStarTree::updateContourNodes() {
    // Process each hierarchy node
    for (const auto& pair : symmetryGroupNodes) {
        auto hierarchyNode = pair.second;
        if (!hierarchyNode) continue;
        
        auto asfTree = hierarchyNode->getASFTree();
        if (!asfTree) continue;
        
        // Get the contours of the symmetry island
        auto contours = asfTree->getContours();
        auto horizontalContour = contours.first;
        if (!horizontalContour) continue;
        
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
            
            if (!current) continue;
            
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
            if (contourNode) {
                contourNode->setContour(segments[i].start, segments[i].height, 
                                       segments[i].end, segments[i].height);
                newContourNodes.push_back(contourNode);
            }
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
            if (oldContourNode && oldContourNode->getRightChild()) {
                danglingNodes.push_back(oldContourNode->getRightChild());
            }
        }
        
        // Reassign dangling nodes
        for (const auto& danglingNode : danglingNodes) {
            if (!danglingNode) continue;
            
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
                    
                    if (leftmostSkewedChild) {
                        leftmostSkewedChild->setLeftChild(danglingNode);
                        danglingNode->setParent(leftmostSkewedChild);
                    }
                }
            }
        }
    }
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
    
    // If root is modified, we need to pack the entire tree
    bool rootModified = false;
    for (const auto& node : modifiedSubtrees) {
        if (node == root) {
            rootModified = true;
            break;
        }
    }
    
    if (rootModified) {
        // Pack the entire tree
        packSubtree(root);
    } else {
        // Find the minimal set of subtrees to repack
        vector<shared_ptr<HBStarTreeNode>> rootsToRepack;
        
        for (const auto& node : modifiedSubtrees) {
            if (!node) continue;
            
            // Skip if already covered by an ancestor in rootsToRepack
            bool alreadyCovered = false;
            for (const auto& repackRoot : rootsToRepack) {
                if (!repackRoot) continue;
                
                auto current = node;
                while (current && current != repackRoot) {
                    current = current->getParent();
                }
                if (current == repackRoot) {
                    alreadyCovered = true;
                    break;
                }
            }
            
            if (!alreadyCovered) {
                rootsToRepack.push_back(node);
            }
        }
        
        // Process in bottom-up order (lowest depth first)
        // Sort by depth in tree (deepest first)
        sort(rootsToRepack.begin(), rootsToRepack.end(),
             [](const shared_ptr<HBStarTreeNode>& a, const shared_ptr<HBStarTreeNode>& b) {
                 // Count depth
                 int depthA = 0, depthB = 0;
                 auto currA = a, currB = b;
                 
                 while (currA && currA->getParent()) {
                     depthA++;
                     currA = currA->getParent();
                 }
                 
                 while (currB && currB->getParent()) {
                     depthB++;
                     currB = currB->getParent();
                 }
                 
                 return depthA > depthB;
             });
        
        // Update contours with already placed nodes
        for (const auto& node : rootsToRepack) {
            if (!node) continue;
            updateContourForSubtree(node);
            packSubtree(node);
        }
    }
    
    // Calculate total area
    int minX = std::numeric_limits<int>::max(), minY = std::numeric_limits<int>::max();
    int maxX = 0, maxY = 0;
    
    for (const auto& pair : modules) {
        const auto& module = pair.second;
        if (module) {
            minX = std::min(minX, module->getX());
            minY = std::min(minY, module->getY());
            maxX = std::max(maxX, module->getX() + module->getWidth());
            maxY = std::max(maxY, module->getY() + module->getHeight());
        }
    }
    
    // Update total area
    if (minX < maxX && minY < maxY) {
        totalArea = (maxX - minX) * (maxY - minY);
    } else {
        cerr << "Warning: Invalid area calculation in repackAffectedSubtrees" << endl;
        totalArea = 0;
    }
    
    // Update contour nodes
    updateContourNodes();
    
    // Clear the modified set
    modifiedSubtrees.clear();
    
    // Validate placement after repacking
    validatePlacement();
}

void HBStarTree::updateContourForSubtree(shared_ptr<HBStarTreeNode> node) {
    if (!node) return;
    
    // Start from root and add all modules to contour until we reach this node
    queue<shared_ptr<HBStarTreeNode>> nodeQueue;
    nodeQueue.push(root);
    
    while (!nodeQueue.empty()) {
        auto current = nodeQueue.front();
        nodeQueue.pop();
        
        if (!current) continue;
        
        // Skip this subtree
        if (current == node) continue;
        
        // Process this node
        if (current->getType() == HBNodeType::MODULE) {
            const string& moduleName = current->getModuleName();
            auto it = modules.find(moduleName);
            if (it != modules.end() && it->second) {
                auto module = it->second;
                
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
                    if (module) {
                        int x = module->getX();
                        int y = module->getY();
                        int width = module->getWidth();
                        int height = module->getHeight();
                        
                        horizontalContour->addSegment(x, x + width, y + height);
                        verticalContour->addSegment(y, y + height, x + width);
                    }
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