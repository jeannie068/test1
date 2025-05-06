/**
 * HBStarTree.hpp
 * 
 * This file defines the HBStarTree class, which is a hierarchical framework
 * that can simultaneously optimize the placement with both symmetry islands
 * and non-symmetric modules.
 * 
 * Based on the paper "Analog Placement Based on Symmetry-Island Formulation".
 */

#pragma once

#include <memory>
#include <vector>
#include <map>
#include <string>
#include <utility>
#include <set>
#include <unordered_map> // Added for efficient node lookup

#include "HBStarTreeNode.hpp"
#include "ASFBStarTree.hpp"
#include "Module.hpp"
#include "SymmetryConstraint.hpp"
#include "../utils/Contour.hpp"
using namespace std;

class HBStarTree {
private:
    shared_ptr<HBStarTreeNode> root;
    
    map<string, shared_ptr<Module>> modules; // All modules
    vector<shared_ptr<SymmetryGroup>> symmetryGroups; // All symmetry groups 
    
    // Map from symmetry group name to its hierarchy node
    map<string, shared_ptr<HBStarTreeNode>> symmetryGroupNodes;
    
    // Map from module name to its node in the tree
    map<string, shared_ptr<HBStarTreeNode>> moduleNodes;
    
    // NEW: Direct node lookup maps for faster operations
    unordered_map<string, shared_ptr<HBStarTreeNode>> nodeMap; // Any node by name
    
    // NEW: Track modified subtrees for partial repacking
    std::set<shared_ptr<HBStarTreeNode>> modifiedSubtrees;
    
    // Contour for efficient packing
    shared_ptr<Contour> horizontalContour;
    shared_ptr<Contour> verticalContour;
    
    // Total area of the placement
    int totalArea;
    
    // Flag indicating whether the tree has been packed
    bool isPacked;
    
    // Internal helper methods
    void updateContourNodes();
    bool validateSymmetryIslandPlacement() const;
    shared_ptr<HBStarTreeNode> findNearestContourNode(shared_ptr<HBStarTreeNode> node) const;
    shared_ptr<HBStarTreeNode> findLeftmostSkewedChild(shared_ptr<HBStarTreeNode> node) const;
    void constructSymmetryIslands();
    void constructInitialTreeStructure();
    void clearTree();
    
    // NEW: Partial repacking methods
    void markSubtreeForRepack(shared_ptr<HBStarTreeNode> node);
    void repackAffectedSubtrees();
    void updateContourForSubtree(shared_ptr<HBStarTreeNode> node);
    void packSubtree(shared_ptr<HBStarTreeNode> node);
    
    // NEW: Helper to register nodes in lookup maps
    void registerNodeInMap(shared_ptr<HBStarTreeNode> node);
    void unregisterNodeFromMap(shared_ptr<HBStarTreeNode> node);
    
public:
    // Constructor
    HBStarTree();
    ~HBStarTree();
    
    // Adds a module to the tree
    void addModule(shared_ptr<Module> module);
    
    // Adds a symmetry group to the tree
    void addSymmetryGroup(shared_ptr<SymmetryGroup> group);
    
    // Constructs an initial HB*-tree
    void constructInitialTree();
    void constructImprovedInitialTree();
    
    /**
     * Calculates the coordinates of all modules by packing the HB*-tree
     * 
     * @return True if packing was successful, false otherwise
     */
    bool pack();
    
    /**
     * Performs a rotation operation on a module
     * 
     * @param moduleName Name of the module to rotate
     * @return True if the rotation was successful, false otherwise
     */
    bool rotateModule(const string& moduleName);
    
    /**
     * Moves a node to a new position in the tree
     * 
     * @param nodeName Name of the node to move
     * @param newParentName Name of the new parent node
     * @param asLeftChild True if the node should be the left child, false for right child
     * @return True if the move was successful, false otherwise
     */
    bool moveNode(const string& nodeName, 
                  const string& newParentName, 
                  bool asLeftChild);
    
    /**
     * Swaps two nodes in the tree
     * 
     * @param nodeName1 Name of the first node
     * @param nodeName2 Name of the second node
     * @return True if the swap was successful, false otherwise
     */
    bool swapNodes(const string& nodeName1, const string& nodeName2);
    
    /**
     * Changes the representative of a symmetry pair in a symmetry group
     * 
     * @param symmetryGroupName Name of the symmetry group
     * @param moduleName Name of the module in the symmetry pair
     * @return True if the change was successful, false otherwise
     */
    bool changeRepresentative(const string& symmetryGroupName, const string& moduleName);
    
    /**
     * Converts the symmetry type of a symmetry group
     * 
     * @param symmetryGroupName Name of the symmetry group
     * @return True if the conversion was successful, false otherwise
     */
    bool convertSymmetryType(const string& symmetryGroupName);
    
    /* Returns the total area of the placement */
    int getArea() const;
    
    /**
     * Returns the total wire length of the placement
     * 
     * @return Total wire length of the placement
     */
    int getWireLength() const;
    
    /**
     * Gets the root node of the HB*-tree
     * 
     * @return Root node of the HB*-tree
     */
    shared_ptr<HBStarTreeNode> getRoot() const;
    
    /**
     * Gets all modules in the design
     * 
     * @return Map of module names to modules
     */
    const map<string, shared_ptr<Module>>& getModules() const;
    
    /**
     * Gets all symmetry groups in the design
     * 
     * @return Vector of symmetry groups
     */
    const vector<shared_ptr<SymmetryGroup>>& getSymmetryGroups() const;
    
    /**
     * Gets the module node with the given name
     * 
     * @param moduleName Name of the module
     * @return Module node with the given name, or nullptr if not found
     */
    shared_ptr<HBStarTreeNode> getModuleNode(const string& moduleName) const;
    
    /**
     * Gets the symmetry group node with the given name
     * 
     * @param symmetryGroupName Name of the symmetry group
     * @return Symmetry group node with the given name, or nullptr if not found
     */
    shared_ptr<HBStarTreeNode> getSymmetryGroupNode(const string& symmetryGroupName) const;
    
    /**
     * Creates a deep copy of this HB*-tree
     * 
     * @return A new HB*-tree that is a deep copy of this one
     */
    shared_ptr<HBStarTree> clone() const;
    
    // NEW: Direct node lookup method
    shared_ptr<HBStarTreeNode> findNode(const string& nodeName) const;
};