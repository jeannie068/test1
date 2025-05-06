// HBStarTreeNode.hpp

#pragma once

#include <memory>
#include <string>
#include "ASFBStarTree.hpp"
using namespace std;

enum class HBNodeType {
    MODULE,    // Regular module node
    HIERARCHY, // Hierarchy node - represents a symmetry island
    CONTOUR    // Contour node - represents a horizontal contour segment
};

class HBStarTreeNode {
private:
    HBNodeType type;
    string name;
    
    // Tree structure
    shared_ptr<HBStarTreeNode> leftChild;  // Module to the right
    shared_ptr<HBStarTreeNode> rightChild; // Module above
    weak_ptr<HBStarTreeNode> parent;       // Parent node (weak to avoid circular references)
    
    // For HIERARCHY type nodes
    shared_ptr<ASFBStarTree> asfTree;      // ASF-B*-tree representing the symmetry island
    
    // For CONTOUR type nodes
    int contourX1;                              // Start x-coordinate of the contour segment
    int contourY1;                              // Start y-coordinate of the contour segment
    int contourX2;                              // End x-coordinate of the contour segment
    int contourY2;                              // End y-coordinate of the contour segment
    
public:
    /**
     * Constructor
     * 
     * @param type Type of the node
     * @param name Name of the node
     */
    HBStarTreeNode(HBNodeType type, const string& name);
    ~HBStarTreeNode();
    
    HBNodeType getType() const;
    string getName() const;
    string getModuleName() const;

    shared_ptr<HBStarTreeNode> getLeftChild() const;
    shared_ptr<HBStarTreeNode> getRightChild() const;
    shared_ptr<HBStarTreeNode> getParent() const;

    void setLeftChild(shared_ptr<HBStarTreeNode> node);
    void setRightChild(shared_ptr<HBStarTreeNode> node);
    void setParent(shared_ptr<HBStarTreeNode> node);
    
    /**
     * Gets the ASF-B*-tree (for HIERARCHY type nodes)
     * 
     * @return ASF-B*-tree representing the symmetry island
     */
    shared_ptr<ASFBStarTree> getASFTree() const;
    
    /**
     * Sets the ASF-B*-tree (for HIERARCHY type nodes)
     * 
     * @param tree New ASF-B*-tree
     */
    void setASFTree(shared_ptr<ASFBStarTree> tree);
    
    /* for CONTOUR type nodes */
    void setContour(int x1, int y1, int x2, int y2);
    void getContour(int& x1, int& y1, int& x2, int& y2) const;
    
    bool isLeaf() const;
    bool isLeftChild() const;
    bool isRightChild() const;
    
    /**
     * Creates a deep copy of this node and its children
     * 
     * @return A new node that is a deep copy of this one
     */
    shared_ptr<HBStarTreeNode> clone() const;
};