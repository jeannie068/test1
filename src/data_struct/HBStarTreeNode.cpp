// HBStarTreeNode.cpp

#include "HBStarTreeNode.hpp"
using namespace std;

/**
 * Constructor
 */
HBStarTreeNode::HBStarTreeNode(HBNodeType type, const string& name)
    : type(type), 
      name(name), 
      leftChild(nullptr), 
      rightChild(nullptr),
      asfTree(nullptr),
      contourX1(0),
      contourY1(0),
      contourX2(0),
      contourY2(0) {
    // Parent is initialized as empty weak_ptr by default
}

HBStarTreeNode::~HBStarTreeNode() {
}

HBNodeType HBStarTreeNode::getType() const {
    return type;
}

/**
 * Gets the node name
 */
string HBStarTreeNode::getName() const {
    return name;
}

string HBStarTreeNode::getModuleName() const {
    // For MODULE type nodes, the name is the module name
    if (type == HBNodeType::MODULE) {
        return name;
    }
    return "";
}

shared_ptr<HBStarTreeNode> HBStarTreeNode::getLeftChild() const {
    return leftChild;
}

shared_ptr<HBStarTreeNode> HBStarTreeNode::getRightChild() const {
    return rightChild;
}

shared_ptr<HBStarTreeNode> HBStarTreeNode::getParent() const {
    // Convert weak_ptr to shared_ptr for returning
    return parent.lock();
}

void HBStarTreeNode::setLeftChild(shared_ptr<HBStarTreeNode> node) {
    leftChild = node;
}

void HBStarTreeNode::setRightChild(shared_ptr<HBStarTreeNode> node) {
    rightChild = node;
}

void HBStarTreeNode::setParent(shared_ptr<HBStarTreeNode> node) {
    parent = node;  // Automatically converts to weak_ptr
}


/**
 * Gets the ASF-B*-tree (for HIERARCHY type nodes)
 */
shared_ptr<ASFBStarTree> HBStarTreeNode::getASFTree() const {
    if (type != HBNodeType::HIERARCHY) {
        return nullptr;
    }
    return asfTree;
}

/**
 * Sets the ASF-B*-tree (for HIERARCHY type nodes)
 */
void HBStarTreeNode::setASFTree(shared_ptr<ASFBStarTree> tree) {
    if (type == HBNodeType::HIERARCHY) {
        asfTree = tree;
    }
}


/**
 * For CONTOUR type nodes
 */
void HBStarTreeNode::setContour(int x1, int y1, int x2, int y2) {
    if (type == HBNodeType::CONTOUR) {
        contourX1 = x1;
        contourY1 = y1;
        contourX2 = x2;
        contourY2 = y2;
    }
}

void HBStarTreeNode::getContour(int& x1, int& y1, int& x2, int& y2) const {
    if (type == HBNodeType::CONTOUR) {
        x1 = contourX1;
        y1 = contourY1;
        x2 = contourX2;
        y2 = contourY2;
    } else {
        // Default values for non-contour nodes
        x1 = 0;
        y1 = 0;
        x2 = 0;
        y2 = 0;
    }
}


bool HBStarTreeNode::isLeaf() const {
    return !leftChild && !rightChild;
}

bool HBStarTreeNode::isLeftChild() const {
    auto parentNode = parent.lock();
    if (!parentNode) return false;  // No parent
    
    return parentNode->getLeftChild().get() == this;
}

bool HBStarTreeNode::isRightChild() const {
    auto parentNode = parent.lock();
    if (!parentNode) return false;  // No parent
    
    return parentNode->getRightChild().get() == this;
}

/**
 * Creates a deep copy of this node and its children
 */
shared_ptr<HBStarTreeNode> HBStarTreeNode::clone() const {
    // Create a new node with the same type and name
    auto clonedNode = make_shared<HBStarTreeNode>(type, name);
    
    // Copy type-specific data
    if (type == HBNodeType::HIERARCHY && asfTree) {
        clonedNode->asfTree = asfTree->clone();
    } else if (type == HBNodeType::CONTOUR) {
        clonedNode->contourX1 = contourX1;
        clonedNode->contourY1 = contourY1;
        clonedNode->contourX2 = contourX2;
        clonedNode->contourY2 = contourY2;
    }
    
    // Recursively clone children
    if (leftChild) {
        auto clonedLeftChild = leftChild->clone();
        clonedNode->setLeftChild(clonedLeftChild);
        clonedLeftChild->setParent(clonedNode);
    }
    
    if (rightChild) {
        auto clonedRightChild = rightChild->clone();
        clonedNode->setRightChild(clonedRightChild);
        clonedRightChild->setParent(clonedNode);
    }
    
    return clonedNode;
}