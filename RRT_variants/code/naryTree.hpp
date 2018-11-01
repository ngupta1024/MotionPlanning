#ifndef NTREE_H
#define NTREE_H

#include "helper.hpp"

struct NodeNT{
    double config[DOF];
    NodeNT* first_child;
    NodeNT* parent;
    NodeNT* right_sibling;
};

NodeNT* createNodeNT(NodeNT* parent,double* config);
NodeNT* addSibling(NodeNT* node, NodeNT* parent, double* config);
NodeNT* addChild(NodeNT* parent_node, double* config);
NodeNT* searchTree(NodeNT* root, double* config);
void insertNodeNT(NodeNT* root, double* config_parent, double* config_child);
void countNumNodes(NodeNT* root, int &num_nodes_NT);
void replaceParent(NodeNT* root, double* config, NodeNT* new_parent);
// void deleteTree(NodeNT* root);
#endif