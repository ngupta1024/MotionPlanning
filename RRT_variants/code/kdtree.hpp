#ifndef KDTREE_H
#define KDTREE_H

#include <iostream>
#include <math.h>
#include "helper.hpp"
#include <queue>

// A structure to represent node of kd tree 
struct NodeKDT{
    double config[DOF];
    NodeKDT *left, *right, *parent;
};

struct SearchKDT{
    bool isThere;
    NodeKDT *node;
};

class ComparePQ{
    double* config_query;
public:
    ComparePQ(double* config){
        config_query=new double[DOF];
        for(int i=0; i<DOF; ++i){
            config_query[i]=config[i];
        }
    }
    bool operator()(NodeKDT* node1, NodeKDT* node2){
        double dist1=configDiff(node1->config,config_query);
        double dist2=configDiff(node2->config,config_query);
        return dist1>dist2;

    }
};

NodeKDT* newNode(double *config,NodeKDT* parent);
NodeKDT *insertKDTree(NodeKDT *root,NodeKDT* parentRoot, double *config, int depth);
SearchKDT* search(NodeKDT* root, double *config);
void find_nearest(NodeKDT* root, double *config, int depth, NodeKDT** best_node, double* best_dist);
void find_Knearest(NodeKDT* root, double *config, int depth, std::priority_queue<NodeKDT*, std::vector<NodeKDT*>,ComparePQ> &best_nodes_pq, double* best_dist);
void deleteChildren(NodeKDT *root);
#endif