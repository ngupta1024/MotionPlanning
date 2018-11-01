#include <iostream>
#include <math.h>
#include "kdtree.hpp"
#include "rrt.hpp"
#include "collisionDetection.hpp"
#include "helper.hpp"
#include <queue>

using namespace std;

// A method to create a node of K D tree 
NodeKDT* newNode(double *config, NodeKDT* parent){
    NodeKDT* temp=new NodeKDT;
    for(int i=0; i<DOF; ++i)
        temp->config[i]=config[i];
    temp->left=temp->right=NULL;
    temp->parent=parent;
    return temp;
}

// Inserts a new node recursively
// @param depth  to decide axis of comparison
// @return root of modified tree
NodeKDT *insertKDTree(NodeKDT *root, NodeKDT* parentRoot, double *config, int depth){
    // cout<<"new node being inserted"<<endl;
    if (root==NULL){
        return newNode(config,parentRoot);
    }
    int comparison_dim=depth%DOF;
    
    if(config[comparison_dim]< (root->config[comparison_dim])){
        root->left=insertKDTree(root->left,root,config,depth+1);
    }
    else{
        root->right=insertKDTree(root->right,root,config,depth+1);
    }
    return root;
}

// Searches a Config in the KD-tree
SearchKDT* searchRec(NodeKDT* root, double *config, int depth){
    SearchKDT *output_struct=new SearchKDT;
    if (root==NULL){
        output_struct->isThere=false;
        output_struct->node=root;
        return output_struct;
    }
    if (areConfigsSame(root->config,config)){
        output_struct->isThere=true;
        output_struct->node=root;
        return output_struct;
    }
    int comparison_dim=depth%DOF;
    if (config[comparison_dim]<root->config[comparison_dim])
        return searchRec(root->left,config,depth+1);
    return searchRec(root->right, config, depth+1);
}

SearchKDT* search(NodeKDT* root, double *config){
    return searchRec(root,config,0);
}

void find_nearest(NodeKDT* root, double *config, int depth, NodeKDT** best_node, double* best_dist){
    if(root==NULL){
        return;
    }
    
    int comparison_dim=depth%DOF;
    double dist=configDiff(root->config,config);
    double dx=angDiff(root->config[comparison_dim], config[comparison_dim]);
    double dx2=dx*dx;

    if(*best_node==NULL || dist<*best_dist){
        *best_node=root;
        *best_dist=dist;    
    }
    if(dist==0){
        return;
    }
    find_nearest(dx>0 ? root->left : root->right,config,depth+1,best_node, best_dist);
    if(dx2>=*best_dist){
        return;
    }
    find_nearest(dx>0 ? root->right : root->left, config, depth+1, best_node, best_dist);
}


void find_Knearest(NodeKDT* root, double *config, int depth, priority_queue<NodeKDT*, std::vector<NodeKDT*>,ComparePQ> &best_nodes_pq,double* best_dist){
    
    if(root==NULL){
        return;
    }
    
    int comparison_dim=depth%DOF;
    double dist=configDiff(root->config,config);
    if(depth==0 || dist<*best_dist){
        *best_dist=dist;
    }
    double dx=angDiff(root->config[comparison_dim], config[comparison_dim]);
    double dx2=dx*dx;

    best_nodes_pq.push(root);

    find_Knearest(dx>0 ? root->left : root->right,config,depth+1,best_nodes_pq, best_dist);
    if(dx2>=*best_dist){
        return;
    }
    find_Knearest(dx>0 ? root->right : root->left, config, depth+1, best_nodes_pq,best_dist);
}


void deleteChildren(NodeKDT *root)
{
    // Recurse left down the tree...
    if(root->left) deleteChildren(root->left);
    // Recurse right down the tree...
    if(root->right) deleteChildren(root->right);

    delete root;
}

////////////////////////////////////////////////////////////////////////////////////////////////
// int main(){
//     NodeKDT* main_tree;
//     double* arr=new double[DOF];
//     for(int i=0; i<DOF; ++i){
//         arr[i]=0; 
//     }
//     NodeKDT* parent=NULL;
//     main_tree=newNode(arr,parent);
//     for(int i=0; i<DOF; ++i){
//         arr[i]=PI/2; 
//     }
//     main_tree=insertRec(main_tree, parent, arr, 0);
//     for(int i=0; i<DOF; ++i){
//         arr[i]=-PI/2; 
//     }
//     main_tree=insertRec(main_tree, parent, arr, 0);

//     double* query=new double[DOF];
//     for(int i=0; i<DOF; ++i){
//         query[i]=0; 
//     }
//     NodeKDT* found=NULL;
//     double best_dist;
//     find_nearest(main_tree, query, 0, &found, &best_dist);
//     SearchKDT* find_exact=search(main_tree, query);
//     cout<<"search: "<<find_exact->isThere<<endl;
//     for(int i=0; i<DOF; ++i){
//         cout<<found->config[i]<<endl;
//     }
//     cout<<best_dist<<endl;
// 	    delete[] arr;
// 	    delete[] query;
//     return 0;
// }