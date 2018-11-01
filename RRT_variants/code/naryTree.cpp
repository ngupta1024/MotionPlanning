#include <iostream>
#include "naryTree.hpp"

#include "helper.hpp"

using namespace std;

NodeNT* createNodeNT(NodeNT* parent,double* config){
    NodeNT* new_node=new NodeNT;
    for(int i=0;i<DOF;++i){
        new_node->config[i]=config[i];
    }
    new_node->first_child=new_node->right_sibling=NULL;
    new_node->parent=parent;
    return new_node;
}

NodeNT* addSibling(NodeNT* node, NodeNT* parent, double* config){
    if(node==NULL)
        return NULL;
    
    while(node->right_sibling){
        node=node->right_sibling;
    } 

    return (node->right_sibling=createNodeNT(parent,config));
}

NodeNT* addChild(NodeNT* parent_node, double* config){
    if(parent_node==NULL)
        return createNodeNT(NULL,config);

    if(parent_node->first_child)
        return(addSibling(parent_node->first_child,parent_node,config));
    else
        return(parent_node->first_child=createNodeNT(parent_node,config));
}

NodeNT* searchTree(NodeNT* root, double* config){
    NodeNT* searched_node=NULL;
    if(root==NULL){
        return NULL;
    }
    while(root){
        if(areConfigsSame(root->config,config)){
            return root;
        }
        if(root->first_child){
            searched_node= searchTree(root->first_child,config);
            if(searched_node!=NULL){
                return searched_node;
            }
        }
        root=root->right_sibling;
    }
    return NULL;
}

void insertNodeNT(NodeNT* root, double* config_parent, double* config_child){
    NodeNT* node_parent=searchTree(root, config_parent);
    addChild(node_parent, config_child);
}

void countNumNodes(NodeNT* root, int &num_nodes_NT){
    if (root==NULL)
        return;
    while(root){
        num_nodes_NT++;
        if(root->first_child)
            countNumNodes(root->first_child,num_nodes_NT);
        root=root->right_sibling;
    }
    return;
}

void replaceParent(NodeNT* root, double* config, NodeNT* new_parent){
    NodeNT* node_del=searchTree(root, config);
    if(areConfigsSame(node_del->parent->first_child->config, node_del->config)){
        node_del->parent->first_child=node_del->right_sibling;
        node_del->parent=new_parent;
        if(new_parent->first_child)
            new_parent->first_child->right_sibling=node_del;
        else
            new_parent->first_child=node_del;
    }
    else{
        //if it's not the first child, it will be one of the siblings of first child
        NodeNT* node_left_sibling=node_del->parent->first_child;
        while(!areConfigsSame(node_left_sibling->right_sibling->config, node_del->config)){
            node_left_sibling=node_left_sibling->right_sibling;
        }
        node_left_sibling->right_sibling=node_del->right_sibling;
        node_del->parent=new_parent;
        if(new_parent->first_child)
            new_parent->first_child->right_sibling=node_del;
        else
            new_parent->first_child=node_del;
    }
}

// void deleteTree(NodeNT* root){
//     while(root){
//         if(root->first_child){
//             root=root->first_child;
//         }
//         else{
//             if(root->right_sibling){
//                 root=root->right_sibling;
//                 delete root->parent->first_child;
//                 root->parent->first_child=root;
//             }
//             else{
//                 if(root->parent){
//                     root=root->parent;
//                     delete root->first_child;
//                 }
//                 else{
//                     root=NULL;
//                 }
//             }
//         }
//     }
// }

// int main(){
//     NodeNT* root_tree=NULL;
//     double arr1[DOF], arr2[DOF];
//     for(int i=0; i<DOF; ++i){
//         arr1[i]=0; 
//     }
//     root_tree=createNodeNT(NULL,arr1);
//     for(int i=0; i<DOF; ++i){
//         arr2[i]=PI/2; 
//     }
//     addChild(root_tree,arr2);
//     for (int i=0;i<DOF;++i){
//         cout<<root_tree->first_child->config[i]<<endl;
//     }
//     for(int i=0; i<DOF; ++i){
//         arr2[i]=-PI/2; 
//     }
//     addChild(root_tree,arr2);

//     for(int i=0; i<DOF; ++i){
//         arr2[i]=-0.1; 
//     }
//     addChild(root_tree,arr2);

//     for(int i=0; i<DOF; ++i){
//         arr2[i]=-0.2; 
//     }
//     addChild(root_tree,arr2);

//     for(int i=0; i<DOF; ++i){
//         arr2[i]=-0.2; 
//     }

//     for(int i=0; i<DOF; ++i){
//         arr1[i]=-0.4; 
//     }

//     insertNodeNT(root_tree, arr2, arr1);
//     int numNodes=0;
//     countNumNodes(root_tree,numNodes);
//     cout<<"numNodes"<<numNodes<<endl;
    
//     for(int i=0; i<DOF; ++i){
//         arr2[i]=-0.4; 
//     }
//     NodeNT* output= searchTree(root_tree,arr2);
//     for (int i=0;i<DOF;++i){
//         cout<<output->config[i]<<endl;
//     }

//     for(int i=0; i<DOF; ++i){
//         arr2[i]=-PI/2; 
//     }
//     for(int i=0; i<DOF; ++i){
//         arr1[i]=-0.2; 
//     }
//     output= searchTree(root_tree,arr1);
//     replaceParent(root_tree, arr2, output);
//     output= searchTree(root_tree,arr2);
//     cout<<output->parent->parent->first_child->right_sibling->right_sibling->config[0]<<endl;
//     return 0;
// }