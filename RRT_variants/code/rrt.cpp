#include <iostream>
#include <stdlib.h>
#include "rrt.hpp"
#include "kdtree.hpp"
#include "collisionDetection.hpp"
#include <math.h>
#include "helper.hpp"
#include <queue>
#include "graph.hpp"
using namespace std;

double* chooseTarget(double* config_goal){
    double* config_target=new double[DOF];
    // r is from 0 to 1
    float r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    if (r<=GOAL_PROB){
        for(int i=0; i<DOF;++i)
            config_target[i]=config_goal[i];
    }
    else{
        for(int i=0; i<DOF; i++){
            double rand_config = ((static_cast <double> (rand()) / static_cast <double> (RAND_MAX))*2*PI)-PI;
            // wrapToPI(rand_config);
            config_target[i]=rand_config;
        }
    }
    return config_target;
}

double* nearestNeighbor(NodeKDT* root, double* config_target){
    double* config_nearest=new double[DOF];
    NodeKDT* best_node=NULL;
    double best_dist;
    find_nearest(root, config_target, 0, &best_node, &best_dist);
    for(int i=0; i<DOF; ++i){
        config_nearest[i]=best_node->config[i];
        // cout<<best_node->config[i]<<endl;
    }
    return config_nearest;
}

std::vector<NodeKDT*> kNearestNeighbor(NodeKDT* root, double* config_target, 
                        priority_queue<NodeKDT*,std::vector<NodeKDT*>,ComparePQ> &pq)
{
    double best_dist=0;
    find_Knearest(root, config_target, 0, pq, &best_dist);
    // cout<<"size of PQ = "<<pq.size()<<endl;
    std::vector<NodeKDT*> kNN_vec;
    for(int i=0; i<pq.size();++i){
        NodeKDT* node_near=pq.top();
        pq.pop();
        double dist=configDiff(node_near->config,config_target);
        if(dist<=MAX_STEP)
            kNN_vec.push_back(node_near);
        else
            break;
    }
    // cout<<"size of kNN vec= "<<kNN_vec.size()<<endl;
    return kNN_vec;
    
}

double* extend( double* config_nearest, double* config_target, double *map, int x_size, int y_size, int* status){
    //status -- reached (0), advanced(1), trapped(2)
    double step=configDiff(config_nearest, config_target);
    double config_delta[DOF];
    double* config_new=new double[DOF];
    
    for(int i=0; i<DOF; ++i){
        wrapToPI(config_target[i]);
        config_delta[i]=angDiff(config_target[i],config_nearest[i]);
        if(step>MAX_STEP){
            double alpha=MAX_STEP/step;
            config_new[i]=(1-alpha)*config_nearest[i]+alpha*config_target[i];
            wrapToPI(config_new[i]);
            *status=1;
        }
        else{
            config_new[i]=config_nearest[i]+config_delta[i];
            wrapToPI(config_new[i]);
            *status=0;
        }
    }

    config_new=IsValidMovement(config_nearest, config_new, DOF, map, x_size, y_size, status);
    // cout<<"status after collision checking"<<*status<<endl;
    return config_new;
}

double* connect(NodeKDT* root,double* config_target, double *map, int x_size, int y_size, int* status){
    double* config_extend,*config_nearest;
    int i=0;
    *status=1;
    config_nearest=nearestNeighbor(root, config_target);
    while(true){
        config_extend=extend(config_nearest, config_target, map, x_size, y_size, status);
        // cout<<"connect 74: "<<*status<<endl;
        if(*status==2){
            cout<<"trapped connect"<<endl;
            break;
        } 
        
        if(*status==1){
            for (int i=0; i<DOF; ++i){
                config_nearest[i]=config_extend[i];
            }
        }
        
        if(*status==0){
            // cout<<"reached connect"<<endl;
            for (int i=0; i<DOF; ++i){
                config_extend[i]=config_target[i];
            }
            break;
        }
    }
    return config_extend;
}

double* randConfigFree(double* map, int x_size, int y_size){
    double* config_target=new double[DOF];
    double* config_collision=new double[DOF];
    int collision_detected=0;
    int status=0;
    float r = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    while(r<0.5){
        for(int i=0; i<DOF-1; i++){
            double rand_config = ((static_cast <double> (rand()) / static_cast <double> (RAND_MAX))*2*PI)-PI;
            config_target[i]=rand_config;
        }
        if(IsValidArmConfiguration(config_target, DOF, map, x_size, y_size) && collision_detected==0){
            return config_target;
        }
        if(IsValidArmConfiguration(config_target, DOF, map, x_size, y_size) && collision_detected==1){
            while(status!=2){
                config_target=extend( config_target, config_collision, map, x_size, y_size, &status);
            }
            if(IsValidArmConfiguration(config_target, DOF, map, x_size, y_size))  return config_target;
        }
        if(!IsValidArmConfiguration(config_target, DOF, map, x_size, y_size) && collision_detected==0){
            for(int i=0;i<DOF;i++){
                collision_detected=1;
                config_collision[i]=config_target[i];
            }
        }
    }
    while(r>=0.5){
        for(int i=0; i<DOF-1; i++){
            double rand_config = ((static_cast <double> (rand()) / static_cast <double> (RAND_MAX))*2*PI)-PI;
            config_target[i]=rand_config;
        }
        if(IsValidArmConfiguration(config_target, DOF, map, x_size, y_size))  return config_target;
    }
}

void dijkstra(double*  map, int x_size, int y_size, double* armstart_anglesV_rad, double* armgoal_anglesV_rad,
                    double*** plan, int* planlength, graph *roadmap){

    std::priority_queue<graphVertex*> openList;
    int extend_status=0;
    int i, j, k;
    int goal_found=0;
    int startID = (roadmap->gVertices).size() + 1;
    int endID = startID + 1;

    roadmap->addVertexGraph(armstart_anglesV_rad, startID);
    graphVertex *startNode = &(roadmap->gVertices).back();
    startNode->gValue = 0;

    roadmap->addVertexGraph(armgoal_anglesV_rad, endID);
    graphVertex *goalNode = &(roadmap->gVertices).back();

    for(std::list<graphVertex>::iterator it= (roadmap->gVertices).begin(); it != (roadmap->gVertices).end(); ++it)
    {
        if(configDiff(armstart_anglesV_rad, it->config) < RADIUS_PRM)
        {
            extend_status=0;
            IsValidMovement((it)->config,  armstart_anglesV_rad,DOF, map,  x_size,  y_size, &extend_status);
            if(extend_status!=2)
            {
                roadmap->addEdgeGraph(startNode, &(*it));
            }
        }
        if(configDiff(armgoal_anglesV_rad, it->config) < RADIUS_PRM)
        {
            extend_status=0;
            IsValidMovement((it)->config,  armgoal_anglesV_rad,DOF, map,  x_size,  y_size, &extend_status);
            if(extend_status!=2)
            {
                roadmap->addEdgeGraph(goalNode, &(*it));
            }
        }
    }

    openList.push(startNode);

    graphVertex* currentNode;
    graphVertex* connectingNode;

    double updatedCost = 0;

    while (!openList.empty())
    {
        currentNode = openList.top();
        currentNode->isClosed = true;
        openList.pop();

        if ((currentNode->connections).size()==0)
        {
            continue;
        }       
        for (std::list<graphEdge*>::iterator it= (currentNode->connections).begin(); it != (currentNode->connections).end(); ++it)
        {
            if(currentNode == (*it)->startAddress)
                connectingNode = (*it)->endAddress;
            else
                connectingNode = (*it)->startAddress;

            updatedCost = currentNode->gValue + (*it)->cost;

            if(!connectingNode->isClosed)
            {
                if( updatedCost < connectingNode->gValue)
                {
                    connectingNode->gValue = updatedCost;
                    connectingNode->parentAddress = currentNode;
                    openList.push(connectingNode); 
                }
            }
        }
        if(currentNode==goalNode){
            // cout<<"goal found"<<endl;
            goal_found=1;
            break;
        }
    }
    if(goal_found){
        std::list<graphVertex*> pathVertices;
        pathVertices.push_back(goalNode);
        graphVertex *predecessor = (pathVertices.back())->parentAddress;

        while(!areConfigsSame(predecessor->config, startNode->config))
        {
            pathVertices.push_back(predecessor);
            predecessor = (pathVertices.back())->parentAddress;
        }
        pathVertices.push_back(predecessor);

        *planlength = pathVertices.size();

        *plan = (double**) malloc((*planlength) *sizeof(double*));
        i = 0;
        // cout<<"almost there"<<endl;
        for (std::list<graphVertex*>::iterator it = --pathVertices.end(); it != --pathVertices.begin(); --it)
        {
            (*plan)[i] = (double*) malloc(DOF*sizeof(double)); 
            for(j = 0; j < DOF; j++)
            {
                (*plan)[i][j] = (*it)->config[j];
            }
            i++;
        }
    }
    return;
}
