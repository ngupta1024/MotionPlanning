#ifndef RRT_H
#define RRT_H

#include "kdtree.hpp"
#include "helper.hpp"
#include <vector>
#include <queue>
#include "graph.hpp"
const float MAX_STEP=0.8;
const float RADIUS_PRM=1.7;
const float MIN_STEP=0.1;
const float GOAL_PROB=0.1;

double* chooseTarget(double* config_goal);
double* nearestNeighbor(NodeKDT* root, double* config_target);
std::vector<NodeKDT*> kNearestNeighbor(NodeKDT* root, double* config_target, std::priority_queue<NodeKDT*,std::vector<NodeKDT*>,ComparePQ> &pq);
double* extend(double* config_nearest, double* config_target,double *map, int x_size, int y_size, int* status);
double* connect(NodeKDT* root,double* config_target, double *map, int x_size, int y_size, int* status);
double* randConfigFree(double* map, int x_size, int y_size);
void dijkstra(double*  map, int x_size, int y_size, double* armstart_anglesV_rad, double* armgoal_anglesV_rad,
                    double*** plan, int* planlength, graph *roadmap);
#endif