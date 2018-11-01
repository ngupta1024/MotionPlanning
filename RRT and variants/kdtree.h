/*
* Functions of KDTree :
* 
* buildKDTree - builds KDTree optimally in non-recursive way by using queue and 
*               presorting the points based on indices.
*
* buildKDTree_recurse - builds KDTree in simple recursive manner and sorts the points themselves rather
*                       than sorting them on indices.
* 
* isPresent           - Use to check if point exists in KDTree
* rectangle_query     - Use to report points if they lie within given orthogonal range query    
* rectangle_count     - Use to count the points in given range rectangle 
* circle_query        - Use to report points if they lie within given circle 
* circle_count        - Use to count the points in given circle 
*/

#ifndef KDTREE
#define KDTREE

#include <stdio.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <stdlib.h>
#include <ctime>
#include <stack>
#include <queue>
#include <math.h>
#include <sys/time.h>

#define TIME_CAL 1

enum dim {XDIM, YDIM  };
enum ch  {LEFT, RIGHT };

typedef struct config {
    double x;
    double y;
} point_t;
 
typedef struct KDNode {
    point_t *pt; 
    int dim;
    int count;
    struct KDNode *left;
    struct KDNode *right;
} kdnode_t;

class KDTree {
    public:
    kdnode_t *root;
    int min, max;
    void construct (bool recursive);
    kdnode_t getroot() { return *root; }
};

typedef struct stack_node {
    int first;
    int last;
    bool child;
    int dim;
    kdnode_t *node;
} stnode_t;

extern std::vector <point_t> pvec;

bool isPresent                (point_t p, KDTree kd);
void rectangle_query          (point_t point_one, point_t point_two, KDTree kd, std::vector <point_t> &result);
unsigned long rectangle_count (point_t point_one, point_t point_two, KDTree kd);
void circle_query             (point_t point_one, double radius, KDTree kd, std::vector <point_t> &result);
unsigned long circle_count    (point_t point_one, double radius, KDTree kd);
point_t* nearest              (point_t p, KDTree kd);

kdnode_t* makeKDNode          (double x, double y, int dim, int count);
stnode_t *makeSTNode          (int first, int last, kdnode_t *node, bool child, int dim);
void buildKDTree              (std::vector <int> &indexes, int dim, kdnode_t **root);
kdnode_t *buildKDTree_recurse (std::vector <point_t> &pvec, int dim);

void traversal  (KDTree kd);
void print_pvec ();

#endif