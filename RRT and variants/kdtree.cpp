#include "kdtree.h"

using namespace std;
std::vector <point_t> pvec;

double compare (const point_t &p, const point_t &q, int dim) {
    
    switch (dim) {
        case XDIM : return p.x - q.x;
        case YDIM : return p.y - q.y;
        default : printf("error");
                  return -1;
    }
}

bool compare_x (const int p, const int q) {
    return (pvec[p].x < pvec[q].x);
}

bool compare_y (const int p, const int q) {
    return (pvec[p].y < pvec[q].y);
}

bool comparept_x (const point_t &p, const point_t &q) {
    return (p.x < q.x);
}


bool comparept_y (const point_t &p, const point_t &q) {
    return (p.y < q.y);
}

double distance_2d (point_t one, point_t two) {

    return sqrt(pow((one.x - two.x), 2) + pow((one.y - two.y) ,2));
}


kdnode_t* makeKDNode (double x, double y, int dim, int count) {
        
    kdnode_t *kdnode = (kdnode_t *) malloc (sizeof(kdnode_t));
    kdnode->pt       = (point_t *) malloc (sizeof(point_t)); 
    kdnode->pt->x    = x; 
    kdnode->pt->y    = y;
    kdnode->dim      = dim;
    kdnode->count    = count;
    kdnode->left     = NULL;
    kdnode->right    = NULL;
    return kdnode;
}

stnode_t *makeSTNode (int first, int last, kdnode_t *node, bool child, int dim) {

    stnode_t *stnode = (stnode_t *) malloc (sizeof(stnode_t));
    stnode->first    = first;
    stnode->last     = last;
    stnode->node     = node;
    stnode->child    = child;
    stnode->dim      = dim;

    return stnode;
};

void KDTree :: construct (bool recursive) {
    
    if (pvec.size() == 0)
        return;
   
    if (recursive) {
         root = buildKDTree_recurse (pvec, XDIM);
    } else {
        std::vector <int> indexes;
        for(int i = 0; i < (int )pvec.size(); ++i)
            indexes.push_back(i);
         
        buildKDTree (indexes, XDIM, &root);
    }
}


void buildKDTree (std::vector <int> &indexes, int dim, kdnode_t **root) {
    kdnode_t *kdnode = NULL; 
    stnode_t *nd = NULL;

    if (indexes.size() == 0)
        return;

    if (dim == XDIM)
        sort (indexes.begin(), indexes.end(), compare_x);
    else
        sort (indexes.begin(), indexes.end(), compare_y);

    std::queue <stnode_t> squeue;
    int split = indexes.size()/2;
    *root = makeKDNode (pvec[indexes[split]].x, pvec[indexes[split]].y, dim, indexes.size());     

    if (split - 0) {
        nd = makeSTNode (0, split, *root, LEFT, dim^1);  
        squeue.push(*nd); 
    } 
    
    if (indexes.size() - (split + 1) > 0) {
        nd = makeSTNode (split + 1, indexes.size(), *root, RIGHT, dim^1);  
        squeue.push(*nd); 
    }

    while (!squeue.empty()) {
        
        int first        = squeue.front().first;
        int last         = squeue.front().last;
        int dim          = squeue.front().dim;
        int child        = squeue.front().child;
        kdnode_t *parent = squeue.front().node;
        squeue.pop();
        
        //printf ("\n First: %d, Last : %d, dim: %d", first, last, dim); 
        if (last - first == 1) {
            kdnode = makeKDNode (pvec[indexes[first]].x, pvec[indexes[first]].y, dim, 1);     
        
            if (parent) {
                if (child == LEFT)
                     parent->left = kdnode;
                 else
                     parent->right = kdnode;
            }
        
        } else {
        
            if (dim == XDIM)
                sort (indexes.begin() + first, indexes.begin() + last, compare_x);
            else
                sort (indexes.begin() + first, indexes.begin() + last, compare_y);
                
            split = (first + last)/2;
            kdnode = makeKDNode (pvec[indexes[split]].x, pvec[indexes[split]].y, dim, last - first);
             
            if (parent) {
                if (child == LEFT)
                     parent->left = kdnode;
                 else
                     parent->right = kdnode;
            }
            
            int first1 = first, last1 = split, first2 = split + 1, last2 = last;
            
            //printf ("\n left size %d, right %d", (int )leftvec.size(), (int)rightvec.size());
            if (last1 - first1 > 0 && first1 >= 0) {
                 nd = makeSTNode (first1, last1, kdnode, LEFT, dim^1);  
                 squeue.push(*nd); 
            }
                  
            if (last2 - first2 > 0 && last2 <= (int )pvec.size()) {
                 nd = makeSTNode (first2, last2, kdnode, RIGHT, dim^1);  
                 squeue.push(*nd); 
            }
        }
    }
}


kdnode_t *buildKDTree_recurse (std::vector <point_t> &pvec, int dim) {
    kdnode_t *kdnode = NULL; 
    
    //printf ("\n\n pvec size %d, dimension %d", (int )pvec.size(), dim);
    
    if (pvec.size() == 1) {
        kdnode = makeKDNode (pvec[0].x, pvec[0].y, dim, pvec.size());     
    
    } else {
        if (dim == XDIM)
            sort (pvec.begin(), pvec.end(), comparept_x);
        else
            sort (pvec.begin(), pvec.end(), comparept_y);
            
       int split = ((int )pvec.size())/2;
       kdnode = makeKDNode (pvec[split].x, pvec[split].y, dim, pvec.size());
       
       std::vector <point_t> leftvec, rightvec;
       for (int it = 0; it < (int )pvec.size(); ++it) {
            
            if (it != split) {
                if (compare(pvec[it], pvec[split], dim) < 0) {
                    leftvec.push_back(pvec[it]);
                
                } else if (compare(pvec[it], pvec[split], dim) > 0) {
                
                    rightvec.push_back(pvec[it]);
                }  else {
                   if (it < split) 
                       leftvec.push_back(pvec[it]);
                   else  
                       rightvec.push_back(pvec[it]);
                } 
            }
       }
        
       //printf ("\n left size %d, right %d", (int )leftvec.size(), (int)rightvec.size());
       if (leftvec.size() > 0) 
           kdnode->left = buildKDTree_recurse (leftvec, dim^1);
   
       if (rightvec.size() > 0) 
           kdnode->right = buildKDTree_recurse (rightvec, dim^1);
    
        leftvec.clear();
        rightvec.clear();
    }
    
    return kdnode;
}


void query (kdnode_t *node, double xmin, double xmax, double ymin, double ymax, std::vector <point_t> &result) {
    
    if (node == NULL)
        return;
   
    point_t *nd = node->pt;

    if (xmin <= nd->x && xmax >= nd->x && ymin <= nd->y && ymax >= nd->y) 
        result.push_back(*nd);
    
    switch (node->dim) {
        case 0 : {      //printf ("\n xdim, nd->x : %f, xmin: %f, xmax: %f", nd->x, xmin, xmax); 
                        if (xmin <= nd->x)
                            query (node->left, xmin, xmax, ymin, ymax, result);
                        
                        if (xmax >= nd->x)
                            query (node->right, xmin, xmax, ymin, ymax, result);
                        
                        break;
                 }
        case 1 : {      //printf ("\n ydim, nd->y : %f, ymin: %f, ymax: %f", nd->x, xmin, xmax); 
                        if (ymin <= nd->y)
                            query (node->left, xmin, xmax, ymin, ymax, result);
                        
                        if (ymax >= nd->y)
                            query (node->right, xmin, xmax, ymin, ymax, result);
                        
                        break; 
                 }

        default : printf("error"); break;
    }
}

void rectangle_query (point_t point_one, point_t point_two, KDTree kd, std::vector <point_t> &result) {
   
   double xmin, xmax, ymin, ymax;
    
   xmin = (point_one.x < point_two.x) ? point_one.x : point_two.x;
   xmax = (point_one.x > point_two.x) ? point_one.x : point_two.x;
   ymin = (point_one.y < point_two.y) ? point_one.y : point_two.y;
   ymax = (point_one.y > point_two.y) ? point_one.y : point_two.y;

   //printf("\n xmax : %f, xmin : %f, ymax : %f, ymin : %f", xmax, xmin, ymax, ymin); 
   query (kd.root, xmin, xmax, ymin, ymax, result);
}

unsigned long rectangle_count (point_t point_one, point_t point_two, KDTree kd) {

   std::vector <point_t> result;
   rectangle_query (point_one, point_two, kd, result);

   return result.size();
}

void circle_query (point_t point_one, double radius, KDTree kd, std::vector <point_t> &result) {
    
   double xmin, xmax, ymin, ymax;
   std::vector <point_t> temp;
    
   xmin = point_one.x - radius;
   xmax = point_one.x + radius;
   ymin = point_one.y - radius;
   ymax = point_one.y + radius;
   
   query (kd.root, xmin, xmax, ymin, ymax, temp);
   
   for (int i = 0; i < (int )temp.size(); ++i) {
        if (distance_2d(temp[i], point_one) <= radius) {
            result.push_back(temp[i]);
        }
   }
}

unsigned long circle_count (point_t point_one, double radius, KDTree kd) {

   std::vector <point_t> result;
   circle_query (point_one, radius, kd, result);
   return result.size();
}


bool find (kdnode_t* curr, point_t p, int dim) {
   
    double cmp = compare(*(curr->pt), p, dim);
    
    if (cmp == 0) {
        int cmp2 = compare(*(curr->pt), p, dim^1); 
        if (cmp2 == 0)
            return true;
        
        if (cmp2 < 0)
            return (find (curr->left, p, dim^1));
        else
            return (find (curr->right, p, dim^1));
    
    } else if (cmp < 0) {
        return (find (curr->left, p, dim^1));
    } else {
        return (find (curr->right, p, dim^1));
    }

    return false;
}


bool isPresent (point_t p, KDTree kd) {
    
    if (find(kd.root, p, XDIM))
        return true;
        
    return false;
}


double find_near_temp (kdnode_t *curr, point_t p, int dim) {

    double cmp = compare(*(curr->pt), p, dim);
   
    if(curr->left == NULL || curr->right == NULL) 
        return cmp;

    if (cmp == 0) {
        double cmp2 = compare(*(curr->pt), p, dim^1); 
        if (cmp2 == 0)
            return cmp;
        
        if (cmp2 < 0)
            return (find_near_temp (curr->left, p, dim^1));
        else
            return (find_near_temp (curr->right, p, dim^1));
    
    } else if (cmp < 0) {
        return (find_near_temp (curr->left, p, dim^1));
    } else {
        return (find_near_temp (curr->right, p, dim^1));
    }
}


void find_nearest (kdnode_t* curr, point_t p, int dim, double &min, point_t *pt) {

    if (!curr)
        return;

    double cmp = compare(*(curr->pt), p, dim);
    
    if (cmp < min) {
        min = cmp;
        *pt = *(curr->pt);
    }
    
    if (cmp == 0) {
        double cmp2 = compare(*(curr->pt), p, dim^1); 
        if (cmp2 == 0)
            return;
        
        if (cmp2 < 0)
            find_nearest (curr->left, p, dim^1, min, pt);
        else
            find_nearest (curr->right, p, dim^1, min, pt);
    
    } else if (cmp < 0) {
        find_nearest (curr->left, p, dim^1, min, pt);
    } else {
        find_nearest (curr->right, p, dim^1, min, pt);
    }

}

point_t* nearest (point_t p, KDTree kd) {
    
    point_t *p1 = (point_t *) malloc (sizeof (point_t));

    double min = find_near_temp (kd.root, p, XDIM);
    
    if (min == 0) {
        cout << "point exist"; 
        p1->x = p.x;
        p1->y = p.y;
        return p1; 
    }

    find_nearest (kd.root, p, XDIM, min, p1);
    return p1;
}

void traversal (KDTree kd) {
   std::queue <kdnode_t> kdqueue;
   
   kdqueue.push(kd.getroot());

   while (!kdqueue.empty()) {
        kdnode_t kdn = kdqueue.front();
        kdqueue.pop();
        
        printf("\n val (%f, %f)", (kdn.pt)->x, (kdn.pt)->y);
        if(kdn.left)
            kdqueue.push(*(kdn.left)); 
        if(kdn.right) 
            kdqueue.push(*(kdn.right)); 
   }
}

void print_pvec () {
      
    for (int i = 0; i < (int )pvec.size(); ++i) 
        printf("\n (%f, %f)", pvec[i].x, pvec[i].y);
}