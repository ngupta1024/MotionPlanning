#ifndef ASTAR_H
#define ASTAR_H

#include <vector>
#include <set>
#include <queue> 
#include <unordered_set>
class Generator
{
public:
    struct Vec2i
    {
        int x;
        int y;
        bool operator == (const Vec2i& coordinates_);
    };
    struct Node
    {
        int G,H;
        Vec2i coordinates;
        Node *parent;
        Node(Vec2i coord_, Node *parent_ = nullptr);
        int getFValueFwd();
    };
    struct ByFValfwd
    {
        //first sort: according to Fval
        //second sort: according to x coordinate
        //third sort: according to y coordinate
        bool operator() (const Node* lhs, const Node* rhs)
        {
            if (lhs->G+weight_fwd*lhs->H==rhs->G+weight_fwd*rhs->H)
            {
                if (lhs->coordinates.x==rhs->coordinates.x)
                {
                    return lhs->coordinates.y<rhs->coordinates.y;
                }
                return lhs->coordinates.x<rhs->coordinates.x;
            }
            else{
                return lhs->G+weight_fwd*lhs->H<rhs->G+weight_fwd*rhs->H;
            }
            
        }
    };

    typedef std::set<Node*, ByFValfwd> NodeSetFwd;
    typedef std::unordered_set<std::string> UnorderedSet;
    typedef NodeSetFwd::const_iterator FwdNodeSetIt;
    typedef std::vector<Vec2i> coordinateList;

    //normal A* and its supporting functions
    Generator(int x_size, int y_size, int collision_thresh_);
    bool detectCollision(double* map, Vec2i coordinates_);
    Node* findNodeOnList(NodeSetFwd& nodes_, Vec2i coordinates_);
    void releaseNodes(NodeSetFwd& nodes_);
    void releaseNodes(UnorderedSet& nodes_);
    coordinateList findPath(double* map, int robotposeX, int robotposeY, int goalposeX,int goalposeY);
    //heuristics and its supporting functions
    // int backwardHeuristic(double *map, int coordinateX, int coordinateY, int goalposeX,int goalposeY);
    int eightConnectedHeuristic(double *map, int coordinateX, int coordinateY, int goalposeX,int goalposeY);
    int euclideanHeuristic(double *map, int coordinateX, int coordinateY, int goalposeX,int goalposeY);
private:
    int collision_thresh;
    coordinateList direction={
        { -1, -1 }, { -1, 0 }, { -1, 1 }, { 0, -1 },
        { 0, 1 }, { 1, -1 }, { 1, 0 }, { 1, 1 } 
    };
    Vec2i worldSize;
    int num_dirs=8;
    static const int weight_fwd=1;
    static const int weight_bwd=1;
    Vec2i getDelta(Vec2i source_, Vec2i target_);
};

#endif