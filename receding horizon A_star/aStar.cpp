#include <iostream>
#include <math.h>
#include <vector>
#include "aStar.hpp"
#include <algorithm>
#include <queue>
#include <unordered_map> 
#include <limits>
#include <string>

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

bool Generator::Vec2i::operator == (const Vec2i& coordinates_)
{
    return (x == coordinates_.x && y == coordinates_.y);
}

Generator::Vec2i operator + (const Generator::Vec2i& left_, const Generator::Vec2i& right_)
{
    return{ left_.x + right_.x, left_.y + right_.y };
}

//unused in forward A*
Generator::Vec2i operator - (const Generator::Vec2i& left_, const Generator::Vec2i& right_)
{
    return{ left_.x - right_.x, left_.y - right_.y };
}

Generator::Node::Node(Vec2i coordinates_, Node *parent_)
{
    parent = parent_;
    coordinates = coordinates_;
    G = H = 0;
}

int Generator::Node::getFValueFwd()
{
    return G+weight_fwd*H;
}

Generator::Generator( int x_size, int y_size, int collision_thresh_)
{
    collision_thresh=collision_thresh_;
    worldSize={x_size,y_size};
}

bool Generator::detectCollision(double* map,Vec2i coordinates_)
{
    //should be in the bounds and below collision threshold
    if(coordinates_.x>=1 && coordinates_.y>=1 && coordinates_.x<=worldSize.x && coordinates_.y<=worldSize.y &&
        ((int)map[GETMAPINDEX(coordinates_.x,coordinates_.y,worldSize.x,worldSize.y)] < collision_thresh))
    {
            return false; //no collision
    } 
    return true; //collision
}

Generator::Node* Generator::findNodeOnList(NodeSetFwd& nodes_, Vec2i coordinates_)
{
    for (auto node : nodes_) {
        if (node->coordinates == coordinates_) {
            return node;
        }
    }
    return nullptr;
}

void Generator::releaseNodes(NodeSetFwd& nodes_)
{
    for (auto it = nodes_.begin(); it != nodes_.end();) 
    {
        delete *it;
        it = nodes_.erase(it);
    }
}

Generator::Vec2i Generator::getDelta(Vec2i source_, Vec2i target_)
{
    return{ abs(source_.x - target_.x),  abs(source_.y - target_.y) };
}


int Generator::eightConnectedHeuristic(double *map, int coordinateX, int coordinateY, int goalposeX,int goalposeY)
{
    Vec2i coordinates_={coordinateX,coordinateY};
    Vec2i goal_pos_={goalposeX,goalposeY};
    auto delta = std::move(getDelta(coordinates_, goal_pos_));
    auto min_of_both=std::min(delta.x,delta.y);
    auto max_of_both=std::max(delta.x,delta.y);
    return static_cast<int>(min_of_both*sqrt(2)+(max_of_both-min_of_both));
}


Generator::coordinateList Generator::findPath(double* map, int robotposeX, int robotposeY, int goalposeX,int goalposeY)
{
    Vec2i start_pos = {robotposeX,robotposeY};
    Vec2i goal_pos  = {goalposeX,goalposeY};
    NodeSetFwd openSet;
    UnorderedSet closedSet;
    Node *current = nullptr;
    openSet.insert(new Node(start_pos));

    while(!openSet.empty())
    {
        //Select node with the highest priority and least f value in the openset
        //check for tie break and if there is a goal state
        current=*openSet.begin();//the one with least f value   
        // insert the current node to be expanded in closed list
        closedSet.insert(std::to_string(current->coordinates.x)+","+std::to_string(current->coordinates.y));
        // remove the current node to be expanded from open list
        openSet.erase(std::find(openSet.begin(), openSet.end(),current));

        //seach for a node that has not been expanded and is collision free
        for(int i=0; i<num_dirs; ++i)
        {
            Vec2i newCoordinates(current->coordinates+direction[i]);
            if (detectCollision(map,newCoordinates)
                ||closedSet.find(std::to_string(newCoordinates.x)+","+std::to_string(newCoordinates.y))!=closedSet.end())
            {
                continue;
            }
            //each step is equal to cost of 1
            int cost_to_go=((int)map[GETMAPINDEX(newCoordinates.x,newCoordinates.y,worldSize.x,worldSize.y)]>2000)? 50:1;
            int tentative_g=current->G+cost_to_go;
            Node *successor=findNodeOnList(openSet,newCoordinates);
            
            //if successor is not in openlist, then add it with tentative g value
            if (successor==nullptr)
            {
                successor=new Node(newCoordinates,current);
                successor->G = tentative_g;
                //check if successor is already expanded by dijkstra
                successor->H = weight_fwd*eightConnectedHeuristic(map, newCoordinates.x, newCoordinates.y, goalposeX,goalposeY);
                openSet.insert(successor);
            }
            //if successor is in openlist, then see and update g value if needed
            else if(tentative_g<successor->G)
            {
                successor->parent=current;
                successor->G=tentative_g;
            }
        }
        //we only want to quit after expanding goal state
        if (current->coordinates==goal_pos)
        {
            break;
        }
    }
    std::cout<<"expansions done by A star: "<<closedSet.size()<<std::endl;
    coordinateList path;
    while(current!=nullptr)
    {
        path.push_back(current->coordinates);
        current=current->parent;
    }

    releaseNodes(openSet);
    return path;
}


/****************end of usable code**************************************

std::unordered_map<std::string,int> mymap_closedSet;

int Generator::backwardHeuristic(double *map, int coordinateX, int coordinateY, int goalposeX,int goalposeY)
{
    Vec2i goal_pos_ = {coordinateX,coordinateY};
    Vec2i start_pos_= {goalposeX,goalposeY};
    NodeSetFwd openSet;
    
    Node *current = nullptr;
    openSet.insert(new Node(start_pos_));

    while(!openSet.empty())
    {
        current=*openSet.begin();//the one with least f value
        // insert the current node to be expanded in closed list
        mymap_closedSet.insert({std::to_string(current->coordinates.x)+","+std::to_string(current->coordinates.y),current->G});
        // remove the current node to be expanded from open list
        openSet.erase(std::find(openSet.begin(), openSet.end(),current));
        //seach for a node that has not been expanded and is collision free
        for(int i=0; i<num_dirs; ++i)
        {
            Vec2i newCoordinates(current->coordinates-direction[i]);
            if (detectCollision(map,newCoordinates)
                ||mymap_closedSet.find(std::to_string(newCoordinates.x)+","+std::to_string(newCoordinates.y))!=mymap_closedSet.end())
            {
                continue;
            }
            //each step is equal to cost of 1
            int tentative_g=current->G+1;
            Node *predecessor=findNodeOnList(openSet,newCoordinates);
            
            //if predecessor is not in openlist, then add it with tentative g value
            if (predecessor==nullptr)
            {
                predecessor=new Node(newCoordinates,current);
                predecessor->G = tentative_g;
                predecessor->H = weight_bwd*eightConnectedHeuristic(map,predecessor->coordinates.x,predecessor->coordinates.y, goal_pos_.x,goal_pos_.y );
                openSet.insert(predecessor);
            }
            //if predecessor is in openlist, then see and update g value if needed
            else if(tentative_g<predecessor->G)
            {
                predecessor->parent=current;
                predecessor->G=tentative_g;
            }
        }
        //we only want to quit after expanding goal state
        if (current->coordinates==goal_pos_)
        {
            releaseNodes(openSet);
            return current->G;
        }
    }
    releaseNodes(openSet);
    return std::numeric_limits<int>::max();
}




int Generator::euclideanHeuristic(double *map, int coordinateX, int coordinateY, int goalposeX,int goalposeY)
{   
    Vec2i coordinates_={coordinateX,coordinateY};
    Vec2i goal_pos_={goalposeX,goalposeY};
    auto delta = std::move(getDelta(coordinates_, goal_pos_));
    return static_cast<int>(sqrt(pow(delta.x, 2) + pow(delta.y, 2)));
}
*/