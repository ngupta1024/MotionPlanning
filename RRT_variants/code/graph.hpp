#ifndef GRAPH__
#define GRAPH__

#include <list>
#include "helper.hpp"

struct graphEdge;

typedef struct graphVertex{
    double config[DOF];
    int vertexID;
    int numConnected;
    bool isClosed;
    double gValue;
    std::list<graphEdge*> connections;
    graphVertex* parentAddress;

    bool operator<(graphVertex & newVertex) const
    {
    	return gValue < newVertex.gValue; 
    }

}graphVertex;

typedef struct graphEdge{
    graphVertex *startAddress;
    graphVertex *endAddress;
    double cost;
}graphEdge;

class graph
{
public:
	void addVertexGraph(double *config, int idNumber);
	void addEdgeGraph(graphVertex *newVertex, graphVertex *nearVertex);
	void createNeighborhoodGraph(graphVertex **newVertex, std::list<graphVertex*> *neighborHood);
public:
	std::list<graphVertex> gVertices;
	std::list<graphEdge> gEdges;
};

#endif