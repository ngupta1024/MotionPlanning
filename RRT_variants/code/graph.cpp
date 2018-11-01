#include <iostream>
#include <math.h>
#include "helper.hpp"
#include "graph.hpp"
#include "rrt.hpp"

using namespace std;
void graph::addVertexGraph(double *config, int idNumber)
{	
	graphVertex newVertex;
	for(int i = 0; i < DOF; ++i)
		newVertex.config[i] = config[i];

	newVertex.vertexID = idNumber;
	newVertex.numConnected = 0;
	newVertex.isClosed = false;
	newVertex.gValue = 10000.0;
	newVertex.parentAddress = NULL;
	gVertices.push_back(newVertex);
	return;
}

void graph::addEdgeGraph(graphVertex *newVertex, graphVertex *nearVertex)
{
	graphEdge newEdge;
	newEdge.startAddress = newVertex;
	newEdge.endAddress = nearVertex;
	double cost = configDiff((newVertex)->config, (nearVertex)->config);
	newEdge.cost = cost;
	gEdges.push_back(newEdge);

    graphEdge *edgePointer = &(gEdges.back());
	(newVertex)->numConnected = (newVertex)->numConnected + 1;
	(nearVertex)->numConnected = (nearVertex)->numConnected + 1;
	(newVertex)->connections.push_back(edgePointer);
	(nearVertex)->connections.push_back(edgePointer);
}

void graph::createNeighborhoodGraph(graphVertex **newVertex, std::list<graphVertex*> *neighborHood)
{
    int count=0;
    for (std::list<graphVertex>::iterator it= gVertices.begin(); it != gVertices.end(); ++it)
    {
        if(configDiff((*it).config, (*newVertex)->config) < RADIUS_PRM && !areConfigsSame((*it).config,(*newVertex)->config))
        {
            neighborHood->push_back(&(*it));     
        }
    }
    return;
}

