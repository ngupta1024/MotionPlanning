#include <iostream>
#include <math.h>
#include "collisionDetection.hpp"
#include "kdtree.hpp"
#include "rrt.hpp"
#include "helper.hpp"
using namespace std;

void ContXY2Cell(double x, double y, short unsigned int *pX, short unsigned int *pY, int x_size, int y_size)
{
	double cellsize = 1.0;
	//take the nearest cell
	*pX = (int)(x / (double)(cellsize));
	if (x < 0)
		*pX = 0;
	if (*pX >= x_size)
		*pX = x_size - 1;

	*pY = (int)(y / (double)(cellsize));
	if (y < 0)
		*pY = 0;
	if (*pY >= y_size)
		*pY = y_size - 1;
}

void get_bresenham_parameters(int p1x, int p1y, int p2x, int p2y, bresenham_param_t *params)
{
	params->UsingYIndex = 0;

	if (fabs((double)(p2y - p1y) / (double)(p2x - p1x)) > 1)
		(params->UsingYIndex)++;

	if (params->UsingYIndex)
	{
		params->Y1 = p1x;
		params->X1 = p1y;
		params->Y2 = p2x;
		params->X2 = p2y;
	}
	else
	{
		params->X1 = p1x;
		params->Y1 = p1y;
		params->X2 = p2x;
		params->Y2 = p2y;
	}

	if ((p2x - p1x) * (p2y - p1y) < 0)
	{
		params->Flipped = 1;
		params->Y1 = -params->Y1;
		params->Y2 = -params->Y2;
	}
	else
		params->Flipped = 0;

	if (params->X2 > params->X1)
		params->Increment = 1;
	else
		params->Increment = -1;

	params->DeltaX = params->X2 - params->X1;
	params->DeltaY = params->Y2 - params->Y1;

	params->IncrE = 2 * params->DeltaY * params->Increment;
	params->IncrNE = 2 * (params->DeltaY - params->DeltaX) * params->Increment;
	params->DTerm = (2 * params->DeltaY - params->DeltaX) * params->Increment;

	params->XIndex = params->X1;
	params->YIndex = params->Y1;
}

void get_current_point(bresenham_param_t *params, int *x, int *y)
{
	if (params->UsingYIndex)
	{
		*y = params->XIndex;
		*x = params->YIndex;
		if (params->Flipped)
			*x = -*x;
	}
	else
	{
		*x = params->XIndex;
		*y = params->YIndex;
		if (params->Flipped)
			*y = -*y;
	}
}

int get_next_point(bresenham_param_t *params)
{
	if (params->XIndex == params->X2)
	{
		return 0;
	}
	params->XIndex += params->Increment;
	if (params->DTerm < 0 || (params->Increment < 0 && params->DTerm <= 0))
		params->DTerm += params->IncrE;
	else
	{
		params->DTerm += params->IncrNE;
		params->YIndex += params->Increment;
	}
	return 1;
}

int IsValidLineSegment(double x0, double y0, double x1, double y1, double *map, int x_size, int y_size)
{
	bresenham_param_t params;
	int nX, nY;
	short unsigned int nX0, nY0, nX1, nY1;

	// printf("checking link <%f %f> to <%f %f>\n", x0,y0,x1,y1);

	//make sure the line segment is inside the environment
	if (x0 < 0 || x0 >= x_size ||
		x1 < 0 || x1 >= x_size ||
		y0 < 0 || y0 >= y_size ||
		y1 < 0 || y1 >= y_size){
			return 0;
		}
		

	ContXY2Cell(x0, y0, &nX0, &nY0, x_size, y_size);
	ContXY2Cell(x1, y1, &nX1, &nY1, x_size, y_size);

	// printf("checking link <%d %d> to <%d %d>\n", nX0,nY0,nX1,nY1);

	//iterate through the points on the segment
	get_bresenham_parameters(nX0, nY0, nX1, nY1, &params);
	do
	{
		
		get_current_point(&params, &nX, &nY);
		
		if (map[GETMAPINDEX(nX, nY, x_size, y_size)] == 1){
			// cout<<nX<<" , "<<nY<<endl;
			return 0;
		}
	} while (get_next_point(&params));

	return 1;
}

int IsValidArmConfiguration(double* angles, int numofDOFs, double*	map, int x_size, int y_size)
{
    double x0,y0,x1,y1;
    int i;
    
 	//iterate through all the links starting with the base
	x1 = ((double)x_size)/2.0;
    y1 = 0;
	for(i = 0; i < numofDOFs; i++)
	{
		//compute the corresponding line segment
		x0 = x1;
		y0 = y1;
		x1 = x0 + LINKLENGTH_CELLS*cos(2*PI-angles[i]);
		y1 = y0 - LINKLENGTH_CELLS*sin(2*PI-angles[i]);

		//check the validity of the corresponding line segment
		if(!IsValidLineSegment(x0,y0,x1,y1,map,x_size,y_size))
				return 0;
	}    
    return 1;
}


double* IsValidMovement(double *angles_start, double *angles_goal, int numofDOFs, double *map,int x_size, int y_size, int* status)
{
	double x0, y0, x1, y1;
	double* angles_delta;
	int count_iter=0;
	while(configDiff(angles_start,angles_goal)>MIN_STEP){
	// for(int loop_iter=0; loop_iter<500; ++loop_iter){
		count_iter++;
		//iterate through all the links starting with the base
		bool noCollision=true;
		x1 = ((double)x_size) / 2.0;
		y1 = 0;
		for (int i = 0; i < numofDOFs; i++)
		{
			//compute the corresponding line segment
			x0 = x1;
			y0 = y1;
			x1 = x0 + LINKLENGTH_CELLS * cos(2 * PI - angles_goal[i]);
			y1 = y0 - LINKLENGTH_CELLS * sin(2 * PI - angles_goal[i]);
			
			//check the validity of the corresponding line segment
			if (!IsValidLineSegment(x0, y0, x1, y1, map, x_size, y_size)){
				noCollision= noCollision & false;
				// cout<<"collision is happening"<<endl;
				*status=2;	
			}
		}
		if (noCollision){
			return angles_goal;
		}
		else{
			double dist=configDiff(angles_goal, angles_start);
			double alpha=0.8;
			if (dist<MIN_STEP){
				return angles_start;
			}
			for (int i = 0; i < numofDOFs; i++){
				// seg_length=0.7*dist;
				angles_goal[i]=alpha*angles_goal[i]+(1-alpha)*angles_start[i];
				wrapToPI(angles_goal[i]);
			}
		}
	}
	*status=2;
	// cout<<count_iter<<endl;
	return angles_start;
}