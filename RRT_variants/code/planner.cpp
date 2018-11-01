/*=================================================================
 *
 * planner.c
 *[path,len]=planner(load('map2.txt'),[pi/2 3*pi/4 pi/2 pi/4 pi/2],[pi/8 3*pi/4 pi 0.9*pi 1.5*pi],3);
 *[path,len]=planner(load('map1.txt'),[pi/2 pi/4 pi/2 pi/4 pi/2],[pi/8 3*pi/4 pi 0.9*pi 1.5*pi],3);
 *=================================================================*/
#include <math.h>
#include <vector>
#include "mex.h"
#include "rrt.hpp"
#include "kdtree.hpp"
#include "helper.hpp"
#include "collisionDetection.hpp"
#include "naryTree.hpp"
#include <queue>
#include <list>
#include "graph.hpp"
/* Input Arguments */
#define MAP_IN prhs[0]
#define ARMSTART_IN prhs[1]
#define ARMGOAL_IN prhs[2]
#define PLANNER_ID_IN prhs[3]

/* Planner Ids */
#define RRT 0
#define RRTCONNECT 1
#define RRTSTAR 2
#define PRM 3

/* Output Arguments */
#define PLAN_OUT plhs[0]
#define PLANLENGTH_OUT plhs[1]

using namespace std;

static void plannerRRT(
	double *map,
	int x_size,
	int y_size,
	double *armstart_anglesV_rad,
	double *armgoal_anglesV_rad,
	int numofDOFs,
	double ***plan,
	int *planlength)
{

	*plan = NULL;
	*planlength = 0;
	int goal_found=0;
	//collision detection on start and goal state
	if (!IsValidArmConfiguration(armstart_anglesV_rad, numofDOFs, map, x_size, y_size)){
		cout<<"Start configuration is in collision"<<endl;
		return;
	}
	if (!IsValidArmConfiguration(armgoal_anglesV_rad, numofDOFs, map, x_size, y_size)){
		cout<<"Goal configuration is in collision"<<endl;
		return;
	}
	//wrap to [-pi,pi] for start and goal state
	for( int i=0; i<DOF;i++){
		wrapToPI(armstart_anglesV_rad[i]);
		wrapToPI(armgoal_anglesV_rad[i]);
	}
	bool start_goal_same=areConfigsSame(armstart_anglesV_rad,armgoal_anglesV_rad);
	if(start_goal_same){
		cout<<"Start and Goal configuration are almost same"<<endl;
		return;
	}
	double min_dist=10;
	int max_iter=10000;
	int status_extend=0;

	NodeKDT *kd_root=NULL;
	NodeKDT *parent_root=NULL;
	NodeNT *nt_root=createNodeNT(NULL,armstart_anglesV_rad);
    kd_root=newNode(armstart_anglesV_rad,parent_root);
	
	double* config_new=new double[DOF];
	double* config_rand=new double[DOF];
	double* config_nearest=new double[DOF];

	//while configs are not same, we keep exploring
	for(int iter=0; iter<max_iter;++iter){ 

		int num_nodes_NT=0;

		config_rand= chooseTarget(armgoal_anglesV_rad);
		config_nearest= nearestNeighbor(kd_root, config_rand);
		config_new= extend(config_nearest, config_rand, map, x_size, y_size, &status_extend);

		if (!areConfigsSame(config_new,config_nearest) ){
			kd_root=insertKDTree(kd_root, parent_root, config_new, 0);
			insertNodeNT(nt_root,config_nearest, config_new);
			double dist_from_goal=configDiff(config_new,armgoal_anglesV_rad);
			countNumNodes(nt_root,num_nodes_NT);

			// if(iter%1000==0)
			// 	cout<<"Num nodes in NT = "<<num_nodes_NT<<" for iteration number= "<<iter<<endl;
			
			if (min_dist>dist_from_goal){
				// cout<<"distance from goal of new config= "<<dist_from_goal<<endl;
				min_dist=dist_from_goal;
			}
			
			if(dist_from_goal<=MIN_STEP){
				// std::cout<<"goal found"<<std::endl;
				goal_found=1;
				cout<<"number of iterations = "<<iter<<"\t";
				// for(int i=0;i<DOF;++i){
				// 	cout<<config_new[i]<<" , ";
				// }
				cout<<endl;
				break;
			}
    	}
	}

	deleteChildren(kd_root);
	if(goal_found){
		NodeNT* last_node=searchTree(nt_root, config_new);
		std::vector<double*> path;
		while (last_node!=NULL){
			path.push_back(last_node->config);
			last_node=last_node->parent;
		}
		*planlength=path.size();
		
		
		*plan = new double *[*planlength];
		for (int i = 0; i < *planlength; ++i)
		{
			(*plan)[i] = new double[DOF];
			for (int j = 0; j < DOF; ++j)
			{
				(*plan)[i][j] = path[path.size()-1-i][j];
			}
		}
	}


	
	delete[] config_new;
	delete[] config_rand;
	delete[] config_nearest;
	return;
}

static void plannerRRTConnect(
	double *map,
	int x_size,
	int y_size,
	double *armstart_anglesV_rad,
	double *armgoal_anglesV_rad,
	int numofDOFs,
	double ***plan,
	int *planlength)
{

// 	//no plan by default
	*plan = NULL;
	*planlength = 0;
	int goal_found=0;
	//collision detection on start and goal state
	if (!IsValidArmConfiguration(armstart_anglesV_rad, numofDOFs, map, x_size, y_size)){
		cout<<"Start configuration is in collision"<<endl;
		return;
	}
	if (!IsValidArmConfiguration(armgoal_anglesV_rad, numofDOFs, map, x_size, y_size)){
		cout<<"Goal configuration is in collision"<<endl;
		return;
	}
	bool start_goal_same=areConfigsSame(armstart_anglesV_rad,armgoal_anglesV_rad);
	if(start_goal_same){
		cout<<"Goal and start configurations are same";
		return;
	}
	//wrap to [-pi,pi] for start and goal state
	for( int i=0; i<DOF;i++){
		wrapToPI(armstart_anglesV_rad[i]);
		wrapToPI(armgoal_anglesV_rad[i]);
	}

	NodeKDT *start_KD_root=NULL;
	NodeKDT *goal_KD_root=NULL;
	NodeKDT *parent_root=NULL;
	start_KD_root=newNode(armstart_anglesV_rad,parent_root);
	goal_KD_root=newNode(armgoal_anglesV_rad,parent_root);

	NodeNT *start_nt_root=createNodeNT(NULL,armstart_anglesV_rad);
	NodeNT *goal_nt_root=createNodeNT(NULL,armgoal_anglesV_rad);
	int status_start=0;
	int status_connect=0;
	int max_iter=10000;
	double* config_new=new double[DOF];
	double* config_rand=new double[DOF];
	double* config_nearest=new double[DOF];
	double* config_connect=new double[DOF];
	for(int iter=0; iter<max_iter;++iter){ 

		status_start=0;
		status_connect=0;
		config_rand= chooseTarget(armgoal_anglesV_rad);
		config_nearest= nearestNeighbor(start_KD_root, config_rand);
		config_new= extend(config_nearest, config_rand, map, x_size, y_size, &status_start);

		if (!areConfigsSame(config_new,config_nearest)){
			start_KD_root=insertKDTree(start_KD_root, parent_root, config_new, 0);
			insertNodeNT(start_nt_root,config_nearest, config_new);
			config_connect=connect(goal_KD_root,config_new,map,x_size,y_size, &status_connect);
			config_nearest=nearestNeighbor(goal_KD_root, config_new);
			if (!areConfigsSame(config_nearest,config_connect)){
				goal_KD_root=insertKDTree(goal_KD_root, parent_root, config_connect, 0);
				insertNodeNT(goal_nt_root,config_nearest, config_connect);
			}
			if (status_connect==0){//reached
				cout<<"number of iterations = "<<iter<<endl;
				// cout<<"goal connect- reached and also "<<areConfigsSame(config_connect,config_new)<<endl;
				goal_found=1;
				break;
			}
		}

		status_start=0;
		status_connect=0;

		config_rand= chooseTarget(armstart_anglesV_rad);
		config_nearest= nearestNeighbor(goal_KD_root, config_rand);
		config_new= extend(config_nearest, config_rand, map, x_size, y_size, &status_start);

		if (!areConfigsSame(config_new,config_nearest)){
			goal_KD_root=insertKDTree(goal_KD_root, parent_root, config_new, 0);
			insertNodeNT(goal_nt_root,config_nearest, config_new);
			config_connect=connect(start_KD_root,config_new,map,x_size,y_size, &status_connect);
			config_nearest=nearestNeighbor(start_KD_root, config_new);
			if (!areConfigsSame(config_nearest,config_connect)){
				start_KD_root=insertKDTree(start_KD_root, parent_root, config_connect, 0);
				insertNodeNT(start_nt_root,config_nearest, config_connect);
			}
			if (status_connect==0){//reached- basically means config_connect is same as config_new
				cout<<"number of iterations = "<<iter<<endl;
				// cout<<"start connect - reached and also "<<areConfigsSame(config_connect,config_new)<<endl;
				goal_found=1;
				break;
			}
		}	
    }
	deleteChildren(start_KD_root);
	deleteChildren(goal_KD_root);
	if(goal_found){
		NodeNT* last_node_start=searchTree(start_nt_root, config_new);
		std::vector<double*> path_start;
		while (last_node_start!=NULL){
			path_start.push_back(last_node_start->config);
			last_node_start=last_node_start->parent;
		}

		NodeNT* last_node_goal=searchTree(goal_nt_root, config_new);
		std::vector<double*> path_goal;
		while (last_node_goal!=NULL){
			path_goal.push_back(last_node_goal->config);
			last_node_goal=last_node_goal->parent;
		}
	


		*planlength=path_start.size()+path_goal.size()-1;
		*plan = new double *[*planlength];
		for (int i = 0; i < path_start.size(); ++i)
		{
			(*plan)[i] = new double[DOF];
			for (int j = 0; j < DOF; ++j)
			{
				(*plan)[i][j] = path_start[path_start.size()-1-i][j];
			}
		}
		for (int i = path_start.size(); i < *planlength; ++i)
		{
			(*plan)[i] = new double[DOF];
			for (int j = 0; j < DOF; ++j)
			{
				(*plan)[i][j] = path_goal[i-path_start.size()+1][j];
			}
		}

	}

	delete[] config_new;
	delete[] config_rand;
	delete[] config_nearest;
	delete[] config_connect;
	return;
}

static void plannerRRTStar(
	double *map,
	int x_size,
	int y_size,
	double *armstart_anglesV_rad,
	double *armgoal_anglesV_rad,
	int numofDOFs,
	double ***plan,
	int *planlength)
{
	//no plan by default
	*plan = NULL;
	*planlength = 0;
	int goal_found=0;
	if (!IsValidArmConfiguration(armstart_anglesV_rad, numofDOFs, map, x_size, y_size)){
		cout<<"Start configuration is in collision"<<endl;
		return;
	}
	if (!IsValidArmConfiguration(armgoal_anglesV_rad, numofDOFs, map, x_size, y_size)){
		cout<<"Goal configuration is in collision"<<endl;
		return;
	}
	//wrap to [-pi,pi] for start and goal state
	for( int i=0; i<DOF;i++){
		wrapToPI(armstart_anglesV_rad[i]);
		wrapToPI(armgoal_anglesV_rad[i]);
	}
	bool start_goal_same=areConfigsSame(armstart_anglesV_rad,armgoal_anglesV_rad);
	if(start_goal_same){
		cout<<"Start and Goal configuration are almost same"<<endl;
		return;
	}
	double min_dist=10;
	int max_iter=10000;
	int status_extend=0;
	
	//radius should be less than or equal to max_step
	//lets keep it equal
	NodeKDT *kd_root=NULL;
	NodeKDT *parent_root=NULL;
	NodeNT *nt_root=createNodeNT(NULL,armstart_anglesV_rad);
    kd_root=newNode(armstart_anglesV_rad,parent_root);
	
	double* config_new=new double[DOF];
	double* config_rand=new double[DOF];
	double* config_nearest=new double[DOF];

	//while configs are not same, we keep exploring
	for(int iter=0; iter<max_iter;++iter){ 

		int num_nodes_NT=0;

		config_rand= chooseTarget(armgoal_anglesV_rad);
		config_nearest= nearestNeighbor(kd_root, config_rand);
		config_new= extend(config_nearest, config_rand, map, x_size, y_size, &status_extend);
		//main RRT star code starts here
		
		double* config_min=config_nearest;
		ComparePQ cmp=ComparePQ(config_new);
		std::priority_queue<NodeKDT*, std::vector<NodeKDT*>,decltype(cmp)> kNN_pq(cmp);
		std::vector<NodeKDT*> node_near_vec= kNearestNeighbor(kd_root, config_new, kNN_pq);
		//chose x_near (x_min) with least cost path starting from from start to x_near to x_new
		for(int near_iter=0;near_iter<node_near_vec.size();++near_iter){
			status_extend=0;
			double* config_near=node_near_vec[near_iter]->config;
			IsValidMovement(config_near, config_new, numofDOFs, map,x_size, y_size, &status_extend);
			if (status_extend==0){
				double cost_x_near=configDiff(config_near, armstart_anglesV_rad);
				double cost_line=configDiff(config_near,config_new);
				double cost_x_new=configDiff(config_new,armstart_anglesV_rad);
				double new_cost=cost_x_near+cost_line;
				if (new_cost<cost_x_new){
					config_min=config_near;
				}
			}
		}
		if (!areConfigsSame(config_new,config_min) ){
			kd_root=insertKDTree(kd_root,parent_root, config_new,0); //do it always
			insertNodeNT(nt_root,config_min, config_new);
		}
		
		//for all other near configs check if the curr cost to them is greater than cost via x_new
		//if yes, remove the existing connection and make x_new their parent
		for(int near_iter=0;near_iter<node_near_vec.size();++near_iter ){
			double* config_near=node_near_vec[near_iter]->config;
			if (!areConfigsSame(config_min,config_near)){
				status_extend=0;
				IsValidMovement(config_near, config_new, numofDOFs, map,x_size, y_size, &status_extend);
				if (status_extend==0){
					double cost_x_near=configDiff(config_near, armstart_anglesV_rad);
					double cost_line=configDiff(config_near,config_new);
					double cost_x_new=configDiff(config_new,armstart_anglesV_rad);
					if(cost_x_near>cost_x_new+cost_line){
						NodeNT* node_new_NT=searchTree(nt_root, config_new);
						//remove this edge
						// cout<<"improving the graph"<<endl;
						replaceParent(nt_root, config_near, node_new_NT);
					}
				}
			}
		}
		//endfor all x_near
		if (!areConfigsSame(config_new,config_min) ){
			double dist_from_goal=configDiff(config_new,armgoal_anglesV_rad);
			countNumNodes(nt_root,num_nodes_NT);

			// if(iter%1000==0)
				// cout<<"Num nodes in NT = "<<num_nodes_NT<<" for iteration number= "<<iter<<endl;
			
			if (min_dist>dist_from_goal){
				// cout<<"distance from goal of new config= "<<dist_from_goal<<endl;
				min_dist=dist_from_goal;
			}
			
			if(dist_from_goal<=MIN_STEP){
				// std::cout<<"goal found"<<std::endl;
				goal_found=1;
				cout<<"number of iterations = "<<iter<<endl;
				// for(int i=0;i<DOF;++i){
				// 	cout<<config_new[i]<<" , ";
				// }
				cout<<endl;
				break;
			}
		}
	}
	//end_iters
	deleteChildren(kd_root);
	if(goal_found){
		NodeNT* last_node=searchTree(nt_root, config_new);
		std::vector<double*> path;
		while (last_node!=NULL){
			path.push_back(last_node->config);
			last_node=last_node->parent;
		}
		*planlength=path.size();
		
		
		*plan = new double *[*planlength];
		for (int i = 0; i < *planlength; ++i)
		{
			(*plan)[i] = new double[DOF];
			for (int j = 0; j < DOF; ++j)
			{
				(*plan)[i][j] = path[path.size()-1-i][j];
			}
		}

	}
	
	delete[] config_new;
	delete[] config_rand;
	delete[] config_nearest;
	return;
}


static void plannerPRM(
	double *map,
	int x_size,
	int y_size,
	double *armstart_anglesV_rad,
	double *armgoal_anglesV_rad,
	int numofDOFs,
	double ***plan,
	int *planlength)
{

	//no plan by default
	*plan = NULL;
	*planlength = 0;
	//collision detection on start and goal state
	if (!IsValidArmConfiguration(armstart_anglesV_rad, numofDOFs, map, x_size, y_size)){
		cout<<"Start configuration is in collision"<<endl;
		return;
	}
	if (!IsValidArmConfiguration(armgoal_anglesV_rad, numofDOFs, map, x_size, y_size)){
		cout<<"Goal configuration is in collision"<<endl;
		return;
	}
	//wrap to [-pi,pi] for start and goal state
	for( int i=0; i<DOF;i++){
		wrapToPI(armstart_anglesV_rad[i]);
		wrapToPI(armgoal_anglesV_rad[i]);
	}
	bool start_goal_same=areConfigsSame(armstart_anglesV_rad,armgoal_anglesV_rad);
	if(start_goal_same){
		cout<<"Start and Goal configuration are almost same"<<endl;
		return;
	}
	double min_dist_goal=10000,min_dist_start=10000;
	int max_iter=10000;
	int IDNUMBER = 1;
	int extend_status=0;
	double dist_from_goal,dist_from_start;
	graph* roadmap=new graph;
	graphVertex *newVertex;
	std::list<graphVertex*> neighborHood;
	int targetDensity = 20;

	double* config_rand=new double[DOF];
	double* nearest_goal=new double[DOF];
	double* nearest_start=new double[DOF];
	for(int iter=0; iter<max_iter;++iter){ 
		//choose a config that's collision free and unique
		neighborHood.clear();
		config_rand=randConfigFree(map,x_size,y_size);
		roadmap->addVertexGraph(config_rand, IDNUMBER);
		newVertex = &((roadmap->gVertices).back());
		IDNUMBER++;
		roadmap->createNeighborhoodGraph(&newVertex, &neighborHood);
		// if(iter%1000==0){
		// 	cout<<"number of iterations passed = "<<iter<<" and nodes added = "<<IDNUMBER<<endl;
		// }
		for (std::list<graphVertex*>::iterator it= neighborHood.begin(); it != neighborHood.end(); ++it){
			if((*it)->numConnected < targetDensity )
			{
				extend_status=0;
				IsValidMovement((*it)->config,  newVertex->config,DOF,map,  x_size,  y_size, &extend_status);
				if(extend_status!=2)
				{
					dist_from_goal=configDiff(newVertex->config,armgoal_anglesV_rad);
					dist_from_start=configDiff(newVertex->config,armstart_anglesV_rad);
					if(min_dist_goal>dist_from_goal){
						min_dist_goal=dist_from_goal;
						// cout<<"min_dist from goal = "<<min_dist_goal<<endl;
						for(int i=0;i<DOF;i++)
							nearest_goal[i]=newVertex->config[i];
					}
					if(min_dist_start>dist_from_start){
						min_dist_start=dist_from_start;
						// cout<<"min_dist from start = "<<min_dist_start<<endl;
						for(int i=0;i<DOF;i++)
							nearest_start[i]=newVertex->config[i];
					}
					roadmap->addEdgeGraph(newVertex, (*it));
				}
      		}
    	}
  	}
	// cout<<"Running Dijkstra..."<<endl;
	dijkstra(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad, plan, planlength, roadmap);
	return;
}


//prhs contains input parameters (3):
//1st is matrix with all the obstacles
//2nd is a row vector of start angles for the arm
//3nd is a row vector of goal angles for the arm
//plhs should contain output parameters (2):
//1st is a 2D matrix plan when each plan[i][j] is the value of jth angle at the ith step of the plan
//(there are D DoF of the arm (that is, D angles). So, j can take values from 0 to D-1
//2nd is planlength (int)
void mexFunction(int nlhs, mxArray *plhs[],
				 int nrhs, const mxArray *prhs[])

{

	/* Check for proper number of arguments */
	if (nrhs != 4)
	{
		mexErrMsgIdAndTxt("MATLAB:planner:invalidNumInputs",
						  "Four input arguments required.");
	}
	else if (nlhs != 2)
	{
		mexErrMsgIdAndTxt("MATLAB:planner:maxlhs",
						  "One output argument required.");
	}

	/* get the dimensions of the map and the map matrix itself*/
	int x_size = (int)mxGetM(MAP_IN);
	int y_size = (int)mxGetN(MAP_IN);
	double *map = mxGetPr(MAP_IN);

	/* get the start and goal angles*/
	int numofDOFs = (int)(MAX(mxGetM(ARMSTART_IN), mxGetN(ARMSTART_IN)));
	if (numofDOFs <= 1)
	{
		mexErrMsgIdAndTxt("MATLAB:planner:invalidnumofdofs",
						  "it should be at least 2");
	}
	double *armstart_anglesV_rad = mxGetPr(ARMSTART_IN);
	if (numofDOFs != MAX(mxGetM(ARMGOAL_IN), mxGetN(ARMGOAL_IN)))
	{
		mexErrMsgIdAndTxt("MATLAB:planner:invalidnumofdofs",
						  "numofDOFs in startangles is different from goalangles");
	}
	double *armgoal_anglesV_rad = mxGetPr(ARMGOAL_IN);

	//get the planner id
	int planner_id = (int)*mxGetPr(PLANNER_ID_IN);
	if (planner_id < 0 || planner_id > 3)
	{
		mexErrMsgIdAndTxt("MATLAB:planner:invalidplanner_id",
						  "planner id should be between 0 and 3 inclusive");
	}

	//call the planner
	double **plan = NULL;
	int planlength = 0;

	//you can may be call the corresponding planner function here
	switch(planner_id)
	{
		case RRT:
			plannerRRT(map,x_size,y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, &plan, &planlength);
			break;
		case RRTCONNECT:
			plannerRRTConnect(map,x_size,y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, &plan, &planlength);
			break;
		case RRTSTAR:
			plannerRRTStar(map,x_size,y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, &plan, &planlength);
			break;
		case PRM:
			plannerPRM(map,x_size,y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, &plan, &planlength);
			break;
	}

	//dummy planner which only computes interpolated path
	// planner(map, x_size, y_size, armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, &plan, &planlength);

	printf("planner returned plan of length=%d\n", planlength);

	/* Create return values */
	if (planlength > 0)
	{
		PLAN_OUT = mxCreateNumericMatrix((mwSize)planlength, (mwSize)numofDOFs, mxDOUBLE_CLASS, mxREAL);
		double *plan_out = mxGetPr(PLAN_OUT);
		//copy the values
		int i, j;
		for (i = 0; i < planlength; i++)
		{
			for (j = 0; j < numofDOFs; j++)
			{
				plan_out[j * planlength + i] = plan[i][j];
			}
		}
	}
	else
	{
		PLAN_OUT = mxCreateNumericMatrix((mwSize)1, (mwSize)numofDOFs, mxDOUBLE_CLASS, mxREAL);
		double *plan_out = mxGetPr(PLAN_OUT);
		//copy the values
		int j;
		for (j = 0; j < numofDOFs; j++)
		{
			plan_out[j] = armstart_anglesV_rad[j];
		}
	}
	PLANLENGTH_OUT = mxCreateNumericMatrix((mwSize)1, (mwSize)1, mxINT8_CLASS, mxREAL);
	int *planlength_out = (int *)mxGetPr(PLANLENGTH_OUT);
	*planlength_out = planlength;

	return;
}
