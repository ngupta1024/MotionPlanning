/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <math.h>
#include "mex.h"
#include "aStar.hpp"
#include <limits>

/* Input Arguments */
#define	MAP_IN                  prhs[0]
#define	ROBOT_IN                prhs[1]
#define	TARGET_TRAJ             prhs[2]
#define	TARGET_POS              prhs[3]
#define	CURR_TIME               prhs[4]
#define	COLLISION_THRESH        prhs[5]


/* Output Arguments */
#define	ACTION_OUT              plhs[0]

//access to the map is shifted to account for 0-based indexing in the map, whereas
//1-based indexing in matlab (so, robotpose and goalpose are 1-indexed)
#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8
int count=0;
static void planner(
        double*	map,
        int collision_thresh,
        int x_size,
        int y_size,
        int robotposeX,
        int robotposeY,
        int target_steps,
        double* target_traj,
        int targetposeX,
        int targetposeY,
        int curr_time,
        double* action_ptr
        )
{
    // 8-connected grid
    int goalposeX,goalposeY;
    static int path_gen=0;
    static int prev_time_stamp=0;
    static Generator::coordinateList path;
    static int steps=0;
    goalposeX= (int) target_traj[curr_time];
    goalposeY= (int) target_traj[curr_time+target_steps];
    int look_ahead=0;
    // printf("curr time: %d \n",curr_time);
    // printf("target steps: %d \n",target_steps);
    // printf("world: %d %d;\n", x_size, y_size);
    // printf("robot: %d %d;\n", robotposeX, robotposeY);
    // printf("goal: %d %d;\n", goalposeX, goalposeY);
    // printf("target: %d %d; \n",targetposeX, targetposeY);
    Generator generator(x_size, y_size, collision_thresh);
    
    //only first time
    if (curr_time==prev_time_stamp){
        path = generator.findPath( map,robotposeX, robotposeY, goalposeX, goalposeY);
        path_gen=1;
        steps=0;
    }
    else{
        if ((curr_time-prev_time_stamp)>path.size()/2 || path.size()<=2+steps)//look ahead and replanning dynamically changes
        {
            look_ahead=steps;
            if(curr_time+look_ahead<target_steps)
            {
                goalposeX= (int) target_traj[curr_time+look_ahead];
                goalposeY= (int) target_traj[curr_time+look_ahead+target_steps];
            }
            else
            {
                goalposeX = (int) target_traj[target_steps-1];
                goalposeY = (int) target_traj[target_steps-1+target_steps];
            }
            path = generator.findPath( map,robotposeX, robotposeY, goalposeX, goalposeY);
            prev_time_stamp=curr_time;
            steps=0;
        }
        else
        {
            steps++;
        }
    }
    
    
    Generator::Vec2i next_state;
    if(path.size()>=(2+steps))
    {
        next_state=path[path.size() - (2+steps)];
    }
    else if(path.size()==1)
    {
        next_state=path[path.size() - 1];
    }
    action_ptr[0] = next_state.x;
    action_ptr[1] = next_state.y;
    // printf("new robot pose: %d %d; \n", next_state.x,next_state.y);
    return;
}

// prhs contains input parameters (4):
// 1st is matrix with all the obstacles
// 2nd is a row vector <x,y> for the robot position
// 3rd is a matrix with the target trajectory
// 4th is an integer C, the collision threshold for the map
// plhs should contain output parameters (1):
// 1st is a row vector <dx,dy> which corresponds to the action that the robot should make
void mexFunction( int nlhs, mxArray *plhs[],
        int nrhs, const mxArray*prhs[] )
        
{
    
    /* Check for proper number of arguments */
    if (nrhs != 6) {
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
                "Six input arguments required.");
    } else if (nlhs != 1) {
        mexErrMsgIdAndTxt( "MATLAB:planner:maxlhs",
                "One output argument required.");
    }
    
    /* get the dimensions of the map and the map matrix itself*/
    int x_size = mxGetM(MAP_IN);
    int y_size = mxGetN(MAP_IN);
    double* map = mxGetPr(MAP_IN);
    
    /* get the dimensions of the robotpose and the robotpose itself*/
    int robotpose_M = mxGetM(ROBOT_IN);
    int robotpose_N = mxGetN(ROBOT_IN);
    if(robotpose_M != 1 || robotpose_N != 2){
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidrobotpose",
                "robotpose vector should be 1 by 2.");
    }
    double* robotposeV = mxGetPr(ROBOT_IN);
    int robotposeX = (int)robotposeV[0];
    int robotposeY = (int)robotposeV[1];
    
    /* get the dimensions of the goalpose and the goalpose itself*/
    int targettraj_M = mxGetM(TARGET_TRAJ);
    int targettraj_N = mxGetN(TARGET_TRAJ);
    
    if(targettraj_M < 1 || targettraj_N != 2)
    {
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidtargettraj",
                "targettraj vector should be M by 2.");
    }
    double* targettrajV = mxGetPr(TARGET_TRAJ);
    int target_steps = targettraj_M;
    
    /* get the current position of the target*/
    int targetpose_M = mxGetM(TARGET_POS);
    int targetpose_N = mxGetN(TARGET_POS);
    if(targetpose_M != 1 || targetpose_N != 2){
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidtargetpose",
                "targetpose vector should be 1 by 2.");
    }
    double* targetposeV = mxGetPr(TARGET_POS);
    int targetposeX = (int)targetposeV[0];
    int targetposeY = (int)targetposeV[1];
    
    /* get the current timestep the target is at*/
    int curr_time = mxGetScalar(CURR_TIME);
    
    /* Create a matrix for the return action */ 
    ACTION_OUT = mxCreateNumericMatrix( (mwSize)1, (mwSize)2, mxDOUBLE_CLASS, mxREAL); 
    double* action_ptr = (double*) mxGetData(ACTION_OUT);
    
    /* Get collision threshold for problem */
    int collision_thresh = (int) mxGetScalar(COLLISION_THRESH);
    
    /* Do the actual planning in a subroutine */
    planner(map, collision_thresh, x_size, y_size, robotposeX, robotposeY, target_steps, targettrajV, targetposeX, targetposeY, curr_time, &action_ptr[0]);
    // printf("DONE PLANNING!\n");
    return;   
}