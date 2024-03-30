/*=================================================================
 *
 * planner.cpp
 *
 *=================================================================*/
#include "planner.h"
#include <math.h>
#include <vector>
#include <iostream>
#include <limits>
#include <algorithm>
#include <queue>
#include <map>


#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define NUMOFDIRS 8

using namespace std; 


bool customComparator(const vector<int>& a, const vector<int>& b) {
    return a[2] > b[2]; // Compare the f_value 
}


vector<vector<int>> path;
bool path_computed = false; 
int current_step = 0;
int backward_count = 0;
bool reached_goal_once = false;

/* A-Star Planner */

void planner(
    int* map,
    int collision_thresh,
    int x_size,
    int y_size,
    int robotposeX,
    int robotposeY,
    int target_steps,
    int* target_traj,
    int targetposeX,
    int targetposeY,
    int curr_time,
    int* action_ptr
    )
{
    // 8-connected grid
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};
    
    // cout << "Robot: " << robotposeX << " " << robotposeY << endl;
    // cout << "Target: " << targetposeX << " " << targetposeY << endl;

    // first goal set to a few steps before last traj point
    int goalposeX = target_traj[target_steps-1 - backward_count];             
    int goalposeY = target_traj[target_steps-1+target_steps - backward_count];
    

    if (robotposeX == goalposeX && robotposeY == goalposeY){
        reached_goal_once = true;           // boolean triggers when the first goal has been reached 
    }

    // If already reached the first goal pose,
    if (reached_goal_once){
            
            // if target is NOT one step away, move backwards along the target's trajectory
            if (!(target_traj[curr_time + 1] == robotposeX || target_traj[curr_time + 1 + target_steps] == robotposeY)){   
                backward_count += 1;
                robotposeX = target_traj[target_steps - 1 - backward_count];
                robotposeY = target_traj[target_steps-1+target_steps - backward_count];
            }

            action_ptr[0] = robotposeX;
            action_ptr[1] = robotposeY;

            return;
        }
    
    // Planner only computes the path once (before the first step), and follows the trajectory for all other steps
    if (path_computed){
        action_ptr[0] = path[path.size() - 2 - current_step][0];
        action_ptr[1] = path[path.size() - 2 - current_step][1];
        
        current_step += 1;
        cout << "Command: " << action_ptr[0] << " " << action_ptr[1] << endl;

        return;
    }


    // initializing datastructures
    vector<vector<int>> h_values(y_size, vector<int>(x_size, 1)); 
    vector<vector<int>> g_values(y_size, vector<int>(x_size, numeric_limits<int>::max())); 
    vector<vector<int>> status(y_size, vector<int>(x_size, -1));                // status = -1 if not explored, 0 if open, 1 if closed
    priority_queue<vector<int>, vector<vector<int>>, decltype(&customComparator)> open(&customComparator); //sorting based on decreasing value in 3rd column (f value)
    vector<int> combine; 
    vector<int> robot_pose; 
    vector<int> parent_pose;
    std::map<std::vector<int>, std::vector<int>> closed; // {robotx, roboty} : {parentx, parenty}




    // initializing variables
    int f_value = 0;
    int first_directionX;
    int first_directionY; 
    int count = 0;              // to keep track of number of iterations of the planner 
    int parentx = robotposeX; 
    int parenty = robotposeY; 
    int h_weight = 1; 


    g_values[robotposeY - 1][robotposeX - 1] = 0;
    h_values[robotposeY - 1][robotposeX - 1] = max(abs(robotposeX - goalposeX), abs(robotposeY - goalposeY)); 
    f_value = g_values[robotposeY - 1][robotposeX - 1] + h_weight * h_values[robotposeY - 1][robotposeX - 1];
    combine = {robotposeY, robotposeX, f_value, parentx, parenty};
    open.push(combine);
    status[robotposeY - 1][robotposeX - 1] = 0;

    
    // Implementing A* search to calculate g-values for all explored cells leading up to the goal cell

    while (open.size() > 0 && status[goalposeY - 1][goalposeX - 1] != 1){

        int best_robotX = open.top()[1];       // s with smallest f in open is the last element
        int best_robotY = open.top()[0];

        int parentx = open.top()[3];
        int parenty = open.top()[4];

        count += 1;
        open.pop();                        // removing s with smallest f from open
        robot_pose = {best_robotX, best_robotY};
        parent_pose = {parentx, parenty};
        closed[robot_pose] = parent_pose;       // storing parent of every explored cell in closed list

        for(int dir = 0; dir < NUMOFDIRS; dir++)
            {  

            int newx = best_robotX + dX[dir];            // moving one step in every direction in x
            int newy = best_robotY + dY[dir];            // moving one step in every direction in y
            
    
            if (newx >= 1 && newx <= x_size && newy >= 1 && newy <= y_size)             // if new position is within map size
                {
                //if new position is valid and free
                if ((map[GETMAPINDEX(newx,newy,x_size,y_size)] >= 0) && (map[GETMAPINDEX(newx,newy,x_size,y_size)] < collision_thresh) 
                        && status[newy - 1][newx - 1] != 1)  //if free
                    {
                        int cost_next_step = map[GETMAPINDEX(newx,newy,x_size,y_size)] + (double)sqrt(((newx-goalposeX)*(newx-goalposeX) + (newy-goalposeY)*(newy-goalposeY)));
                        if (g_values[newy - 1][newx - 1] > g_values[best_robotY - 1][best_robotX - 1] + cost_next_step){
                            g_values[newy - 1][newx - 1] = g_values[best_robotY - 1][best_robotX - 1] + cost_next_step;

                            h_values[robotposeY - 1][robotposeX - 1] = max(abs(newx - goalposeX), abs(newy - goalposeY)); 
                            f_value = g_values[newy - 1][newx - 1] + h_weight * h_values[newy - 1][newx - 1];
                            
                            combine = {newy, newx, f_value, best_robotX, best_robotY};
                            if (status[newy - 1][newx - 1] == -1){
                                open.push(combine);
                                status[newy - 1][newx - 1] = 0;
                            }
                        }
                    }
                }
            status[best_robotY - 1][best_robotX - 1] = 1;     // marking current position as closed
            
            
            }

        }

    // Backtracking from goal to start

    int currentX = goalposeX;
    int currentY = goalposeY;

    while (currentX != robotposeX || currentY != robotposeY) {
        path.push_back({currentX, currentY});

        vector<int> current_pose = {currentX, currentY};
        parentx = closed[current_pose][0];
        parenty = closed[current_pose][1];

        currentX = parentx;
        currentY = parenty;
    }

    path_computed = true;

    // Add the starting position to the path
    path.push_back({robotposeX, robotposeY});

    action_ptr[0] = path[path.size() - 2 - current_step][0];
    action_ptr[1] = path[path.size() - 2 - current_step][1];
    
    current_step += 1;
    return;
}
