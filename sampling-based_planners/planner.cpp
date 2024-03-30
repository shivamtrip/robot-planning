/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <math.h>
#include <random>
#include <vector>
#include <array>
#include <algorithm>
#include <map>
#include <queue>
#include <chrono>

#include <tuple>
#include <string>
#include <stdexcept>
#include <regex> // For regex and split logic
#include <iostream> // cout, endl
#include <fstream> // For reading/writing files
#include <assert.h> 

/* Input Arguments */
#define	MAP_IN      prhs[0]
#define	ARMSTART_IN	prhs[1]
#define	ARMGOAL_IN     prhs[2]
#define	PLANNER_ID_IN     prhs[3]

/* Planner Ids */
#define RRT         0
#define RRTCONNECT  1
#define RRTSTAR     2
#define PRM         3

/* Output Arguments */
#define	PLAN_OUT	plhs[0]
#define	PLANLENGTH_OUT	plhs[1]

#define NUMOFDIRS 8

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) (Y*XSIZE + X)

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define PI 3.141592654

//the length of each link in the arm
#define LINKLENGTH_CELLS 10

// Some potentially helpful imports
using std::vector;
using std::array;
using std::string;
using std::runtime_error;
using std::tuple;
using std::make_tuple;
using std::tie;
using std::cout;
using std::endl;

//*******************************************************************************************************************//
//                                                                                                                   //
//                                                GIVEN FUNCTIONS                                                    //
//                                                                                                                   //
//*******************************************************************************************************************//

/// @brief 
/// @param filepath 
/// @return map, x_size, y_size
tuple<double*, int, int> loadMap(string filepath) {
	std::FILE *f = fopen(filepath.c_str(), "r");
	if (f) {
	}
	else {
		printf("Opening file failed! \n");
		throw runtime_error("Opening map file failed!");
	}
	int height, width;
	if (fscanf(f, "height %d\nwidth %d\n", &height, &width) != 2) {
		throw runtime_error("Invalid loadMap parsing map metadata");
	}
	
	////// Go through file and add to m_occupancy
	double* map = new double[height*width];

	double cx, cy, cz;
	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			char c;
			do {
				if (fscanf(f, "%c", &c) != 1) {
					throw runtime_error("Invalid parsing individual map data");
				}
			} while (isspace(c));
			if (!(c == '0')) { 
				map[y+x*width] = 1; // Note transposed from visual
			} else {
				map[y+x*width] = 0;
			}
		}
	}
	fclose(f);
	return make_tuple(map, width, height);
}

// Splits string based on deliminator
vector<string> split(const string& str, const string& delim) {   
		// https://stackoverflow.com/questions/14265581/parse-split-a-string-in-c-using-string-delimiter-standard-c/64886763#64886763
		const std::regex ws_re(delim);
		return { std::sregex_token_iterator(str.begin(), str.end(), ws_re, -1), std::sregex_token_iterator() };
}


double* doubleArrayFromString(string str) {
	vector<string> vals = split(str, ",");
	double* ans = new double[vals.size()];
	for (int i = 0; i < vals.size(); ++i) {
		ans[i] = std::stod(vals[i]);
	}
	return ans;
}

bool equalDoubleArrays(double* v1, double *v2, int size) {
    for (int i = 0; i < size; ++i) {
        if (abs(v1[i]-v2[i]) > 1e-3) {
            cout << endl;
            return false;
        }
    }
    return true;
}

typedef struct {
	int X1, Y1;
	int X2, Y2;
	int Increment;
	int UsingYIndex;
	int DeltaX, DeltaY;
	int DTerm;
	int IncrE, IncrNE;
	int XIndex, YIndex;
	int Flipped;
} bresenham_param_t;


void ContXY2Cell(double x, double y, short unsigned int* pX, short unsigned int *pY, int x_size, int y_size) {
	double cellsize = 1.0;
	//take the nearest cell
	*pX = (int)(x/(double)(cellsize));
	if( x < 0) *pX = 0;
	if( *pX >= x_size) *pX = x_size-1;

	*pY = (int)(y/(double)(cellsize));
	if( y < 0) *pY = 0;
	if( *pY >= y_size) *pY = y_size-1;
}


void get_bresenham_parameters(int p1x, int p1y, int p2x, int p2y, bresenham_param_t *params) {
	params->UsingYIndex = 0;

	if (fabs((double)(p2y-p1y)/(double)(p2x-p1x)) > 1)
		(params->UsingYIndex)++;

	if (params->UsingYIndex)
		{
			params->Y1=p1x;
			params->X1=p1y;
			params->Y2=p2x;
			params->X2=p2y;
		}
	else
		{
			params->X1=p1x;
			params->Y1=p1y;
			params->X2=p2x;
			params->Y2=p2y;
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

	params->DeltaX=params->X2-params->X1;
	params->DeltaY=params->Y2-params->Y1;

	params->IncrE=2*params->DeltaY*params->Increment;
	params->IncrNE=2*(params->DeltaY-params->DeltaX)*params->Increment;
	params->DTerm=(2*params->DeltaY-params->DeltaX)*params->Increment;

	params->XIndex = params->X1;
	params->YIndex = params->Y1;
}

void get_current_point(bresenham_param_t *params, int *x, int *y) {
	if (params->UsingYIndex) {
        *y = params->XIndex;
        *x = params->YIndex;
        if (params->Flipped)
            *x = -*x;
    }
	else {
        *x = params->XIndex;
        *y = params->YIndex;
        if (params->Flipped)
            *y = -*y;
    }
}

int get_next_point(bresenham_param_t *params) {
	if (params->XIndex == params->X2) {
        return 0;
    }
	params->XIndex += params->Increment;
	if (params->DTerm < 0 || (params->Increment < 0 && params->DTerm <= 0))
		params->DTerm += params->IncrE;
	else {
        params->DTerm += params->IncrNE;
        params->YIndex += params->Increment;
	}
	return 1;
}



int IsValidLineSegment(double x0, double y0, double x1, double y1, double*	map,
			 int x_size, int y_size) {
	bresenham_param_t params;
	int nX, nY; 
	short unsigned int nX0, nY0, nX1, nY1;

	//printf("checking link <%f %f> to <%f %f>\n", x0,y0,x1,y1);
		
	//make sure the line segment is inside the environment
	if(x0 < 0 || x0 >= x_size ||
		x1 < 0 || x1 >= x_size ||
		y0 < 0 || y0 >= y_size ||
		y1 < 0 || y1 >= y_size)
		return 0;

	ContXY2Cell(x0, y0, &nX0, &nY0, x_size, y_size);
	ContXY2Cell(x1, y1, &nX1, &nY1, x_size, y_size);

	//printf("checking link <%d %d> to <%d %d>\n", nX0,nY0,nX1,nY1);

	//iterate through the points on the segment
	get_bresenham_parameters(nX0, nY0, nX1, nY1, &params);
	do {
		get_current_point(&params, &nX, &nY);
		if(map[GETMAPINDEX(nX,nY,x_size,y_size)] == 1)
			return 0;
	} while (get_next_point(&params));

	return 1;
}

int IsValidArmConfiguration(double* angles, int numofDOFs, double*	map,
			 int x_size, int y_size) {
    double x0,y0,x1,y1;
    int i;
		
	 //iterate through all the links starting with the base
	x1 = ((double)x_size)/2.0;
	y1 = 0;
	for(i = 0; i < numofDOFs; i++){
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

int IsValidArmConfiguration_custom(vector<double> angles, int numofDOFs, double* map,
			 int x_size, int y_size) {
    double x0,y0,x1,y1;
    int i;
		
	// cout << "IN VALID ARM CONFIG" << endl;

	 //iterate through all the links starting with the base
	x1 = ((double)x_size)/2.0;
	y1 = 0;
	for(i = 0; i < numofDOFs; i++){
		//compute the corresponding line segment
		x0 = x1;
		y0 = y1;
		x1 = x0 + LINKLENGTH_CELLS*cos(2*PI-angles[i]);
		y1 = y0 - LINKLENGTH_CELLS*sin(2*PI-angles[i]);

		//check the validity of the corresponding line segment
		if(!IsValidLineSegment(x0,y0,x1,y1,map,x_size,y_size))
			return 0;
	}    

	// cout << "EXITING VALID ARM CONFIG" << endl;

	return 1;
}


double calculateL2Norm(vector<double> vector1, vector<double> vector2, int numDOFs) {
    double sumOfSquares = 0.0;

    for (int i = 0; i < numDOFs; i++) {
		// cout << vector1[i] << " " << vector2[i] << endl; 
        sumOfSquares += (vector1[i] - vector2[i]) * (vector1[i] - vector2[i]);
    }

    return std::sqrt(sumOfSquares);
}


//*******************************************************************************************************************//
//                                                                                                                   //
//                                          DEFAULT PLANNER FUNCTION                                                 //
//                                                                                                                   //
//*******************************************************************************************************************//

static void planner(
			double* map,
			int x_size,
			int y_size,
			double* armstart_anglesV_rad,
			double* armgoal_anglesV_rad,
            int numofDOFs,
            double*** plan,
            int* planlength)
{
	//no plan by default
	*plan = NULL;
	*planlength = 0;
		
    //for now just do straight interpolation between start and goal checking for the validity of samples

    double distance = 0;
    int i,j;
    for (j = 0; j < numofDOFs; j++){
        if(distance < fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]))
            distance = fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]);
    }
    int numofsamples = (int)(distance/(PI/20));
    if(numofsamples < 2){
        printf("the arm is already at the goal\n");
        return;
    }
    *plan = (double**) malloc(numofsamples*sizeof(double*));
    int firstinvalidconf = 1;
    for (i = 0; i < numofsamples; i++){
        (*plan)[i] = (double*) malloc(numofDOFs*sizeof(double)); 
        for(j = 0; j < numofDOFs; j++){
            (*plan)[i][j] = armstart_anglesV_rad[j] + ((double)(i)/(numofsamples-1))*(armgoal_anglesV_rad[j] - armstart_anglesV_rad[j]);
        }
        if(!IsValidArmConfiguration((*plan)[i], numofDOFs, map, x_size, y_size) && firstinvalidconf) {
            firstinvalidconf = 1;
            printf("ERROR: Invalid arm configuration!!!\n");
        }
    }    
    *planlength = numofsamples;
    
    return;
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                              RRT IMPLEMENTATION                                                   //
//                                                                                                                   //
//*******************************************************************************************************************//

std::mt19937 gen(std::random_device{}());


vector<double> sample_random(int numDOFs, vector<double> vector_goal, bool goal_bias){

	double sigma = 0.6;			 // deviation from goal
	double goalBiasFactor = 0.5; 

    std::uniform_real_distribution<double> angle_distribution(0.0, 3.14);
	std::uniform_real_distribution<double> goal_distribution(0.0, sigma);


	vector<double> q_rand; 

	double* q = new double[numDOFs];

    // Generate a random vector of angles
	for (int i = 0; i < numDOFs; i++){

		if ((static_cast<double>(rand()) / RAND_MAX) < goalBiasFactor && goal_bias){
			double randomValue = goal_distribution(gen);
			q_rand.push_back(vector_goal[i] + randomValue);
		}
		else{
			double random_angle = angle_distribution(gen);
			q_rand.push_back(random_angle);
		}
	}
    
    return q_rand;
}


vector<double> extend(std::map<int, int> &rrt_parent_map, std::map<int, vector<double>> &rrt_node_map, vector<double> q_rand, int numDOFs, double epsilon,
						double *map, int x_size, int y_size, int iteration){

	vector<double> q_new; 
	vector<double> q_old; 
	double min_dist = std::numeric_limits<double>::max();
	int nearest_node; 
	double total_move_dist = 0;
	double max_move_dist; 
	double move_dist;  
	double step_length = 0.1;

	// Find nearest node
	for (auto node : rrt_node_map){
		double d = calculateL2Norm(node.second, q_rand, numDOFs);
		if (d < min_dist){
			min_dist = d;
			nearest_node = node.first; 
		}
	}

	q_new = rrt_node_map[nearest_node]; 
	q_old = q_new; 

	while(total_move_dist < epsilon){
		max_move_dist = 0; 

		for(int i = 0; i < numDOFs; i++){
			move_dist = ((q_rand[i] - rrt_node_map[nearest_node][i])/min_dist) * step_length;
			q_new[i] += move_dist; 
			max_move_dist = std::max(max_move_dist, abs(move_dist));
		}

		total_move_dist += max_move_dist;

		//Check if sampled configuration is valid
		if(!IsValidArmConfiguration_custom(q_new, numDOFs, map, x_size, y_size)){
			q_new = q_old;
			// cout << "Not valid!" << endl;
			break; 
		}

		q_old = q_new; 
	}  

	rrt_node_map[iteration] = q_new;
	rrt_parent_map[iteration] = nearest_node;

	return q_new;
}



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

	std::map<int, int> rrt_parent_map; 			// Node Number : Parent Number
	std::map<int, vector<double>> rrt_node_map; 		// Node Number : Angles

    vector<double> vector_start; 
	vector<double> vector_goal; 

	for (int i = 0; i < numofDOFs; i++){
		vector_start.push_back(armstart_anglesV_rad[i]);
		vector_goal.push_back(armgoal_anglesV_rad[i]);
	}


	double epsilon = 1.0; 
	int max_iter = 3000;
	int i = 1;
	bool found_path = false; 

	//Initialize tree with start node
	rrt_parent_map[i] = -1; 
	rrt_node_map[i] = vector_start; 

	cout << "Starting RRT Planner" << endl; 

	while (i < max_iter){
		// cout << "Iteration Number: " << i << endl; 

		// Sample a random configuration
		vector<double> q_rand = sample_random(numofDOFs, vector_goal, true);

		//Check if sampled configuration is valid
		if(!IsValidArmConfiguration_custom(q_rand, numofDOFs, map, x_size, y_size)){
			continue;
		}

		//Extend the tree and add q_new
		vector<double> q_new = extend(rrt_parent_map, rrt_node_map, q_rand, numofDOFs, epsilon, map, x_size, y_size, i + 1);

		//Check if sampled configuration is valid
		if(!IsValidArmConfiguration_custom(q_new, numofDOFs, map, x_size, y_size)){
			cout << "q_new is not valid!!!!" << endl;
			break; 
		}		


		//Calculate distance of q_new from goal
		double distance_to_goal = calculateL2Norm(q_new, vector_goal, numofDOFs);
		
		//If q_new is near goal, add goal and break
		if (distance_to_goal < epsilon){

			rrt_parent_map[i + 2] = i + 1; 
			rrt_node_map[i + 2] = vector_goal;

			found_path = true; 
			cout << "Found a path to the goal node" << endl; 
			break; 
		}

		i += 1; 
	}

	//Reconstruct path
	int current_node = i + 2;
	vector<vector<double>> path; 

	if (found_path){

		while (current_node > 0){

			path.push_back(rrt_node_map[current_node]);
			current_node = rrt_parent_map[current_node];

		}

		// Reversing temp path to hold nodes from start to goal
		reverse(path.begin(), path.end());

		cout << "THE PATH FROM START TO GOAL IS BELOW" << endl;

		for (int i = 0; i < path.size(); i++){
			for (int j = 0; j <= numofDOFs; j++){
				cout << path[i][j] << " ";
			}
			cout << endl;
		}


		// Allocate memory for the plan
		*plan = (double**) malloc(path.size()*sizeof(double*));
		for (int i = 0; i < path.size(); i++) {
			(*plan)[i] = (double*) malloc(numofDOFs*sizeof(double)); 
			for(int j = 0; j < numofDOFs; j++){
            	(*plan)[i][j] = path[i][j];
        	}
		}	

		*planlength = path.size();

	}
	else{
		cout << "Could not find a path to the goal" << endl;
	}

	cout << "SUCCESS! Found plan with " << path.size() << " steps" << endl;


	cout << "Number of vertices: " << endl;
	cout << i + 2 << endl; 

	return; 

}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                         RRT CONNECT IMPLEMENTATION                                                //
//                                                                                                                   //
//*******************************************************************************************************************//


vector<double> connect(std::map<int, int> &rrt_parent_map, std::map<int, vector<double>> &rrt_node_map, vector<double> q_rand, int numDOFs, double epsilon,
						double *map, int x_size, int y_size, int iteration){

	vector<double> q_new; 
	vector<double> q_old; 
	double min_dist = std::numeric_limits<double>::max();
	int nearest_node; 
	double total_move_dist = 0;
	double max_move_dist; 
	double move_dist;  
	double step_length = 0.1;
	double threshold = 1000; 


	// Find nearest node
	for (auto node : rrt_node_map){
		double d = calculateL2Norm(node.second, q_rand, numDOFs);
		if (d < min_dist){
			min_dist = d;
			nearest_node = node.first; 
		}
	}

	q_new = rrt_node_map[nearest_node]; 
	q_old = q_new; 


	while(true){
		max_move_dist = 0; 

		for(int i = 0; i < numDOFs; i++){
			move_dist = ((q_rand[i] - rrt_node_map[nearest_node][i])/min_dist) * step_length;
			q_new[i] += move_dist; 
			max_move_dist = std::max(max_move_dist, abs(move_dist));
		}

		total_move_dist += max_move_dist;

		//Check if sampled configuration is valid
		if(!IsValidArmConfiguration_custom(q_new, numDOFs, map, x_size, y_size)){
			q_new = q_old;
			break; 
		}

		if (calculateL2Norm(rrt_node_map[nearest_node], q_new, numDOFs) < epsilon){
			break; 
		}

		q_old = q_new; 
	}  

	rrt_node_map[iteration] = q_new;
	rrt_parent_map[iteration] = nearest_node;

	return q_new;
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

	std::map<int, int> rrt_parent_map_A; 			// Node Number : Parent Number
	std::map<int, vector<double>> rrt_node_map_A; 		// Node Number : Angles
	std::map<int, int> rrt_parent_map_B; 			// Node Number : Parent Number
	std::map<int, vector<double>> rrt_node_map_B; 		// Node Number : Angles

    vector<double> vector_start; 
	vector<double> vector_goal; 

	for (int i = 0; i < numofDOFs; i++){
		vector_start.push_back(armstart_anglesV_rad[i]);
		vector_goal.push_back(armgoal_anglesV_rad[i]);
	}

    
	double epsilon = 1.0; 
	int max_iter = 3000;
	int i = 1;
	bool found_path = false; 

	//Initialize tree A with start node
	rrt_parent_map_A[i] = -1; 
	rrt_node_map_A[i] = vector_start; 	

	//Initialize tree B with goal node
	rrt_parent_map_B[i] = -1; 
	rrt_node_map_B[i] = vector_goal; 

	vector<double> q_new_A; 
	vector<double> q_new_B; 	
	int node_joined; 

	bool swap = false; 

	cout << "Starting RRT Connect Planner" << endl; 

	while (i < max_iter){

		// cout << "Iteration Number: " << i << endl; 

		// Sample a random configuration
		vector<double> q_rand = sample_random(numofDOFs, vector_goal, false);

		//Check if sampled configuration is valid
		if(!IsValidArmConfiguration_custom(q_rand, numofDOFs, map, x_size, y_size)){
			continue;
		}

		if (!swap){
			//Extend the tree A and add q_new
			q_new_A = extend(rrt_parent_map_A, rrt_node_map_A, q_rand, numofDOFs, epsilon, map, x_size, y_size, i + 1);

			//Check if sampled configuration is valid
			if(!IsValidArmConfiguration_custom(q_new_A, numofDOFs, map, x_size, y_size)){
				cout << "q_new_A is not valid!!!!" << endl;
				break; 
			}		

			q_new_B = connect(rrt_parent_map_B, rrt_node_map_B, q_new_A, numofDOFs, epsilon, map, x_size, y_size, i + 1);

			//Check if sampled configuration is valid
			if(!IsValidArmConfiguration_custom(q_new_B, numofDOFs, map, x_size, y_size)){
				cout << "q_new_B is not valid!!!!" << endl;
				break; 
			}	

			swap = true; 
		}
		else{

			//Extend the tree A and add q_new
			q_new_B = extend(rrt_parent_map_B, rrt_node_map_B, q_rand, numofDOFs, epsilon, map, x_size, y_size, i + 1);

			//Check if sampled configuration is valids
			if(!IsValidArmConfiguration_custom(q_new_B, numofDOFs, map, x_size, y_size)){
				cout << "q_new_B is not valid!!!!" << endl;
				break; 
			}		

			q_new_A = connect(rrt_parent_map_A, rrt_node_map_A, q_new_B, numofDOFs, epsilon, map, x_size, y_size, i + 1);

			//Check if sampled configuration is valid
			if(!IsValidArmConfiguration_custom(q_new_A, numofDOFs, map, x_size, y_size)){
				cout << "q_new_A is not valid!!!!" << endl;
				break; 
			}	

			swap = false; 
		}
	
		//Calculate distance of q_new_A from q_new_B
		double distance_between_trees = calculateL2Norm(q_new_A, q_new_B, numofDOFs);

		// cout << "Distance between trees: " << distance_between_trees << endl; 

		if (distance_between_trees < epsilon){

			if (swap){
				rrt_node_map_B[i + 2] = q_new_A;
				rrt_parent_map_B[i + 2] = i + 1; 
				found_path = true; 
				cout << "TREE B has joined with TREE A at Node : " << i + 2 << endl;
				node_joined = i + 2; 
			}
			else{
				rrt_node_map_A[i + 2] = q_new_B;
				rrt_parent_map_A[i + 2] = i + 1; 
				found_path = true; 
				cout << "TREE A has joined with TREE B at Node : " << i + 2 << endl;	
				node_joined = i + 2; 		
			}
			break;
			}

		i += 1; 
		}


	//Reconstruct path
	int current_node = i + 1;
	vector<vector<double>> b_path; 
	vector<vector<double>> a_path; 
	vector<vector<double>> path;
	bool joined = false; 

	// cout << "STARTING TO RECONSTRUCT PATH NOW!" << endl;

	if (found_path){

		while (current_node > 0){
			b_path.push_back(rrt_node_map_B[current_node]);
			current_node = rrt_parent_map_B[current_node];
		}

		current_node = i + 1; 

		while (current_node > 0){

			a_path.push_back(rrt_node_map_A[current_node]);
			current_node = rrt_parent_map_A[current_node];
			
		}

		// Reversing temp path to hold nodes from start to goal
		reverse(a_path.begin(), a_path.end());


		for (auto x : a_path){
			path.push_back(x); 
		}

		for (auto y : b_path){
			path.push_back(y);
		}

		cout << "THE PATH FROM START TO GOAL IS BELOW" << endl;

		for (int i = 0; i < path.size(); i++){
			for (int j = 0; j <= numofDOFs; j++){
				cout << path[i][j] << " ";
			}
			cout << endl;
		}


		// Allocate memory for the plan
		*plan = (double**) malloc(path.size()*sizeof(double*));
		for (int i = 0; i < path.size(); i++) {
			(*plan)[i] = (double*) malloc(numofDOFs*sizeof(double)); 
			for(int j = 0; j < numofDOFs; j++){
            	(*plan)[i][j] = path[i][j];
        	}
		}	

		*planlength = path.size();

	}
	else{
		cout << "Could not find a path to the goal" << endl;
	}

	cout << "SUCCESS! Found plan with " << path.size() << " steps" << endl;


	cout << "Number of vertices: " << endl;
	cout << i + 2 << endl; 

	return; 

}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                           RRT STAR IMPLEMENTATION                                                 //
//                                                                                                                   //
//*******************************************************************************************************************//

vector<double> extend_star(std::map<int, int> &rrt_parent_map, std::map<int, vector<double>> &rrt_node_map, std::map<int, int> &rrt_cost_map, vector<double> q_rand, int numDOFs, double epsilon,
						double *map, int x_size, int y_size, int iteration){

	vector<double> q_new; 
	vector<double> q_old; 
	double min_dist = std::numeric_limits<double>::max();
	double min_cost = std::numeric_limits<double>::max();
	int nearest_node; 
	double total_move_dist = 0;
	double max_move_dist; 
	double move_dist;  
	double step_length = 0.1;
	double radius = epsilon * 3;

	// Find nearest node
	for (auto node : rrt_node_map){
		double d = calculateL2Norm(node.second, q_rand, numDOFs);
		if (d < min_dist){
			min_dist = d;
			nearest_node = node.first; 
		}
	}

	q_new = rrt_node_map[nearest_node]; 
	q_old = q_new; 

	while(total_move_dist < epsilon){
		max_move_dist = 0; 

		for(int i = 0; i < numDOFs; i++){
			move_dist = ((q_rand[i] - rrt_node_map[nearest_node][i])/min_dist) * step_length;
			q_new[i] += move_dist; 
			max_move_dist = std::max(max_move_dist, abs(move_dist));
		}

		total_move_dist += max_move_dist;

		//Check if sampled configuration is valid
		if(!IsValidArmConfiguration_custom(q_new, numDOFs, map, x_size, y_size)){
			q_new = q_old;
			// cout << "Not valid!" << endl;
			break; 
		}

		q_old = q_new; 
	}  

	// rrt_node_map[iteration] = q_new;
	// rrt_parent_map[iteration] = nearest_node;
	// rrt_cost_map[iteration] = rrt_cost_map[nearest_node] + 1; 

	min_cost = rrt_cost_map[nearest_node] + 1; 

	// Find nearest node within radius with least cost
	for (auto node : rrt_node_map){
		if (calculateL2Norm(node.second, q_new, numDOFs) < radius){
			double cost = rrt_cost_map[node.first];
			if (cost < min_cost){
				min_cost = cost;
				nearest_node = node.first; 
			}
		}
	}

	// Checking for collisions to least-cost node
	total_move_dist = 0;
	while(total_move_dist < radius){
		max_move_dist = 0; 

		for(int i = 0; i < numDOFs; i++){
			move_dist = ((q_rand[i] - rrt_node_map[nearest_node][i])/min_dist) * step_length;
			q_new[i] += move_dist; 
			max_move_dist = std::max(max_move_dist, abs(move_dist));
		}

		total_move_dist += max_move_dist;

		//Check if sampled configuration is valid
		if(!IsValidArmConfiguration_custom(q_new, numDOFs, map, x_size, y_size)){
			q_new = q_old;
			// cout << "Not valid!" << endl;
			break; 
		}

		q_old = q_new; 
	}  

	rrt_node_map[iteration] = q_new;
	rrt_parent_map[iteration] = nearest_node;
	rrt_cost_map[iteration] = rrt_cost_map[nearest_node] + 1;

	double cur_cost = rrt_cost_map[nearest_node] + 1;

	// Rewiring other nodes in neighborhood
	for (auto node : rrt_node_map){
		if (calculateL2Norm(node.second, q_new, numDOFs) < radius){
			if (cur_cost < rrt_cost_map[node.first]){
				rrt_parent_map[node.first] = iteration;
				rrt_cost_map[node.first] = cur_cost;
			}
		}
	}

	return q_new;
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

	std::map<int, int> rrt_parent_map; 					// Node Number : Parent Number
	std::map<int, vector<double>> rrt_node_map; 		// Node Number : Angles
	std::map<int, int> rrt_cost_map; 					// Node Number : Cost

    vector<double> vector_start; 
	vector<double> vector_goal; 

	for (int i = 0; i < numofDOFs; i++){
		vector_start.push_back(armstart_anglesV_rad[i]);
		vector_goal.push_back(armgoal_anglesV_rad[i]);
	}


	double epsilon = 1.0; 
	int max_iter = 3000;
	int i = 1;
	bool found_path = false; 

	//Initialize tree with start node
	rrt_parent_map[i] = -1; 
	rrt_node_map[i] = vector_start; 
	rrt_cost_map[i] = 1; 

	cout << "Starting RRT Star Planner" << endl; 

	while (i < max_iter){
		// cout << "Iteration Number: " << i << endl; 

		// Sample a random configuration
		vector<double> q_rand = sample_random(numofDOFs, vector_goal, true);

		//Check if sampled configuration is valid
		if(!IsValidArmConfiguration_custom(q_rand, numofDOFs, map, x_size, y_size)){
			continue;
		}

		//Extend the tree and add q_new
		vector<double> q_new = extend_star(rrt_parent_map, rrt_node_map, rrt_cost_map, q_rand, numofDOFs, epsilon, map, x_size, y_size, i + 1);

		//Check if sampled configuration is valid
		if(!IsValidArmConfiguration_custom(q_new, numofDOFs, map, x_size, y_size)){
			cout << "q_new is not valid!!!!" << endl;
			break; 
		}		


		//Calculate distance of q_new from goal
		double distance_to_goal = calculateL2Norm(q_new, vector_goal, numofDOFs);
		
		//If q_new is near goal, add goal and break
		if (distance_to_goal < epsilon){

			rrt_parent_map[i + 2] = i + 1; 
			rrt_node_map[i + 2] = vector_goal;

			found_path = true; 
			cout << "Found a path to the goal node" << endl; 
			break; 
		}

		i += 1; 
	}

	//Reconstruct path
	int current_node = i + 2;
	vector<vector<double>> path; 

	if (found_path){

		while (current_node > 0){

			path.push_back(rrt_node_map[current_node]);
			current_node = rrt_parent_map[current_node];

		}

		// Reversing temp path to hold nodes from start to goal
		reverse(path.begin(), path.end());

		cout << "THE PATH FROM START TO GOAL IS BELOW" << endl;

		for (int i = 0; i < path.size(); i++){
			for (int j = 0; j <= numofDOFs; j++){
				cout << path[i][j] << " ";
			}
			cout << endl;
		}


		// Allocate memory for the plan
		*plan = (double**) malloc(path.size()*sizeof(double*));
		for (int i = 0; i < path.size(); i++) {
			(*plan)[i] = (double*) malloc(numofDOFs*sizeof(double)); 
			for(int j = 0; j < numofDOFs; j++){
            	(*plan)[i][j] = path[i][j];
        	}
		}	

		*planlength = path.size();

	}
	else{
		cout << "Could not find a path to the goal" << endl;
	}

	cout << "SUCCESS! Found plan with " << path.size() << " steps" << endl;


	cout << "Number of vertices: " << endl;
	cout << i + 2 << endl; 

	return; 


}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                              PRM IMPLEMENTATION                                                   //
//                                                                                                                   //
//*******************************************************************************************************************//

std::map<int, vector<int>> prm_connections_map;
std::map<int, vector<double>> prm_node_map;


bool customComparator(const vector<int>& a, const vector<int>& b) {
    return a[1] > b[1]; // Compare the f_value 
}

bool customComparator_2(const std::pair<int, double>& a, std::pair<int, double>& b) {
    return a.second > b.second; // Compare the f_value 
}

bool path_computed = false; 
int current_step = 0;
int backward_count = 0;
bool reached_goal_once = false;

/* A-Star Planner */

vector<int> planner(
    int robotposeX,
	int goalposeX,
	int number_of_nodes,
	int numDOFs
    )
{
	vector<int> path; 

    // 8-connected grid
    int dX[NUMOFDIRS] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[NUMOFDIRS] = {-1,  0,  1, -1,  1, -1, 0, 1};


    if (robotposeX == goalposeX){
        reached_goal_once = true;           // boolean triggers when the first goal has been reached 
		path.push_back(goalposeX);
    }

    // If already reached the first goal pose,
    if (reached_goal_once){
        
            return path;
        }
    

    // initializing datastructures
    vector<int> h_values(number_of_nodes, 1); 
    vector<int> g_values(number_of_nodes, std::numeric_limits<int>::max()); 
    vector<int> status(number_of_nodes, -1);                // status = -1 if not explored, 0 if open, 1 if closed
    std::priority_queue<vector<int>, vector<vector<int>>, decltype(&customComparator)> open(&customComparator); //sorting based on decreasing value in 3rd column (f value)
    vector<int> combine; 
    int robot_pose; 
    int parent_pose;
    std::map<int, int> closed; 	// robotx : parentx


    // initializing variables
    int f_value = 0;
    int first_directionX;
    int first_directionY; 
    int count = 0;              // to keep track of number of iterations of the planner 
    int parentx = robotposeX; 
    int h_weight = 1; 


    g_values[robotposeX - 1] = 0;
    h_values[robotposeX - 1] = calculateL2Norm(prm_node_map[robotposeX], prm_node_map[goalposeX], numDOFs);
    f_value = g_values[robotposeX - 1] + h_weight * h_values[robotposeX - 1];
    combine = {robotposeX, f_value, parentx};
    open.push(combine);
    status[robotposeX - 1] = 0;


    // Implementing A* search to calculate g-values for all explored cells leading up to the goal cell

    while (open.size() > 0 && status[goalposeX - 1] != 1){

        int best_robotX = open.top()[0];       // s with smallest f in open is the last element

        int parentx = open.top()[2];

        count += 1;
        open.pop();                        // removing s with smallest f from open
        robot_pose = best_robotX;
        parent_pose = parentx;
        closed[robot_pose] = parent_pose;       // storing parent of every explored cell in closed list

        for(int i = 0; i < prm_connections_map[robot_pose].size(); i++)
            {  

            int newx = prm_connections_map[robot_pose][i];           // moving one step in every direction in x
            
    
            if (newx >= 1 && newx <= number_of_nodes)             // if new position is within map size
                {
                //if new position is valid and free
				int cost_next_step = 1;					// can try to replace with L2 norm 
				if (g_values[newx - 1] > g_values[best_robotX - 1] + cost_next_step){
					g_values[newx - 1] = g_values[best_robotX - 1] + cost_next_step;

					h_values[robotposeX - 1] = abs(calculateL2Norm(prm_node_map[newx], prm_node_map[goalposeX], numDOFs)); 
					f_value = g_values[newx - 1] + h_weight * h_values[newx - 1];
					
					combine = {newx, f_value, best_robotX};
					if (status[newx - 1] == -1){
						open.push(combine);
						status[newx - 1] = 0;
					}
				}
                }
            status[best_robotX - 1] = 1;     // marking current position as closed
            }
        }

	if (status[goalposeX - 1] == 1){
		cout << "A-Star found a path to goal!" << endl; 
	}
	else{
		cout << "A-Star did NOT find path to goal!" << endl; 
	}
	

    // Backtracking from goal to start by searching adjacent cells with least g value

    int currentX = goalposeX;

    while (currentX != robotposeX) {
        path.push_back(currentX);

        int current_pose = currentX;
        parentx = closed[current_pose];

        currentX = parentx;
    }

    path_computed = true;

    // Add the starting position to the path
    path.push_back(robotposeX);
	
    return path;
}



bool check_collisions(vector<double> node1, vector<double> node2, int numDOFs, double *map, int x_size, int y_size){

	vector<double> q_new; 
	vector<double> q_old; 
	double min_dist = std::numeric_limits<double>::max();
	int nearest_node; 
	double total_move_dist = 0;
	double max_move_dist; 
	double move_dist;  
	double step_length = 0.1;


	double distance_between_nodes = calculateL2Norm(node1, node2, numDOFs);

	q_new = node1;

	// cout << "Checking collision" << endl; 

	while(total_move_dist < distance_between_nodes){
		max_move_dist = 0; 

		// cout << "Distance between nodes: " << distance_between_nodes << " -- Total Move Distance: " << total_move_dist << endl;  

		for(int i = 0; i < numDOFs; i++){
			move_dist = ((node2[i] - node1[i])/distance_between_nodes) * step_length;
			q_new[i] += move_dist; 
			max_move_dist = std::max(max_move_dist, abs(move_dist));
		}

		total_move_dist += max_move_dist;

		//Check if sampled configuration is valid
		if(!IsValidArmConfiguration_custom(q_new, numDOFs, map, x_size, y_size)){
			q_new = q_old;
			// cout << "Not valid!" << endl;
			return false; 
		}
	}  

	// cout << "DONE CHECKING COLLISION" << endl;

	return true; 
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

    vector<double> vector_start; 
	vector<double> vector_goal; 

	for (int i = 0; i < numofDOFs; i++){
		vector_start.push_back(armstart_anglesV_rad[i]);
		vector_goal.push_back(armgoal_anglesV_rad[i]);
	}

	// double epsilon = 1.5; 
	int max_iter = 1000;
	int i = 1;
	bool found_path = false; 
	double radius = 1.5;
	int K_vertices = 20;
	std::map<int, bool> connected_map; 
	bool goal_reached = false; 
	std::pair<int, double> node_distance; 
	// vector<std::pair<int, double>> k_nearest; 

	//Initialize tree with start node
	prm_node_map[i] = vector_start; 
	// prm_node_map[max_iter + 1] = vector_goal; 
	
	cout << "Starting PRM Planner" << endl;

	while (i <= max_iter && !goal_reached){

		std::priority_queue<std::pair<int, double>, vector<std::pair<int, double>>, decltype(&customComparator_2)> k_nearest(&customComparator_2); //sorting based on decreasing value in 3rd column (f value)
 
		// Sample a random configuration
		vector<double> q_rand = sample_random(numofDOFs, vector_goal, false);

		//Check if sampled configuration is valid
		if(!IsValidArmConfiguration_custom(q_rand, numofDOFs, map, x_size, y_size)){
			continue;
		}
	
		
		if (i == max_iter){
			q_rand = vector_goal; 
			i -= 1; 
			goal_reached = true; 
		}

		// Sort all nodes in the graph in a priority queue from q_rand, based on distance 
		for (auto node : prm_node_map){
			double distance = calculateL2Norm(prm_node_map[node.first], q_rand, numofDOFs);
			
			node_distance.first = node.first; 
			node_distance.second = distance; 

			k_nearest.push(node_distance);
		}


		// Check the first K_vertices for collision, connect to q_rand if no collision
		for (int j = 0; j < K_vertices; j++){

				bool collision_free = check_collisions(prm_node_map[k_nearest.top().first], q_rand, numofDOFs, map, x_size, y_size);
				k_nearest.pop();

				if (collision_free){
					prm_connections_map[i + 1].push_back(k_nearest.top().first);
					prm_connections_map[k_nearest.top().first].push_back(i + 1);
				}	
		}

		prm_node_map[i + 1] = q_rand;

		i += 1;
	}

	vector<vector<double>> path; 
	cout << "A-Star to find path from start to goal" << endl;

	// Use A* to search map
	int start_node = 1; 
	int goal_node = max_iter; 
	int number_of_nodes = max_iter + 1; 
	vector<int> a_star_path = planner(start_node, goal_node, number_of_nodes, numofDOFs);

	// Reversing temp path to hold nodes from start to goal
	reverse(a_star_path.begin(), a_star_path.end());

	for (auto x : a_star_path){
		path.push_back(prm_node_map[x]);
		cout << x << endl;
	}



	cout << "THE PATH FROM START TO GOAL IS BELOW" << endl;

	for (int i = 0; i < path.size(); i++){
		for (int j = 0; j <= numofDOFs; j++){
			cout << path[i][j] << " ";
		}
		cout << endl;
	}


	// Allocate memory for the plan
	*plan = (double**) malloc(path.size()*sizeof(double*));
	for (int i = 0; i < path.size(); i++) {
		(*plan)[i] = (double*) malloc(numofDOFs*sizeof(double)); 
		for(int j = 0; j < numofDOFs; j++){
			(*plan)[i][j] = path[i][j];
		}
	}	

	*planlength = path.size();

	cout << "SUCCESS! Found plan with " << path.size() << " steps" << endl;


	cout << "Number of vertices: " << endl;
	cout << max_iter << endl; 

	return; 


}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                                MAIN FUNCTION                                                      //
//                                                                                                                   //
//*******************************************************************************************************************//

/** Your final solution will be graded by an grading script which will
 * send the default 6 arguments:
 *    map, numOfDOFs, commaSeparatedStartPos, commaSeparatedGoalPos, 
 *    whichPlanner, outputFilePath
 * An example run after compiling and getting the planner.out executable
 * >> ./planner.out map1.txt 5 1.57,0.78,1.57,0.78,1.57 0.392,2.35,3.14,2.82,4.71 0 myOutput.txt
 * >> ./planner.out map2.txt 5 0.642525,2.2704,0.250386,0.689152,2.36339 0.392,2.35,3.14,2.82,4.71 0 myOutput.txt
 * See the hw handout for full information.
 * If you modify this for testing (e.g. to try out different hyper-parameters),
 * make sure it can run with the original 6 commands.
 * Programs that do not will automatically get a 0.
 * */
int main(int argc, char** argv) {
	double* map;
	int x_size, y_size;

	tie(map, x_size, y_size) = loadMap(argv[1]);
	const int numOfDOFs = std::stoi(argv[2]);
	double* startPos = doubleArrayFromString(argv[3]);
	double* goalPos = doubleArrayFromString(argv[4]);
	int whichPlanner = std::stoi(argv[5]);
	string outputFile = argv[6];

	if(!IsValidArmConfiguration(startPos, numOfDOFs, map, x_size, y_size)||
			!IsValidArmConfiguration(goalPos, numOfDOFs, map, x_size, y_size)) {
		throw runtime_error("Invalid start or goal configuration!\n");
	}

	///////////////////////////////////////
	//// Feel free to modify anything below. Be careful modifying anything above.

	double** plan = NULL;
	int planlength = 0;

	auto start = std::chrono::high_resolution_clock::now();

    // Call the corresponding planner function
    if (whichPlanner == PRM)
    {
        plannerPRM(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
    }
    else if (whichPlanner == RRT)
    {
        plannerRRT(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
    }
    else if (whichPlanner == RRTCONNECT)
    {
        plannerRRTConnect(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
    }
    else if (whichPlanner == RRTSTAR)
    {
        plannerRRTStar(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
    }
    else
    {
        planner(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
    }

	//// Feel free to modify anything above.
	//// If you modify something below, please change it back afterwards as my 
	//// grading script will not work and you will recieve a 0.
	///////////////////////////////////////

    // Your solution's path should start with startPos and end with goalPos
    if (!equalDoubleArrays(plan[0], startPos, numOfDOFs) || 
    	!equalDoubleArrays(plan[planlength-1], goalPos, numOfDOFs)) {
		throw std::runtime_error("Start or goal position not matching");
	}

	auto end = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

	cout << "Time taken: " << endl; 
	cout << duration * 1e-6 << endl;

	/** Saves the solution to output file
	 * Do not modify the output log file output format as it is required for visualization
	 * and for grading.
	 */
	std::ofstream m_log_fstream;
	m_log_fstream.open(outputFile, std::ios::trunc); // Creates new or replaces existing file
	if (!m_log_fstream.is_open()) {
		throw std::runtime_error("Cannot open file");
	}
	m_log_fstream << argv[1] << endl; // Write out map name first
	/// Then write out all the joint angles in the plan sequentially
	for (int i = 0; i < planlength; ++i) {
		for (int k = 0; k < numOfDOFs; ++k) {
			m_log_fstream << plan[i][k] << ",";
		}
		m_log_fstream << endl;
	}
}
