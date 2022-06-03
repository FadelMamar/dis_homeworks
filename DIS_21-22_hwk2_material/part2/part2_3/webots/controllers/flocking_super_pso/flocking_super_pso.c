/*****************************************************************************/
/* File:         flocking_super.c                                            */
/* Version:      1.0                                                         */
/* Date:         10-Oct-14                                                   */
/* Description:  Reynolds flocking control 				*/
/*                                                                            */
/* Author:       10-Oct-14 by Ali marjovi			           */
/* Last revision: 10-March-22 by Kagan Erunsal			*/
/*****************************************************************************/

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/supervisor.h>


#include "../flock_param.h"
#define TIME_STEP	32		// [ms] Length of time step


WbNodeRef robs[FLOCK_SIZE];		// Robots nodes
WbFieldRef robs_trans[FLOCK_SIZE];	// Robots translation fields
WbFieldRef robs_rotation[FLOCK_SIZE];	// Robots rotation fields
WbDeviceTag emitter_loc;			// Single emitter

float loc[FLOCK_SIZE][3];		// Location of everybody in the flock

#define INTER_VEHICLE_COM 0 // Set 1 if there is intervehicle communication (not supported here)
#define VERBOSE 0

float migrx = 0; 			// Migration component x
float migry = -2.5;			// Migration component y

// List of destinations to reach by the flock 
static const double destinations[][2] = {
									{0.6,-0.42},
									{-1.33,0.8},
									{0.15,1.85}
};
static int num_destination = sizeof(destinations)/sizeof(double)/2;
int curr_dest = 0;

/**
 * Multi migration targets for single flock case
 */
void update_migration(){
	
	double center[2] = {0.,0.}; // flock center 

	for (int i=0;i<FLOCK_SIZE;i++) {
		// Get data
		center[0] += wb_supervisor_field_get_sf_vec3f(robs_trans[i])[0]/FLOCK_SIZE; // X
		center[1] += wb_supervisor_field_get_sf_vec3f(robs_trans[i])[1]/FLOCK_SIZE; // Y
	}
	double distance = sqrt(pow(center[0]-migrx,2) + pow(center[1]-migry,2));
	if(distance < .3){
		printf("Reached destination %d\n",curr_dest);
		curr_dest++;
	}
	if(curr_dest >= num_destination){
		printf("All destinations are reached, stopping simulation\n"); 
		wb_supervisor_simulation_set_mode(WB_SUPERVISOR_SIMULATION_MODE_PAUSE);
	}
	//printf("dist: %.2lf, migrx:%.2lf,migry:%.2lf,centery:%.2lf,centery:%.2lf,curr_dest:%d\n",distance,migrx, migry, center[0],center[1],curr_dest);

	migrx = destinations[curr_dest][0];
	migry = destinations[curr_dest][1];

	WbNodeRef mig_target = wb_supervisor_node_get_from_def("migration_target");
	if(mig_target == NULL) return;
	WbFieldRef mig_trans = wb_supervisor_node_get_field(mig_target,"translation");
	double trans[3] = {migrx,migry,0};
	wb_supervisor_field_set_sf_vec3f(mig_trans,trans);
}

/*
 * Initialize flock position and devices
 */
void reset(void) {

	wb_robot_init();

	emitter_loc = wb_robot_get_device("emitter_loc");
	if (emitter_loc==0) printf("missing emitter_loc\n");
	
	char rob[8] = "epuck0";
	int i;
	for (i=0;i<FLOCK_SIZE;i++) {
		sprintf(rob,"epuck%d",i);
		robs[i] = wb_supervisor_node_get_from_def(rob);
		robs_trans[i] = wb_supervisor_node_get_field(robs[i],"translation");
		robs_rotation[i] = wb_supervisor_node_get_field(robs[i],"rotation");
	}
}


void send_poses(void) {

  	char buffer[255];	// Buffer for sending data
	int i;
         
	for (i=0;i<FLOCK_SIZE;i++) {
		// Get data
		loc[i][0] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[0]; // X
		loc[i][1] = wb_supervisor_field_get_sf_vec3f(robs_trans[i])[1]; // Y
		int sign = wb_supervisor_field_get_sf_rotation(robs_rotation[i])[2] >= 0 ? 1 : -1;
		loc[i][2] = sign * wb_supervisor_field_get_sf_rotation(robs_rotation[i])[3]; // THETA

		// Send it out
		sprintf(buffer,"%d#%f#%f#%f##%f#%f",i,loc[i][0],loc[i][1],loc[i][2], migrx, migry);
		wb_emitter_send(emitter_loc,buffer,strlen(buffer));
	}
}


/*
 * Main function.
 */

int main(int argc, char *args[]) {

	reset();
	
	send_poses();
		
	for(;;) {

		wb_robot_step(TIME_STEP);

		update_migration();
			
		send_poses();

	}
	
}
