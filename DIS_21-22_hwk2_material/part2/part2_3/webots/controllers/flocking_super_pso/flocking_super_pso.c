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
#include "pso.h"
#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/supervisor.h>

#define ROBOTS 5
#define MAX_ROB 5
#define ROB_RAD 0.035
#define ARENA_SIZE 1.89
#define TIME_STEP 32

#define NB 1                            // Number of neighbors on each side
#define LWEIGHT 2.0                     // Weight of attraction to personal best
#define NBWEIGHT 2.0                    // Weight of attraction to neighborhood best
#define VMAX 20.0                       // Maximum velocity particle can attain
#define MININIT 0                   // Lower bound on initialization value
#define MAXINIT 1.0                    // Upper bound on initialization value
#define ITS 20                          // Number of iterations to run
#define DATASIZE 4         // Number of elements in particle (2 Neurons with 8 proximity sensors 

/* Fitness definitions */
#define FIT_ITS 180                     // Number of fitness steps to run during evolution

#define FINALRUNS 10
#define NEIGHBORHOOD STANDARD
#define RADIUS 0.8

#include "../flock_param.h"
#define MAX_SPEED_WEB      	6.28    // Maximum wheel speed webots
#define WHEEL_RADIUS		0.0205	// Wheel radius (meters)
#define TIME_STEP	32		// [ms] Length of time step


WbNodeRef robs[FLOCK_SIZE];		// Robots nodes
WbFieldRef robs_trans[FLOCK_SIZE];	// Robots translation fields
WbFieldRef robs_rotation[FLOCK_SIZE];	// Robots rotation fields
WbDeviceTag emitter_loc;			// Single emitter
WbDeviceTag emitter_particles;

float loc[FLOCK_SIZE][3];		// Location of everybody in the flock

#define INTER_VEHICLE_COM 0 // Set 1 if there is intervehicle communication (not supported here)
#define VERBOSE 0

float migrx = 0; 			// Migration component x
float migry = -2.5;			// Migration component y
float avg[2] = {0,0};

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
	
	emitter_particles = wb_robot_get_device("emitter_particles");
	if (emitter_particles==0) printf("missing emitter_paticles\n");
	
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

double compute_flocking_fitness(){
        
       int i;
       	double old_avg[2] = {0,0};
       	double proj = 0;
       	double o = 0;
	double complexe_O[2] = {0,0};
	double avg_speed[2] = {0,0};
	double vector_migr[2] = {0,0};
	double migr_norm; 
       	
       	old_avg[0] = avg[0];
       	old_avg[1] = avg[1];
       	
       	avg[0] = 0;
       avg[1] = 0;
	// To Do : Compute orientation performance 

	
	for(int i=0; i<FLOCK_SIZE; i++) {
		complexe_O[0] += cosf(fabs(loc[i][2]));
		complexe_O[1] += sinf(fabs(loc[i][2]));
	}
	
	o = sqrt(powf(complexe_O[0],2) + pow(complexe_O[1],2))/FLOCK_SIZE;
	//printf("o: %f\n", o);


	// To Do : Compute distance performance 
	double d = 0;
	
	for(i=0; i<FLOCK_SIZE; i++){ 
        	avg[0] += loc[i][0]/FLOCK_SIZE;
        	avg[1] += loc[i][1]/FLOCK_SIZE;}

	 
	for(i=0; i<FLOCK_SIZE; i++){ 
        	d += fabs(sqrt(pow(loc[i][0]-avg[0],2)+pow(loc[i][1]-avg[1],2))-RULE1_THRESHOLD);}
        d /= FLOCK_SIZE;
        d += 1;
        d = 1/d;
        //printf("d: %f\n", d);
        


	// To Do : Compute velocity performance (if migratory urge enabled)
	double v = 1.;
	
	if(MIGRATORY_URGE){

		avg_speed[0] = 1000*(avg[0] - old_avg[0])/TIME_STEP; 
		avg_speed[1] = 1000*(avg[1] - old_avg[1])/TIME_STEP;
		
		vector_migr[0] = migrx - avg[0];
		vector_migr[1] = migry - avg[1];
              
		migr_norm = sqrt(pow(vector_migr[0],2) + pow(vector_migr[1],2));
		
		proj = (vector_migr[0]*avg_speed[0] + vector_migr[1]*avg_speed[1])/migr_norm;
		v = fmax(proj,0);
		v = v/(MAX_SPEED_WEB* WHEEL_RADIUS);
		
	}else {
	
		v = 1.;}
       v = 1.;
	//printf("v: %f\n", v);
	return (o)*(d)*(v);
}
// Distribute fitness functions among robots
void calc_fitness(double weights[ROBOTS][DATASIZE], double fit[ROBOTS], int its, int numRobs) {
  double buffer[255];
  int i,j;

  /* Send data to robots */
  for (i=0;i<numRobs;i++) {
    //random_pos(i);
    for (j=0;j<DATASIZE;j++) {
      buffer[j] = weights[i][j];
    }
  }

  wb_emitter_send(emitter_particles, (void *)buffer,(DATASIZE)*sizeof(double));

  for (i=0;i<numRobs;i++) {
    fit[i] = compute_flocking_fitness();
  }
}


/*
 * Main function.
 */

/* Optimization fitness function , used in pso.c */
/************************************************************************************/
/* Use the NEIGHBORHOOD definition at the top of this file to                        */
/* change the neighborhood type for the PSO. The possible values are:               */
/* STANDARD    : Local neighborhood with 2*NB (defined above) nearest neighbors     */
/*               NEIGHBORHOOD is set to STANDARD by default                         */
/* RAND_NB     : 2*NB random neighbors                                              */
/* NCLOSE_NB   : 2*NB closest neighbors                                             */
/* FIXEDRAD_NB : All robots within a defined radius are neighbors                   */
/************************************************************************************/
void fitness(double weights[ROBOTS][DATASIZE], double fit[ROBOTS], int neighbors[SWARMSIZE][SWARMSIZE]) {
  calc_fitness(weights,fit,FIT_ITS,ROBOTS);
}

int main(int argc, char *args[]) {
      //double weights[4] = {0,1,2,3};
      double fit; 				// Fitness of the current FINALRUN
      double endfit; 			// Best fitness over 10 runs
      double fitvals[FINALRUNS]; 		// Fitness values for final tests
      double w[MAX_ROB][DATASIZE]; 		// Weights to be send to robots (determined by pso() )
      double f[MAX_ROB];			// Evaluated fitness (modified by calc_fitness() )
      double bestfit, bestw[DATASIZE];	// Best fitness and weights
      char outbuffer[255];
      double *weights;
      reset();
	
	send_poses();
	
	weights = pso(SWARMSIZE,NB,LWEIGHT,NBWEIGHT,VMAX,MININIT,MAXINIT,ITS,DATASIZE,ROBOTS);
       
       sprintf(outbuffer,"%f#%f#%f#%f",weights[0], weights[1],weights[2], weights[3]);
       printf("%s",outbuffer);
       wb_emitter_send(emitter_particles, outbuffer,strlen(outbuffer));
	
	printf("weights_send: %f\n",weights[0]);
	printf("weights_send: %f\n",weights[1]);
	printf("weights_send: %f\n",weights[2]);
	printf("weights_send: %f\n",weights[3]);
	 /*	
	for(;;) {

		wb_robot_step(TIME_STEP);

		update_migration();
			
		send_poses();

             
		fit = 0.0;
	for (int i=0;i<MAX_ROB;i++) {
          for (int k=0;k<DATASIZE;k++)
              w[i][k] = weights[k];
       }
	
              
              for (int i=0;i<FINALRUNS;i+=MAX_ROB) {
                  printf("f: %f\n",compute_flocking_fitness());
                  calc_fitness(w,f,FIT_ITS,MAX_ROB);
                  for (int k=0;k<MAX_ROB && i+k<FINALRUNS;k++) {
                    fitvals[i+k] = f[k];
                    fit += f[k];
                  }
              }
              
              fit /= FINALRUNS;
              
              if (fit > bestfit) {
                bestfit = fit;
                for (int i = 0; i < DATASIZE; i++){
        	       bestw[i] = weights[i];}
        	    
              }

    endfit += fit/10; // average over the 10 runs
*/	

}