#include <stdio.h>
#include <math.h>
#include "pso.h"
#include <stdlib.h>
#include <string.h>
#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/supervisor.h>

#define ROBOTS 5						// Was previously 1
#define MAX_ROB 5
#define ROB_RAD 0.035
#define ARENA_SIZE .15
#define MAX_DIST 3.0
#define SENSOR_RANGE 0.3

#define TIME_STEP	32		// [ms] Length of time step

#define NB_SENSOR 8                     // Number of proximity sensors

/* PSO definitions */
#define NB 2                           // Number of neighbors on each side
#define LWEIGHT 2.0                     // Weight of attraction to personal best
#define NBWEIGHT 2.0                    // Weight of attraction to neighborhood best
#define VMAX 60.0                       // Maximum velocity particle can attain
#define MININIT 0.0                   // Lower bound on initialization value
#define MAXINIT 0.2                   // Upper bound on initialization value
#define ITS 10                          // Number of iterations to run

/* Neighborhood types */
#define STANDARD    -1
#define RAND_NB      0
#define NCLOSE_NB    1
#define FIXEDRAD_NB  2
#define MIGRATORY_URGE 1

#include "../flock_param.h"
#define MAX_SPEED_WEB      	6.28    // Maximum wheel speed webots
#define WHEEL_RADIUS		0.0205	// Wheel radius (meters)

/* Fitness definitions */
#define FIT_ITS 2000                     // Number of fitness steps to run during evolution

#define NOISY 1
#define FINALRUNS 10
#define NEIGHBORHOOD STANDARD
#define RADIUS 0.8
#define N_RUNS 1        // Change this to 10 for question 2.10

#define PI 3.1415926535897932384626433832795
enum {POS_X=0,POS_Y,POS_Z};

static WbNodeRef robs[MAX_ROB];
WbDeviceTag emitter[MAX_ROB];
WbDeviceTag emitter_loc;
WbDeviceTag rec[MAX_ROB];
WbFieldRef robs_trans[ROBOTS];	// Robots translation fields
WbFieldRef robs_rotation[ROBOTS];	// Robots rotation fields
float loc[ROBOTS][3];		// Location and heading of everybody in the flock (x,y,heading)
double initial_loc[MAX_ROB][3];
double initial_rot[MAX_ROB][4];
const double *location[MAX_ROB];
const double *rot[MAX_ROB];


static FILE *fp;


int reached = 0;
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

void calc_fitness(double[][DATASIZE],double[],int,int);
void random_pos(int);
void nRandom(int[][SWARMSIZE],int);
void nClosest(int[][SWARMSIZE],int);
void fixedRadius(int[][SWARMSIZE],double);
double robdist(int i, int j);
double rob_orientation(int i);

/* RESET - Get device handles and starting locations */
void reset(void) {
    // Device variables
    char em[] = "emitter0";
    int i;  //counter
    char rob[8] = "epuck0";
    
    emitter_loc = wb_robot_get_device("emitter_loc");
    if (emitter_loc==0) printf("missing emitter_loc\n");
	
    for (i=0;i<MAX_ROB;i++) {
        sprintf(rob,"epuck%d",i);
        robs[i] = wb_supervisor_node_get_from_def(rob);
        robs_trans[i] = wb_supervisor_node_get_field(robs[i],"translation");
        robs_rotation[i] = wb_supervisor_node_get_field(robs[i],"rotation");
        location[i] = wb_supervisor_field_get_sf_vec3f(wb_supervisor_node_get_field(robs[i],"translation"));
        initial_loc[i][0] = location[i][0]; initial_loc[i][1] = location[i][1]; initial_loc[i][2] = location[i][2];
        rot[i] = wb_supervisor_field_get_sf_rotation(wb_supervisor_node_get_field(robs[i],"rotation"));
        initial_rot[i][0] = rot[i][0]; initial_rot[i][1] = rot[i][1]; initial_rot[i][2] = rot[i][2]; initial_rot[i][3] = rot[i][3];
        emitter[i] = wb_robot_get_device(em);
        if (emitter[i]==0) printf("missing emitter %d\n",i);
        rob[3]++;
        em[7]++;
        printf("%s",em);
    }
    if (NOISY == 0){
        fp = fopen("PSO_evaluation.txt","w");
        fprintf(fp, "Perfomance evaluation of the best controllers found by standard PSO for each run: \n");
    } else {
        fp = fopen("PSO_noise_resistant_evaluation.txt","w");
        fprintf(fp, "Perfomance evaluation of the best controllers found by noise-resistant PSO for each run: \n");
    }


}

void update_migration(){
	
	double center[2] = {0.,0.}; // flock center
	 

	for (int i=0;i<ROBOTS;i++) {
		// Get data
		center[0] += wb_supervisor_field_get_sf_vec3f(robs_trans[i])[0]/ROBOTS; // X
		center[1] += wb_supervisor_field_get_sf_vec3f(robs_trans[i])[1]/ROBOTS; // Y
	}
	double distance = sqrt(pow(center[0]-migrx,2) + pow(center[1]-migry,2));
	if(distance < .3){
		//printf("Reached destination %d\n",curr_dest);
		curr_dest++;
	}
	if(curr_dest >= num_destination){
		printf("All destinations are reached, stopping simulation\n"); 
		//wb_supervisor_simulation_set_mode(WB_SUPERVISOR_SIMULATION_MODE_PAUSE);
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

/* MAIN - Distribute and test conctrollers */
int main() {
    double *weights;                         // Optimized result
    int i,j,k;				     // Counter variables

    /* Initialisation */
    wb_robot_init();
    printf("Particle Swarm Optimization Super Controller\n");
    reset();

    wb_robot_step(256);

    double fit, w[ROBOTS][DATASIZE], f[ROBOTS];

    
    // Do N_RUNS runs and send the best controller found to the robot
    for (j=0;j<N_RUNS;j++) {
        printf("Start %d PSO run\n",j+1);
        // Get result of optimization
        weights = pso(SWARMSIZE,NB,LWEIGHT,NBWEIGHT,VMAX,MININIT,MAXINIT,ITS,DATASIZE,1);

        // Set robot weights to optimization results
        fit = 0.0;
        for (i=0;i<1;i++) {
            for (k=0;k<DATASIZE;k++)
              w[i][k] = weights[k];
        }

        // Run FINALRUN tests and calculate average
        printf("Running final runs\n");
        for (i=0;i<FINALRUNS;i++) {
            calc_fitness(w,f,FIT_ITS,1);
            for (k=0;k<ROBOTS && i+k<FINALRUNS;k++) {
                //fitvals[i+k] = f[k];
                fit += f[k];
            }
        }

        fit /= FINALRUNS;  // average over the FINALRUNS runs
        
        printf("current best weights %f%f%f%f\n",w[0][0],w[0][1],w[0][2],w[0][3]);
        printf("Average Performance of %d PSO run: %.3f\n",j+1,fit);
        fprintf(fp, "Average Performance of %d run: %f\n", j+1, fit);

    }
    // Close the log file
	if(fp != NULL)
		fclose(fp);
    /* Wait forever */
    while (1){
        calc_fitness(w,f,FIT_ITS,ROBOTS);
    }

    return 0;
}


void send_poses(void) {

  	char buffer[255];	// Buffer for sending data
	int i;
         
	for (i=0;i<ROBOTS;i++) {
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

	
	for(int i=0; i<ROBOTS; i++) {
		complexe_O[0] += cosf(fabs(loc[i][2]));
		complexe_O[1] += sinf(fabs(loc[i][2]));
	}
	o = sqrt(powf(complexe_O[0],2) + pow(complexe_O[1],2))/ROBOTS;
	


	// To Do : Compute distance performance 
	double d = 0;
	
	for(i=0; i<ROBOTS; i++){ 
        	avg[0] += loc[i][0]/ROBOTS;
        	avg[1] += loc[i][1]/ROBOTS;}

	 
	for(i=0; i<ROBOTS; i++){ 
        	d += fabs(sqrt(pow(loc[i][0]-avg[0],2)+pow(loc[i][1]-avg[1],2))-RULE1_THRESHOLD);}
        d /= ROBOTS;
        d += 1;
        d = 1/d;
        


	// To Do : Compute velocity performance (if migratory urge enabled)
	double v = 0.;
	
	if(MIGRATORY_URGE){

		avg_speed[0] = 1000*(avg[0] - old_avg[0])/TIME_STEP; 
		avg_speed[1] = 1000*(avg[1] - old_avg[1])/TIME_STEP;
		
		vector_migr[0] = migrx - avg[0];
		vector_migr[1] = migry - avg[1];
              
		migr_norm = sqrt(pow(vector_migr[0],2) + pow(vector_migr[1],2));
			
		proj = (vector_migr[0]*avg_speed[0] + vector_migr[1]*avg_speed[1])/migr_norm;
		v = fmax(proj,0);
		
		v = v/(MAX_SPEED_WEB* WHEEL_RADIUS);
		
	}else 
	
		v = 1.;


	return d*v*o;
}

void re_pos(int rob_id) {
    
    initial_rot[rob_id][3] += rnd();
    wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(robs[rob_id],"translation"), initial_loc[rob_id]);
    wb_supervisor_field_set_sf_rotation(wb_supervisor_node_get_field(robs[rob_id],"rotation"), initial_rot[rob_id]);   
    
}

void calc_fitness(double weights[ROBOTS][DATASIZE], double fit[ROBOTS], int its, int numRobs) {
    double buffer[255];
    int i,j;
    double flock_fit = 0.0;
    char label[255];
    double avg = 0;
    int t = its;


   /* Send data to robots */
    for (i=0;i<numRobs;i++) {
       re_pos(i);
       curr_dest = 0;
       for (j=0;j<DATASIZE;j++) {
	buffer[j] = weights[0][j];
       }
       //printf("send %f\n", buffer[0]);
       
       wb_emitter_send(emitter[i],(void *)buffer,(DATASIZE)*sizeof(double));
    }

    wb_supervisor_simulation_reset_physics();
    

    while (t>0){
      
      update_migration();
			
      send_poses();
      if (t<its/2){	
      flock_fit = compute_flocking_fitness();
      avg += flock_fit;
      }
      
     
      
      t--;
      
      wb_robot_step(32);
      }
    
    avg /= its;
    //reached = 0;
    //calculate fitness here
    for (i=0;i<numRobs;i++) {
      fit[i] = avg;}

    // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  sprintf(label,"Last fitness: %.3f\n",fit[0]);
  wb_supervisor_set_label(2,label,0.01,0.95,0.05,0xffffff,0,FONT);
}

/* Optimization fitness function , used in pso.c */
/************************************************************************************/
/* Use the NEIHBORHOOD definition at the top of this file to                        */
/* change the neighborhood type for the PSO. The possible values are:               */
/* STANDARD    : Local neighborhood with 2*NB (defined above) nearest neighbors     */
/*               NEIGHBORHOOD is set to STANDARD by default                         */
/* RAND_NB     : 2*NB random neighbors                                              */
/* NCLOSE_NB   : 2*NB closest neighbors                                             */
/* FIXEDRAD_NB : All robots within a defined radius are neighbors                   */
/************************************************************************************/
void fitness(double weights[ROBOTS][DATASIZE], double fit[ROBOTS], int neighbors[SWARMSIZE][SWARMSIZE]) {
    calc_fitness(weights,fit,FIT_ITS,ROBOTS);
#if NEIGHBORHOOD == RAND_NB
    nRandom(neighbors,2*NB);
#endif
#if NEIGHBORHOOD == NCLOSE_NB
    nClosest(neighbors,2*NB);
#endif
#if NEIGHBORHOOD == FIXEDRAD_NB
    fixedRadius(neighbors,RADIUS);
#endif
}

/* Get distance between robots */
double robdist(int i, int j) {
    return sqrt(pow(loc[i][0]-loc[j][0],2) + pow(loc[i][1]-loc[j][1],2));
}

/* Choose n random neighbors */
void nRandom(int neighbors[SWARMSIZE][SWARMSIZE], int numNB) {

    int i,j;

    /* Get neighbors for each robot */
    for (i = 0; i < ROBOTS; i++) {

        /* Clear old neighbors */
        for (j = 0; j < ROBOTS; j++)
        	neighbors[i][j] = 0;

        /* Set new neighbors randomly */
        for (j = 0; j < numNB; j++)
        	neighbors[i][(int)(SWARMSIZE*rnd())] = 1;

    }
}

/* Choose the n closest robots */
void nClosest(int neighbors[SWARMSIZE][SWARMSIZE], int numNB) {

    int r[numNB];
    int tempRob;
    double dist[numNB];
    double tempDist;
    int i,j,k;

    /* Get neighbors for each robot */
    for (i = 0; i < ROBOTS; i++) {

        /* Clear neighbors */
        for (j = 0; j < numNB; j++)
        dist[j] = ARENA_SIZE;

        /* Find closest robots */
        for (j = 0; j < ROBOTS; j++) {

            /* Don't use self */
            if (i == j) continue;

            /* Check if smaller distance */
            if (dist[numNB-1] > robdist(i,j)) {
                dist[numNB-1] = robdist(i,j);
                r[numNB-1] = j;

                /* Move new distance to proper place */
                for (k = numNB-1; k > 0 && dist[k-1] > dist[k]; k--) {

                    tempDist = dist[k];
                    dist[k] = dist[k-1];
                    dist[k-1] = tempDist;
                    tempRob = r[k];
                    r[k] = r[k-1];
                    r[k-1] = tempRob;

                }
            }

        }

        /* Update neighbor table */
        for (j = 0; j < ROBOTS; j++)
        neighbors[i][j] = 0;
        for (j = 0; j < numNB; j++)
        neighbors[i][r[j]] = 1;

    }

}

/* Choose all robots within some range */
void fixedRadius(int neighbors[SWARMSIZE][SWARMSIZE], double radius) {

    int i,j;

    /* Get neighbors for each robot */
    for (i = 0; i < ROBOTS; i++) {

        /* Find robots within range */
        for (j = 0; j < ROBOTS; j++) {

            if (i == j) continue;

            if (robdist(i,j) < radius) neighbors[i][j] = 1;
            else neighbors[i][j] = 0;

        }

    }

}

void step_rob() {
    wb_robot_step(64);
}

