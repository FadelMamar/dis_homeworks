#include <stdio.h>
#include <math.h>
#include "pso.h"
#include <stdlib.h>
#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/supervisor.h>

#define ROBOTS 2 						// Was previously 1
#define MAX_ROB 2
#define ROB_RAD 0.035
#define ARENA_SIZE .15
#define MAX_DIST 3.0
#define SENSOR_RANGE 0.3

#define NB_SENSOR 8                     // Number of proximity sensors

/* PSO definitions */
#define NB 1                            // Number of neighbors on each side
#define LWEIGHT 2.0                     // Weight of attraction to personal best
#define NBWEIGHT 2.0                    // Weight of attraction to neighborhood best
#define VMAX 40.0                       // Maximum velocity particle can attain
#define MININIT -20.0                   // Lower bound on initialization value
#define MAXINIT 20.0                    // Upper bound on initialization value
#define ITS 10                          // Number of iterations to run

/* Neighborhood types */
#define STANDARD    -1
#define RAND_NB      0
#define NCLOSE_NB    1
#define FIXEDRAD_NB  2

/* Fitness definitions */
#define FIT_ITS 240                     // Number of fitness steps to run during evolution

#define FINALRUNS 10
#define NEIGHBORHOOD STANDARD
#define RADIUS 0.8
#define N_RUNS 10        // Change this to 10 for question 2.10

#define PI 3.1415926535897932384626433832795
enum {POS_X=0,POS_Y,POS_Z};

static WbNodeRef robs[MAX_ROB];
WbDeviceTag emitter[MAX_ROB];
WbDeviceTag phy_emitter;
WbDeviceTag rec[MAX_ROB];
const double *loc[MAX_ROB];
const double *rot[MAX_ROB];
double new_loc[MAX_ROB][3];
double new_rot[MAX_ROB][4];

static FILE *fp;


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
    char rob[] = "rob0";
    char em[] = "emitter0";
    char receive[] = "receiver0";
    int i;  //counter
    for (i=0;i<MAX_ROB;i++) {
        robs[i] = wb_supervisor_node_get_from_def(rob);
        //printf("robot %s\n", rob);
        loc[i] = wb_supervisor_field_get_sf_vec3f(wb_supervisor_node_get_field(robs[i],"translation"));
        new_loc[i][0] = loc[i][0]; new_loc[i][1] = loc[i][1]; new_loc[i][2] = loc[i][2];
        rot[i] = wb_supervisor_field_get_sf_rotation(wb_supervisor_node_get_field(robs[i],"rotation"));
        new_rot[i][0] = rot[i][0]; new_rot[i][1] = rot[i][1]; new_rot[i][2] = rot[i][2]; new_rot[i][3] = rot[i][3];
        emitter[i] = wb_robot_get_device(em);
        if (emitter[i]==0) printf("missing emitter %d\n",i);
        rec[i] = wb_robot_get_device(receive);
        rob[3]++;
        em[7]++;
        receive[8]++;
    }
    if (NOISY == 0){
        fp = fopen("PSO_evaluation.txt","w");
        fprintf(fp, "Perfomance evaluation of the best controllers found by standard PSO for each run: \n");
    } else {
        fp = fopen("PSO_noise_resistant_evaluation.txt","w");
        fprintf(fp, "Perfomance evaluation of the best controllers found by noise-resistant PSO for each run: \n");
    }


}

/* MAIN - Distribute and test conctrollers */
int main() {
    double *weights;                         // Optimized result
    int i,j,k;				     // Counter variables

    /* Initialisation */
    wb_robot_init();
    printf("Particle Swarm Optimization Super Controller\n");
    reset();
    for (i=0;i<MAX_ROB;i++)
    wb_receiver_enable(rec[i],32);

    wb_robot_step(256);

    double fit, w[ROBOTS][DATASIZE], f[ROBOTS];


    // Do N_RUNS runs and send the best controller found to the robot
    for (j=0;j<N_RUNS;j++) {
        printf("Start %d PSO run\n",j+1);
        // Get result of optimization
        weights = pso(SWARMSIZE,NB,LWEIGHT,NBWEIGHT,VMAX,MININIT,MAXINIT,ITS,DATASIZE,ROBOTS);

        // Set robot weights to optimization results
        fit = 0.0;
        for (i=0;i<ROBOTS;i++) {
            for (k=0;k<DATASIZE;k++)
              w[i][k] = weights[k];
        }

        // Run FINALRUN tests and calculate average
        printf("Running final runs\n");
        for (i=0;i<FINALRUNS;i+=ROBOTS) {
            calc_fitness(w,f,FIT_ITS,ROBOTS);
            for (k=0;k<ROBOTS && i+k<FINALRUNS;k++) {
                //fitvals[i+k] = f[k];
                fit += f[k];
            }
        }

        fit /= FINALRUNS;  // average over the FINALRUNS runs
        
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

// Makes sure no robots are overlapping
char valid_locs(int rob_id) {
    int i;

    for (i = 0; i < MAX_ROB; i++) {
        if (rob_id == i) continue;
        if (pow(new_loc[i][0]-new_loc[rob_id][0],2) +
                pow(new_loc[i][1]-new_loc[rob_id][1],2) < (2*ROB_RAD+0.02)*(2*ROB_RAD+0.02))
        return 0;
    }
    return 1;
}

// Randomly position specified robot
void random_pos(int rob_id) {
    //printf("Setting random position for %d\n",rob_id);
    new_rot[rob_id][0] = 0.0;
    new_rot[rob_id][1] = 0.0;
    new_rot[rob_id][2] = 1.0;
    new_rot[rob_id][3] = 2.0*PI*rnd();

    do {
        new_loc[rob_id][0] = (ARENA_SIZE-ROB_RAD)*rnd() - (ARENA_SIZE-ROB_RAD)/2.0;
        new_loc[rob_id][1] = (ARENA_SIZE-ROB_RAD)*rnd() - (ARENA_SIZE-ROB_RAD)/2.0; 
        new_loc[rob_id][2] = 0.001;
        //printf("%d at %.2f, %.2f\n", rob_id, new_loc[rob_id][0], new_loc[rob_id][2]);
    } while (!valid_locs(rob_id));

    wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(robs[rob_id],"translation"), new_loc[rob_id]);
    wb_supervisor_field_set_sf_rotation(wb_supervisor_node_get_field(robs[rob_id],"rotation"), new_rot[rob_id]);
}

void calc_fitness(double weights[ROBOTS][DATASIZE], double fit[ROBOTS], int its, int numRobs) {
    double buffer[255];
    double *rbuffer;
    int i,j;
    char label[255];

    // Distribute fitness functions among robots
    // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    /* Send data to robots */
    /*   public-heterogeneous <> Base model
    random_pos(0);
    for (j=0;j<DATASIZE/2;j++) {
        buffer[j] = weights[0][j];
    }
    buffer[DATASIZE/2] = its;
    wb_emitter_send(emitter[0],(void *)buffer,(DATASIZE/2+1)*sizeof(double));
    random_pos(1);
    for (j=0;j<DATASIZE/2;j++) {
        buffer[j] = weights[0][j+DATASIZE/2];
    }
    buffer[DATASIZE/2] = its;
    wb_emitter_send(emitter[1],(void *)buffer,(DATASIZE/2+1)*sizeof(double));
    */

   /* Send data to robots */
    for (i=0;i<numRobs;i++) {
       random_pos(i);
       for (j=0;j<DATASIZE;j++) {
	buffer[j] = weights[i][j];
        }
       buffer[DATASIZE] = its;
       wb_emitter_send(emitter[i],(void *)buffer,(DATASIZE+1)*sizeof(double));
    }
	
    // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    wb_supervisor_simulation_reset_physics();

    /* Get the positions while waiting for response from robots */
    while (wb_receiver_get_queue_length(rec[0]) == 0){
        wb_robot_step(64);
        loc[0] = wb_supervisor_field_get_sf_vec3f(wb_supervisor_node_get_field(robs[0],"translation"));
        loc[1] = wb_supervisor_field_get_sf_vec3f(wb_supervisor_node_get_field(robs[1],"translation"));
    }

    /* Receive fitness messages from robots */
    for (i=0;i<MAX_ROB;i++) {
        rbuffer = (double *)wb_receiver_get_data(rec[i]);
        wb_receiver_next_packet(rec[i]);
    }
    // TODO : Q 2.8 complete the function below to calculate the fitness 
    // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    double maze_endpoint[3] = {-1.39679, 2.19582,0.001};
    fit[0] = 1/(1+sqrt(pow(loc[0][0]-maze_endpoint[0],2)+pow(loc[0][1]-maze_endpoint[1],2)));
    fit[1] = 1/(1+sqrt(pow(loc[1][0]-maze_endpoint[0],2)+pow(loc[1][1]-maze_endpoint[1],2)));
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

