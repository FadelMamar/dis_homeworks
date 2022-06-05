/*****************************************************************************/
/* File:         reynolds.c                                                  */
/* Version:      1.0                                                         */
/* Date:         10-Oct-14                                                   */
/* Description:  Reynolds flocking control 									 */
/*                                                                           */
/* Author:        10-Oct-14 by Ali marjovi			           				 */
/* Last revision: 16-March-22 by Lucas Waelti					 			 */
/*****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/emitter.h>
#include <webots/receiver.h>

#include "../flock_param.h"

#define TIME_STEP			32	  	// [ms] Length of time step

#define NB_SENSORS	  		8	  	// Number of distance sensors
#define MIN_SENS          	350     // Minimum sensibility value
#define MAX_SENS          	4096    // Maximum sensibility value

#define MAX_SPEED_WEB      	6.28    // Maximum speed webots

#define AXLE_LENGTH 		0.052	// Distance between wheels of robot (meters)
#define SPEED_UNIT_RADS		0.00628	// Conversion factor from speed unit to radian per second
#define WHEEL_RADIUS		0.0205	// Wheel radius (meters)
#define DELTA_T	  TIME_STEP/1000.	// Timestep (seconds)

#define VERBOSE 			0

#define ABS(x) ((x>=0)?(x):-(x))

#define NB_SENSOR 8 // Number of proximity sensorsw

WbDeviceTag left_motor; 	// Handle for left wheel of the robot
WbDeviceTag right_motor; 	// Handle for the right wheel of the robot
WbDeviceTag ds[NB_SENSORS];	// Handle for the infrared distance sensors
WbDeviceTag receiver;		// Handle for the receiver node
WbDeviceTag emitter;		// Handle for the emitter node
WbDeviceTag receiver_epuck;	


int robot_id_u, robot_id;		// Unique and normalized (between 0 and FLOCK_SIZE-1), robot ID
float loc[FLOCK_SIZE][3];		// X, Y, Theta of all robots
float prev_loc[FLOCK_SIZE][3];	// Previous X, Y, Theta values
float speed[FLOCK_SIZE][2];		// Speeds calculated with Reynold's rules
int initialized[FLOCK_SIZE];	// != 0 if initial positions have been received
float migr[2];	                // Migration vector


WbDeviceTag rec;

double default_reynolds_weights[4] = {0.06, 0.002, 0.1, 0.05}; 
/*
 * Reset the robot's devices and get its ID
 *
 */
/*
static void reset() {
	
	wb_robot_init();

	receiver = wb_robot_get_device("receiver");
	emitter = wb_robot_get_device("emitter");
	rec = wb_robot_get_device("receiver_epuck");
	
	//get motors
	left_motor = wb_robot_get_device("left wheel motor");
	right_motor = wb_robot_get_device("right wheel motor");
	wb_motor_set_position(left_motor, INFINITY);
	wb_motor_set_position(right_motor, INFINITY);

	int i;
	char s[4]="ps0";
	for(i=0; i<NB_SENSORS;i++) {
		ds[i]=wb_robot_get_device(s);	// the device name is specified in the world file
      		s[2]++;				// increases the device number
	}
	char* robot_name; 
	robot_name=(char*) wb_robot_get_name(); 

	for(i=0;i<NB_SENSORS;i++) {
		wb_distance_sensor_enable(ds[i],64); // limit update frequency
	}
	wb_receiver_enable(receiver,64); // limit update frequency

	//Reading the robot's name. Pay attention to name specification when adding robots to the simulation!
	sscanf(robot_name,"epuck%d",&robot_id_u); // read robot id from the robot's name
	robot_id = robot_id_u%FLOCK_SIZE;	  // normalize between 0 and FLOCK_SIZE-1

	// If the robot's index is beyond the flock size, turn leds on and exit 
	if(robot_id_u >= FLOCK_SIZE){
		printf("Robot %d not part of the flock, exited\n",robot_id_u);
		char led_name[5] = "led0";
		for(i=0;i<NB_SENSORS;i++){
			WbDeviceTag led = wb_robot_get_device(led_name);
			wb_led_set(led,1);
			led_name[3]++;
		}
		wb_motor_set_velocity(left_motor,  0);
		wb_motor_set_velocity(right_motor, 0);
		wb_robot_step(TIME_STEP);
		exit(EXIT_SUCCESS);	
	}

	for(i=0; i<FLOCK_SIZE; i++) {
		initialized[i] = 0; 		  // Set initialization to 0 (= not yet initialized)
	}

	printf("Reset: robot %d\n",robot_id_u);
}
*/
void reset(void) {
    char text[4];
    int i;
    text[1]='s';
    text[3]='\0';
    for (i=0;i<NB_SENSOR;i++) {
        text[0]='p';
        text[2]='0'+i;
        ds[i] = wb_robot_get_device(text); // distance sensors
    }

    rec = wb_robot_get_device("receiver_epuck");
    
    left_motor = wb_robot_get_device("left wheel motor");
    right_motor = wb_robot_get_device("right wheel motor");
    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_position(right_motor, INFINITY);
}
/*
 * Keep given float number within interval {-limit, limit}
 */
void limitf(float *number, int limit) {

	if (*number > limit)
		*number = (float)limit;
	if (*number < -limit)
		*number = (float)-limit;
}

/*
 * Keep given int number within interval {-limit, limit}
 */
void limit(int *number, int limit) {

	if (*number > limit)
		*number = limit;
	if (*number < -limit)
		*number = -limit;
}

/*
 * recenter_angle to [-PI, PI]
 */
void recenter_angle(double* angle){
	if (*angle > M_PI){
		*angle -= 2*M_PI;
	}
	if (*angle < -M_PI){
		*angle += 2*M_PI;
	}
}

/**
 * Get the position and velocity of the robots 
 */
void get_robot_positions()
{
	int rob_nb;						// Robot number
	float rob_x, rob_y, rob_theta;  // Robot position and orientation
	float mx, my;					// Migratory direction 
	char *inbuffer;					// Buffer for the receiver node

	/* Get own position */
	int count = 0;
	while (wb_receiver_get_queue_length(receiver) > 0 && count < FLOCK_SIZE) 
	{
		inbuffer = (char*) wb_receiver_get_data(receiver);
		//sscanf(inbuffer,"%d#%f#%f#%f",&rob_nb,&rob_x,&rob_y,&rob_theta);
		sscanf(inbuffer,"%d#%f#%f#%f##%f#%f",&rob_nb,&rob_x,&rob_y,&rob_theta,&mx,&my);
		
		if ((int) rob_nb/FLOCK_SIZE == (int) robot_id_u/FLOCK_SIZE) {
			//printf("%d received info about %d (mig: %.2lf,%.2lf)\n",robot_id_u,rob_nb,migr[0],migr[1]);
			migr[0] = mx;
			migr[1] = my;
			rob_nb %= FLOCK_SIZE;
			if (initialized[rob_nb] == 0) {
				// Get initial positions
				loc[rob_nb][0] = rob_x; //x-position
				loc[rob_nb][1] = rob_y; //y-position
				loc[rob_nb][2] = rob_theta; //theta
				prev_loc[rob_nb][0] = loc[rob_nb][0];
				prev_loc[rob_nb][1] = loc[rob_nb][1];
				initialized[rob_nb] = 1;
			} else {
				// Get position update
				prev_loc[rob_nb][0] = loc[rob_nb][0];
				prev_loc[rob_nb][1] = loc[rob_nb][1];
				loc[rob_nb][0] = rob_x; //x-position
				loc[rob_nb][1] = rob_y; //y-position
				loc[rob_nb][2] = rob_theta; //theta
			}
			speed[rob_nb][0] = (1/DELTA_T)*(loc[rob_nb][0]-prev_loc[rob_nb][0]);
			speed[rob_nb][1] = (1/DELTA_T)*(loc[rob_nb][1]-prev_loc[rob_nb][1]);
			count++;
		}
		wb_receiver_next_packet(receiver);
	}

	// Compute self position & speed
	prev_loc[robot_id][0] = loc[robot_id][0];
	prev_loc[robot_id][1] = loc[robot_id][1];
}

/**
 * @brief 	Update speed (world frame) according to Reynold's rules
 */
void reynolds_rules(double reynolds_weights[4]) {

	int i, j, k;					// Loop counters
	float avg_loc[2] 	= {0,0};	// Flock average positions
	float avg_speed[2] 	= {0,0};	// Flock average speeds
	float cohesion[2] 	= {0,0};
	float dispersion[2] = {0,0};
	float alignment[2] 	= {0,0};
	float dist;
	float n_robots;

	double rule1_weight = reynolds_weights[0];
	double rule2_weight = reynolds_weights[1];
	double rule3_weight = reynolds_weights[2];
	double migration_weight = reynolds_weights[3];
	
	// Compute the average speed and loc of the neighbourhood/flock 
	if (NEIGHBOURHOOD) {

		n_robots =1;

		for(i=0; i<FLOCK_SIZE; i++) {
			if (i == robot_id) {	
          		// don't consider yourself for the average 
				continue;
			}
			dist = sqrt(pow(loc[i][0]-loc[robot_id][0],2.0)+pow(loc[i][1]-loc[robot_id][1],2.0));
			if (dist < NEIGH_THRESHOLD){
				for (j=0;j<2;j++) {      		         
					avg_speed[j] += speed[i][j];
					avg_loc[j] += loc[i][j];
				}
				n_robots= n_robots+1;
			}	
		}

		for (j=0;j<2;j++) {
			if (n_robots >1){
				avg_speed[j] /= (n_robots-1);
				avg_loc[j] /= (n_robots-1) ;
			}
			else{
				avg_speed[j] = speed[robot_id][j];
				avg_loc[j] = loc[robot_id][j];
			}
		}

	} else {
		/* Compute averages over the whole flock */
		for(i=0; i<FLOCK_SIZE; i++) {
			if (i == robot_id) {	
          		// don't consider yourself for the average 
				continue;
			}
			for (j=0;j<2;j++) {
				avg_speed[j] += speed[i][j];
				avg_loc[j] += loc[i][j];
			}
		}

		for (j=0;j<2;j++) {
			avg_speed[j] /= FLOCK_SIZE-1;
			avg_loc[j] /= FLOCK_SIZE-1;
		}
	}
	
	/* Reynold's rules */
	
	/* Rule 1 - Aggregation/Cohesion: move towards the center of mass */
	for (j=0;j<2;j++) {
		// If center of mass is too far
		if (sqrt(pow(loc[robot_id][0]-avg_loc[0],2)+pow(loc[robot_id][1]-avg_loc[1],2)) > RULE1_THRESHOLD) {
         		cohesion[j] = avg_loc[j] - loc[robot_id][j];   // Relative distance to the center of the swarm
         	}
         }

	/* Rule 2 - Dispersion/Separation: keep far enough from flockmates */
    for (k=0;k<FLOCK_SIZE;k++) {
		if (k != robot_id) {        // Loop on flockmates only
			// If neighbor k is too close (Euclidean distance)
			if (pow(loc[robot_id][0]-loc[k][0],2)+pow(loc[robot_id][1]-loc[k][1],2) < RULE2_THRESHOLD) {
				for (j=0;j<2;j++) {
					dispersion[j] += 1/(loc[robot_id][j] -loc[k][j]);	// Relative distance to k
				}
			}
		}
	}

	/* Rule 3 - Alignment: match the speeds of flockmates */
	alignment[0] = 0;
	alignment[1] = 0;
	for (j=0;j<2;j++) {
		// Compute alignment 
		alignment[j] = avg_speed[j] - speed[robot_id][j];
	}

	if (VERBOSE) {
        printf("id = %d, coh_x:%f coh_y:%f, dis_x:%f dis_y:%f, align_x:%f align_y:%f\n", 
			robot_id, cohesion[0], cohesion[1], dispersion[0], dispersion[1], alignment[0], alignment[1]);
	}
	
	for (j=0;j<2;j++) {
		speed[robot_id][j]  = cohesion[j]   * rule1_weight;
		speed[robot_id][j] += dispersion[j] * rule2_weight;
		speed[robot_id][j] += alignment[j]  * rule3_weight;
	}
	
	// move the robot according to some migration rule
	if(MIGRATORY_URGE){
		/* Implement migratory urge */
		double a = atan2(migr[1]-loc[robot_id][1],migr[0]-loc[robot_id][0]);
		speed[robot_id][0] += migration_weight*cos(a);
		speed[robot_id][1] += migration_weight*sin(a); 
	}
}

/**
 * @brief	Computes wheel speed given a certain X,Y speed
 * @param	left	left wheel speed command in rad/s
 * @param	right 	right wheel speed command in rad/s
 */
void compute_wheel_speeds(double *left, double *right) {

	// Compute wanted position from Reynold's speed and current location (transform speed to robot frame)
	float x =  speed[robot_id][0]*cosf(loc[robot_id][2]) + speed[robot_id][1]*sinf(loc[robot_id][2]); // x in robot coordinates
	float y = -speed[robot_id][0]*sinf(loc[robot_id][2]) + speed[robot_id][1]*cosf(loc[robot_id][2]); // y in robot coordinates

	float Ku = 0.2;  // Forward control coefficient
	float Kw = 0.6;  // Rotational control coefficient
	float range = sqrtf(x*x + y*y);	  // Distance to the wanted position
	float bearing = atan2(y, x);	  // Orientation of the wanted position
	
	// Compute forward control
	float u = Ku*range*cosf(bearing);
	// Compute rotational control
	float w = Kw*bearing;
	
	// Compute percentage of max speed to apply 
	float msl = (u - AXLE_LENGTH*w/2.0) / WHEEL_RADIUS;
	float msr = (u + AXLE_LENGTH*w/2.0) / WHEEL_RADIUS;
	limitf(&msl,1);
	limitf(&msr,1);

	// Convert to wheel speeds 
	*left  = msl*MAX_SPEED_WEB;
	*right = msr*MAX_SPEED_WEB;

	if(isnan(*left)) *left = 0.;
	if(isnan(*right)) *right = 0.;
}

/**
 * @brief 	Compute left and right speed for obstacle avoidance 
 * @param	left	left wheel speed command in rad/s
 * @param	right 	right wheel speed command in rad/s
 * @param	avoid	whether avoidance is required 
 */
void compute_braitenberg_speed(double *left, double *right, bool *avoid){

	// coefficients
	static double l_weight[NB_SENSORS] = {-4,-2, 0, 2, 2, 3,  1, -2};
	static double r_weight[NB_SENSORS] = {-2, 1, 3, 2, 2, 0, -2, -4};

	// bias 
	double left_speed = .5, right_speed = .5;

	// gain
	double K = 1.;
	
	// whether braitenberg should be applied 
	*avoid = false;

	// define speed with respect to the sensory feedback
	for (int i = 0; i < NB_SENSORS; i++){
		// ds_values: 0 (far) to 4095 (0 distance (in theory))
		double sens = wb_distance_sensor_get_value(ds[i]);
		if(sens > 80) *avoid = true;
		left_speed  += K*(l_weight[i]) * sens/MAX_SENS; 
		right_speed += K*(r_weight[i]) * sens/MAX_SENS;
	}

	*left  = left_speed  * MAX_SPEED_WEB;
	*right = right_speed * MAX_SPEED_WEB;
}

/*
 * Main function
 */
int main(){ 	

	float wsl, wsr;			// Wheel speed left and right 
	double ws_reyn[2]  = {0.,0.}; 	// Wheel speed computed with reynolds 
	double ws_brait[2] = {0.,0.}; 	// Wheel speed computed with braitenberg 
	bool avoid; 				// Whether avoidance is required 
       float n1,n2,n3,n4;   // Robot position and orientation
	char *inbuffer;					// Buffer for the receiver node
        
       wb_robot_init();
 	reset();
 	
 	for(int i=0;i<NB_SENSOR;i++) {
          wb_distance_sensor_enable(ds[i],64);}
       
       wb_receiver_enable(rec,32);
 							// Resetting the robot
 	while (wb_receiver_get_queue_length(rec) == 0){
         	wb_robot_step(64);}
         	
       inbuffer = (char*)wb_receiver_get_data(rec);
       printf("%s", inbuffer);
       sscanf(inbuffer,"%f#%f#%f#%f",n1,&n2,&n3,&n4);

       wb_receiver_next_packet(rec);
      
       	printf("weights_received: %f\n", n1);
	printf("weights_received: %f\n", n2);
	printf("weights_received: %f\n", n3);
	printf("weights_received: %f\n", n4);
	printf("------\n"); 
       	//wb_receiver_next_packet(rec);
	/*
	// Forever
	for(;;){

		/* Get information
		get_robot_positions();
              
		// Reynold's rules with all previous info (updates the speed[robot_id][] table)
		reynolds_rules(default_reynolds_weights);

		// Compute wheels speed from Reynold's speed
		compute_wheel_speeds(&ws_reyn[0], &ws_reyn[1]);

		// Add Braitenberg
		compute_braitenberg_speed(&ws_brait[0], &ws_brait[1], &avoid);

		// Compute wheel velocities 
		if(avoid){
			// apply avoidance 
			wsl = ws_brait[0];
			wsr = ws_brait[1];
		}
		else{
			// apply reynolds 
			wsl = ws_reyn[0];
			wsr = ws_reyn[1];
		}
		
		// Set speeds 
		limitf(&wsl, MAX_SPEED_WEB);
		limitf(&wsr, MAX_SPEED_WEB);
		wb_motor_set_velocity(left_motor, wsl); 
		wb_motor_set_velocity(right_motor, wsr);

		// Send current position to neighbors, uncomment for I15, don't forget to add the declaration of "outbuffer" at the begining of this function.
		/*Implement your code here
		if (INTER_VEHICLE_COM) {
			sprintf(outbuffer,"%1d#%f#%f#%f",robot_id,loc[robot_id][0],loc[robot_id][1], loc[robot_id][2]);
			wb_emitter_send(emitter,outbuffer,strlen(outbuffer));
		}

		// Continue one step
		wb_robot_step(TIME_STEP);
	}*/
} 

