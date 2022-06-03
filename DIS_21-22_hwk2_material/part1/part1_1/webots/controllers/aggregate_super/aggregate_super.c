/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * file:         aggregate_super.c
 * description:  supervisor controller for seed aggregation
 *
 * $Author$: Kagan Erunsal
 * $Revision$:
 * $Date$: 2022-05-20 
 * 
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// Includes 
#include <time.h>
#include <webots/robot.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <webots/supervisor.h>
#include <stdio.h>
#include <math.h>
#include <webots/emitter.h>
#include <webots/receiver.h>

//Defines
#define TIMESTEP                 32        // Time-step of controller
#define NUM_ROBOTS               5         // Change this for number of robots
#define NUM_SEEDS                5        // Change this for number of seeds
#define ARENASIZE                0.9       // (0.1 security margin)
#define MIN_ROBOT_DIST_SQUARE    0.15*0.15 // Robot-robot safety distance
#define MIN_SEED_DIST_SQUARE     0.1*0.1   // Robot-seed safety
#define FREE                     0         // Physical height of free seeds
#define GRIPPED                  0.2       // Physical height of gripped seeds
#define RAND                     ((float) rand()/RAND_MAX)
#define MAX_SIM_TIME_SEC     	10800     // Sim time in seconds (3h)
#define ROBOT_RAD                0.035     // Robot radius
#define SEED_RAD                 0.035     // Seed radius
#define RELEASE_BIAS             0.5       // Angle bias for releasing seeds
#define TO_STUCK                 20000     // Timeout for respawning robots
#define MIN_STUCK_DIST_SQUARE    0.02*0.02 // Threshold movement for stuck robots
#define NULL_SEED                99        // Null number for seed
#define NULL_ROBOT               99        // Null number for robot
#define ENB_LOG_CON              0         // Enable to see logs on console
#define ENB_LOG_FILE             0         // Enable to see logs on file
#define ENB_STUCK_PREV           0         // Enable the measure for stuck robots
#define THR_ROT                  0.3       // Threshold angle for dangerous rotations

// Global Variables 
WbNodeRef robots_ref[NUM_ROBOTS];   // Webots Node Reference
WbNodeRef seeds_ref[NUM_SEEDS];   // Webots Node Reference
WbFieldRef robots_translation[NUM_ROBOTS];  // Contains translation node for robots
WbFieldRef seeds_translation[NUM_SEEDS];  // Contains translation node for shelters
WbFieldRef robots_orientation[NUM_ROBOTS]; // Contains orientation node for robots
WbDeviceTag emitter, receiver; // Supervisor emitter/receiver

// Seed structure
typedef struct{

  int id; //Seed id, same as def
  int carrying_robot; // Robot id carrying the seed
  const double *pos; // Seed poisition
  int inside_cluster; // Seed is inside the cluster with id
     
} seed;

// Robot structure
typedef struct{
  
  int id; //Robot id, same as def
  const double *pos; //Robot posiiton
  const double *rot; // Robot orientation
  double pos_prev[3]; // Robots previous position for stuck detection
  int state; // Robots state
  int gripped_seed; // id of the seed that robot grips
  int approached_seed; // id of the seed that robot approaches

} robot;

//Cluster structure
typedef struct{

   int Nseed; // Number of seeds in the cluster
   
} cluster;

robot robots[NUM_ROBOTS];           // an array of robots
seed seeds[NUM_SEEDS];              // array of seeds
cluster clusters[NUM_SEEDS];        // array of clusters

double ps_orient[8] = {1.27, 0.77, 0.0, 5.21, 4.21, 3.14159, 2.37, 1.87};
bool all_robots_deployed = false;
bool all_seeds_deployed = false;
bool redeploy = true;
double NC, ACS, SBC; // metrics
FILE *fp; // File for logs
double default_rot[4] ={0,0,1,9};

// Distance Calculations 
double get_dist_square(const double *pos1, const double *pos2){
	return((pos1[0]-pos2[0])*(pos1[0]-pos2[0])+(pos1[1]-pos2[1])*(pos1[1]-pos2[1]));
}

// Send decisions to robots for gripping and releasing
void emit_message_to_robots(int robot_id, int seed) {

  int pkt_send[2];
  
  pkt_send[0] = robot_id; pkt_send[1] = seed;
  wb_emitter_send(emitter,pkt_send,2*sizeof(int));
  
}

// Receive queries from the robots for girpping and releasing
void receive_emit_message_from_to_robots() {

  int queue_length, k, e;
  const int *pkt;
  double Pos[3];

  // Receive messages for GRIP/RELEASE
  queue_length = wb_receiver_get_queue_length(receiver);
  
  for (k = 0; k < queue_length; k++) {
  
       pkt = (int*)wb_receiver_get_data(receiver);
       int r = pkt[0];
       int decision = pkt[1];
       int detected_ps = pkt[2];
       
       if (decision == 1) { //Grip
       
           for(e=0;e<NUM_SEEDS;e++) {
           
             if(seeds[e].carrying_robot == NULL_ROBOT) {
               
               double distance = get_dist_square(seeds[e].pos, robots[r].pos);
                                                                                                                           
               if (distance <  MIN_SEED_DIST_SQUARE) {
               
                       Pos[2] = GRIPPED;                   // use 0 z height
                       Pos[0] = robots[r].pos[0];     // randomize x
                       Pos[1] = robots[r].pos[1];     // randomize y
                       wb_supervisor_field_set_sf_vec3f(seeds_translation[e],Pos);
                       
                       robots[r].gripped_seed = e;
                       seeds[e].carrying_robot = r;
                       
                       emit_message_to_robots(r,e);                                            
                       
                       break;
                }
           
              }
              
           }
                      
        }
            
        if (decision == 0) { //Release
        
          double yaw_diff = ps_orient[detected_ps] + robots[r].rot[3]*robots[r].rot[2] - M_PI/2 + RELEASE_BIAS;
                                  
          Pos[2] = FREE;                                 // use 0 z height
          Pos[0] = robots[r].pos[0] + cos(yaw_diff) * (ROBOT_RAD + SEED_RAD);     // randomize x
          Pos[1] = robots[r].pos[1] + sin(yaw_diff) * (ROBOT_RAD + SEED_RAD);     // randomize y
          
          int current_seed = robots[r].gripped_seed;
          
          wb_supervisor_field_set_sf_vec3f(seeds_translation[current_seed],Pos);
                    
          emit_message_to_robots(r,NULL_SEED);
                   
          seeds[current_seed].carrying_robot = NULL_ROBOT;
          robots[r].gripped_seed = NULL_SEED;
 
        }   
                                   
  wb_receiver_next_packet(receiver); 

  }

}

// Animation for carrying a seed by robots
void update_seed_animation() {

  double Pos[3];
  
  for(int e=0;e<NUM_SEEDS;e++) {
             
    if(seeds[e].carrying_robot != NULL_ROBOT) {
    
       Pos[2] = GRIPPED;
       Pos[0] = robots[seeds[e].carrying_robot].pos[0];
       Pos[1] = robots[seeds[e].carrying_robot].pos[1];
       
      wb_supervisor_field_set_sf_vec3f(seeds_translation[e],Pos);
      
      }
   }
}

// Initialization of robots
void initialize_robots(){
  
  // Give robots a random position and initialize their task list to empty
  int r;
  char rname[8] = "e-puck0";
  
  for(r=0; r<NUM_ROBOTS; r++)
  { 
    robots_ref[r] = wb_supervisor_node_get_from_def(rname);
    robots_translation[r] = wb_supervisor_node_get_field(robots_ref[r],"translation");
    robots_orientation[r] = wb_supervisor_node_get_field(robots_ref[r],"rotation");
    
    if (!robots_ref[r]) {
      printf("Missing node for robot #%d\n", r);
      exit(1);
    }
    
    rname[6]++;
    
    robots[r].gripped_seed = NULL_SEED;
  }

  double distance_square = 0;     // distance between 2 robots squared
  int i,j;
  double initPos[NUM_ROBOTS][3];

  // While not correctly deployed, reset robot position
  while(!all_robots_deployed){
  
      // deploy elements at random positions
      for(i=0; i<NUM_ROBOTS; i++){
        initPos[i][2] = 0;                      // use 0 z height
        initPos[i][0] = (RAND*(2*ARENASIZE) - ARENASIZE)/2;      // randomize x
        initPos[i][1] = (RAND*(2*ARENASIZE) - ARENASIZE)/2;      // randomize y
      } 
      all_robots_deployed = true; 
      
      // check if elements do not overlap -> re-deploy if overlaps detected
      for(i=0; i<NUM_ROBOTS-1; i++){
          for(j=i+1; j<NUM_ROBOTS;j++){
          
              distance_square = get_dist_square(initPos[i], initPos[j]);
                                                                                                         
              if(distance_square < MIN_ROBOT_DIST_SQUARE){ 
              
              all_robots_deployed = false; 
              break;}
              
          }
      }
  }  

  // Set robot position
  for(i=0;i<NUM_ROBOTS;i++)
  {
    wb_supervisor_field_set_sf_vec3f(robots_translation[i],initPos[i]);
    
    for (j=0;j<3;j++) {
      robots[i].pos_prev[j] = initPos[i][j];
    }
  }

}

// Initialization of clusters
void initialize_clusters() {

  int c;
  
  clusters[0].Nseed = NUM_SEEDS;
  
  for (c = 1; c<NUM_SEEDS;c++) {
  
    clusters[c].Nseed = 0;
  }

}

// Initialization of seeds
void initialize_seeds() {

  int e, i;
  double initPos[NUM_SEEDS][3];
  
  // Get seeds references and place seed 0-9
  char ename[3] = "e0";
  for(e=0; e<NUM_SEEDS; e++)
  { 
    seeds_ref[e] = wb_supervisor_node_get_from_def(ename);   
    seeds_translation[e] = wb_supervisor_node_get_field(seeds_ref[e],"translation");
      
    if (!seeds_ref[e]) {
      printf("Missing node for seed #%d\n", e);
      exit(1);
    }
    
    seeds[e].carrying_robot = NULL_ROBOT;
    seeds[e].inside_cluster = 1;
    ename[1]++;
   
  }
  
  for(i=0;i<NUM_SEEDS;i++)
  {
    initPos[i][2] = FREE;                      // use 0 z height
    initPos[i][0] = (RAND*(2*ARENASIZE) - ARENASIZE)/2;     // randomize x
    initPos[i][1] = (RAND*(2*ARENASIZE) - ARENASIZE)/2;      // randomize y
    
    wb_supervisor_field_set_sf_vec3f(seeds_translation[i],initPos[i]);
  }
  
}

// Initialization of robots, clusters, seeds and communication
void initialize(){
  
  printf("\n\n");
  printf("***Welcome to the distributed seed aggregation experiment\n");
  printf("***Number of robots: %d, Number of seeds: %d\n", NUM_ROBOTS, NUM_SEEDS);
  printf("***Total simulation time: %d seconds\n",MAX_SIM_TIME_SEC);
  printf("***Logs enabled or disabled for console: %d\n", ENB_LOG_CON);
  printf("***Logs enabled or disabled for file: %d\n", ENB_LOG_FILE);

  wb_robot_init();
  
  receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver,TIMESTEP);
  wb_receiver_set_channel(receiver,2);
  
  emitter = wb_robot_get_device("emitter");
  wb_emitter_set_channel(emitter,1);
  
  srand(time(NULL));
  initialize_robots();
  initialize_seeds();
  initialize_clusters();
 
  fp = fopen("aggregate_metrics.csv","w+");
  fprintf(fp,"NC ACS SBC\n");
}

// Update the pose and id fields of robots and seeds
void update_objects() {

  int r,e;
  
  for (e=0; e<NUM_SEEDS; e++) {
  
    seeds[e].pos = wb_supervisor_field_get_sf_vec3f(seeds_translation[e]);
    
  }
  
  for (r=0; r<NUM_ROBOTS; r++) {
    robots[r].pos = wb_supervisor_field_get_sf_vec3f(robots_translation[r]);
    robots[r].rot = wb_supervisor_field_get_sf_rotation(robots_orientation[r]);
    robots[r].id = r;
    
  }
}

// Calculate metrics
void calculate_metrics() {
//TODO
NC = 0;
ACS = 0;
SBC = 0;

}

// Show logs if there is any
void print_logs_console() {

  printf("NC: %f, ACS %f, SBC: %f\n",NC, ACS, SBC);

}

// Show logs if there is any
void print_logs_file() {
  
  fprintf(fp,"%f %f %f %f\n", wb_robot_get_time(), NC, ACS, SBC); 

}


// Respawn robots that are stuck or misplaced
void check_for_stucked_robots() {

  int j,r;
  double newPos[NUM_ROBOTS][3];

  for (r=0; r<NUM_ROBOTS; r++) {
  
      redeploy = true;
  
      double distance_square = get_dist_square(robots[r].pos,robots[r].pos_prev);
      
      double rotx = robots[r].rot[0];
      
      double roty = robots[r].rot[1];

      if( distance_square < MIN_STUCK_DIST_SQUARE || rotx > THR_ROT || roty>THR_ROT) {
      
        while (redeploy) {
      
          newPos[r][2] = 0;                      // use 0 z height
          newPos[r][0] = (RAND*(2*ARENASIZE) - ARENASIZE)/2;      // randomize x
          newPos[r][1] = (RAND*(2*ARENASIZE) - ARENASIZE)/2;      // randomize y
          
          redeploy = false;
          
          for(j=1; j<NUM_ROBOTS;j++){
               
             if (j!=r) {
            
                distance_square = get_dist_square(newPos[r], robots[j].pos);
                                                                                                           
                if(distance_square < MIN_ROBOT_DIST_SQUARE){ 
                
                  redeploy= true; 
                  break;
                }
                
              }
          }
         
        }
      
        wb_supervisor_field_set_sf_vec3f(robots_translation[r],newPos[r]);
        wb_supervisor_field_set_sf_rotation(robots_orientation[r],default_rot);
        robots[r].pos = newPos[r];
        
     }
               
    for (j=0;j<3;j++) {    
      robots[r].pos_prev[j] = robots[r].pos[j];
    }
  
  }

}

//Simple main
int main(int argc, char **argv){

int time = 0;
int prev_time = 0;

  initialize();

  while(wb_robot_get_time() < MAX_SIM_TIME_SEC)
  { 
     
     update_objects();
     
     receive_emit_message_from_to_robots();
     
     update_seed_animation();
     
     if (((time-prev_time) > TO_STUCK) && ENB_STUCK_PREV) {
         
         check_for_stucked_robots();
         prev_time = time;
     }
    
     calculate_metrics();
     
     if (ENB_LOG_CON) {
       print_logs_console();
      }
      
      if (ENB_LOG_FILE) {
       print_logs_file();
      }
 
     time +=TIMESTEP;
     
     wb_robot_step(TIMESTEP);
     
  }
  
  wb_supervisor_simulation_set_mode(WB_SUPERVISOR_SIMULATION_MODE_PAUSE);
  fclose(fp);

  return 0;
}
