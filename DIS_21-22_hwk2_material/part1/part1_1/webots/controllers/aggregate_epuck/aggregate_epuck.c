/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * file:         aggregate_epuck.c
 * description:  E-Puck controller for seed aggregation
 *
 * $Author$: Kagan Erunsal
 * $Revision$: 
 * $Date$: 2022-05-20
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
//Includes
#include <stdio.h>
#include <stdlib.h>
#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <webots/receiver.h>
#include <webots/emitter.h>
#include <webots/motor.h>
#include <math.h>
//Include time.h t0 generate random speed.
#include <time.h>

//Gloabal defines
#define MAX_SPEED_WEB      6.28        // Maximum speed webots
#define NB_SENSORS           8         // Number of distance sensors 
#define BIAS_SPEED          100         // Bias speed for Braitanberg
#define MAX_SPEED           800        // Max wheel speed
#define TIMESTEP            32         // Timestep of the controller
#define RAND ((float) rand()/RAND_MAX) 
#define THR_DIST_SENS_CRIT  1000        // Distance sensor critical threshold for detection
#define THR_DIST_SENS_NORM  100        // Distance sensor normal threshold for detection
#define TO_GRIP             200        // Counter timeout for gripping transition
#define TO_REL              200        // Counter timeour for releasing transition
#define TO_OA               200        // Counter timeout for obstacle avoidance
#define TO_SEARCH           200        // Counter timeout for search
#define ENB_LOG             0        // Enable it to see the logs
#define FAIL                 99        // Failed message coming from supervisor

//Gloabl variables
WbDeviceTag left_motor; //handler for left wheel of the robot
WbDeviceTag right_motor; //handler for the right wheel of the robot
WbDeviceTag receiver;  // handler for the receiver     
WbDeviceTag emitter; // handler for the emitter
WbDeviceTag ds[NB_SENSORS]; //handler for the distance sensors

int msl, msr;
float msl_w, msr_w;
double braiten[16] = {0.6,0.4,0.4,0.1,-0.1,-0.2,-0.4,-0.6,-0.5,-0.5,-0.3,-0.1,0.1,0.3,0.4,0.5};	
double distances[NB_SENSORS]; // array keeping the distance sensor readings
int robot_id = 0; // id for robots 1-10
int obj_type = 0; // No object: 0, Free seed:1, Wall,robot or cluster:2
int detected_ps = 0; // The distance sensors that detects free seed
int state = 0; // Search-free: 0, Search-loaded: 1, Gripping: 2, Releasing: 3, Obs. Avoid-free: 4, Obs Avoid-loaded: 5   
bool loaded = false; // Robot carries seed: true, Robot doesnt carry seed: false
int decision = 0; //Supervisors decision on physically grabbing the seed !99, cannot grabbed the seed: 99 
int query = 0; //Robot wants to release the seed: 0, to grip the seed 1
int counter = 0; // Counter for transitions between states

// Initialize robots, motors, sensors and communication
static void initalize(void) {
  
  //Webots init
  wb_robot_init();               
  
  //Motors
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  
  //Distance sensors
  char s[4]="ps0";
  int i = 0;
  for(i=0; i<NB_SENSORS;i++) {
    ds[i] = wb_robot_get_device(s);      
    wb_distance_sensor_enable(ds[i],TIMESTEP);
    s[2]++;                        
  }
  
  //Robot
  char* robot_name;
  robot_name=(char*) wb_robot_get_name(); 
  sscanf(robot_name,"e-puck%d",&robot_id);
  
  //Receiver
  receiver = wb_robot_get_device("receiver");
  wb_receiver_set_channel(receiver,1);
  wb_receiver_enable(receiver,TIMESTEP);
  
  //Emitter
  emitter = wb_robot_get_device("emitter");
  wb_emitter_set_channel(emitter,2);
  
}

//Read all distance sensors
void read_distance_sensors() {

  int sensor_nb;

  for(sensor_nb = 0;sensor_nb < NB_SENSORS;sensor_nb++){  // read sensor values
      distances[sensor_nb] = wb_distance_sensor_get_value(ds[sensor_nb]);     
    }
}

//Obstacle avoidance behaviour
void perform_obstacle_avoidance() {
//TODO
  static double l_weight[NB_SENSORS] = {-1, -0.75, -0.2, 0, 0, 0.2, 0.75, 1};
  static double r_weight[NB_SENSORS] = {1, 0.75, 0.2, 0, 0, -0.2, -0.75, -1};
  
  double left_speed, right_speed;
  
  for (int i = 0; i < NB_SENSORS; i++)
  {
    right_speed += (r_weight[i]) * ds[i];
    left_speed += (l_weight[i]) * ds[i];
  }
  
  double val = abs(ds[7] - ds[0]);
  
  
  if (val < 200 && ds[0] > 0){
  
  left_speed = 100; // left  wheel velocity in rad/s
  right_speed = -100; // right wheel velocity in rad/s
  
  }
  
  msl_w = left_speed*MAX_SPEED_WEB/1000;
  msr_w = right_speed*MAX_SPEED_WEB/1000;
  counter++;
}

// Send query to supervisor about gripping and releasing
void emit_message_to_supervisor(int robot_id, int query, int detected_ps) {

  int pkt_send[3];
  
  pkt_send[0] = robot_id; pkt_send[1] = query; pkt_send[2] = detected_ps;
  wb_emitter_send(emitter,pkt_send,3*sizeof(int));
  
}

// Receive decision from supervisor about gripping and releasing
void receive_message_from_supervisor() {

  int queue_length, k;
  const int *pkt;
  
  queue_length = wb_receiver_get_queue_length(receiver);
  
  for (k = 0; k < queue_length; k++) {
  
       pkt = (int*)wb_receiver_get_data(receiver);
       int r = pkt[0];

       if (r==robot_id) {
               
         decision = pkt[1];
       
       }
  }
    
  wb_receiver_next_packet(receiver); 

}

// Limit speeds inside physical limits
void limit_speeds() {

    if(msl > MAX_SPEED) msl = MAX_SPEED;
    if(msl < -MAX_SPEED) msl = -MAX_SPEED;
    if(msr > MAX_SPEED) msr = MAX_SPEED;
    if(msr < -MAX_SPEED) msr = -MAX_SPEED;

}

// Detect big and small objects
void identify_object() {
// TODO
  int num_detected = 0;
  int final_activated_sensor = 0;
  for (int sensor_nb = 0;sensor_nb < NB_SENSORS;sensor_nb++) {  // read sensor values
      if (distances[sensor_nb] > THR_DIST_SENS_CRIT) {
        num_detected++;
        final_activated_sensor = sensor_nb;
        for (int j=0; j<sensor_nb; j++) {
          if (distances[j] > 150) {
            num_detected++;
          }
        }
        for (int k=sensor_nb+1; k<NB_SENSORS; k++) {
          if (distances[k] > 150) {
            num_detected++;
          }
        }
      }
  }
  // printf("***num_detected: %d***\n", num_detected);
  if (num_detected == 0) {obj_type = 0;}
  else if (num_detected == 1) {
    obj_type = 1;
    detected_ps = final_activated_sensor;
  }
  else {obj_type = 2;}
  counter++;
}

// Basic search behaviour with timeout
void search() {
//TODO
  srand((unsigned)(time(NULL)));
  int const_speed = 800;
  int rand_num[5];
  for (int i=0; i<5; i++){
    rand_num[i] = rand();
  }
  for (int j=0; j<5; j++) {
    if (robot_id == j) {
        msl = rand_num[j] % MAX_SPEED;
      }
   }
  msr = const_speed - msl;
  counter++;
}

// Just wait with timeout
void wait() {

  msl = 0;
  msr = 0;
  counter++;
}

// Send release message to supervisor
void release_seed() {
//TODO
  query = 0;
  emit_message_to_supervisor(robot_id, query, detected_ps);
}

// Send grip message to supervisor
void grip_seed() {
//TODO
  query = 1;
  emit_message_to_supervisor(robot_id, query, detected_ps);
}

// All FSM is implemented here
void perform_task_transition() {

 switch (state) {
    
      case 0: // Search-free
        
        search();
        
        if (counter>TO_SEARCH) {
        
          identify_object();
          
          switch (obj_type) {
          
            case 1:
            
              state = 2; //Gripping
              grip_seed();
              counter = 0;
              break;
              
            case 2:
            
              state = 4; //OA-free
              counter = 0;
              break;
          }     
        }      
        break;
            
      case 1: // Search-loaded
      
        search();
        
        if (counter>TO_SEARCH) {
        
        identify_object();
        
          switch (obj_type) {
          
            case 1:
            
              state = 3; //Releasing
              release_seed();
              counter = 0;
              break;
              
            case 2:
            
              state = 5; //OA-loaded
              counter = 0;
              break;
          }
        }
        break;
          
      case 2: //Gripping
          
        wait();
                    
        if(counter>TO_GRIP) {
        
          if(decision != FAIL) {
                  
            counter = 0;
            state = 5; //OA-loaded
            loaded = true;
            
          } else {
          
            counter = 0;
            state = 4; //OA-free
            loaded = false;
          
          }
        
        }
        break;
      
      case 3: //Releasing
      
        wait();
              
        if(counter>TO_REL) {
                 
          if(decision == FAIL) {
            
            counter = 0;
            state = 4; //OA-free
            loaded = false;
                  
          } else {
          
            counter = 1;
            state = 5; //OA-loaded
            loaded = true;
          
          
          }        
        }
        break;
      
      case 4: //OA-free
      
        perform_obstacle_avoidance();
        
        if(counter>TO_OA) {
        
          state = 0;  // Search-free
          counter = 0;
        
        }
        break;
             
      case 5: //OA-loaded
      
        perform_obstacle_avoidance();
        
        if(counter>TO_OA) {
        
          state = 1; // Search-loaded
          counter = 0;
        
        }
        break;
      
      }
}

// Set motor speeds
void set_speeds() {

  msl_w = msl*MAX_SPEED_WEB/1000;
  msr_w = msr*MAX_SPEED_WEB/1000;
  wb_motor_set_velocity(left_motor, msl_w);
  wb_motor_set_velocity(right_motor, msr_w);
  
}


void print_logs_on_console() {

  printf("***Robot ID: %d***\n", robot_id);
  printf("Distance sensors 0-7: %f %f %f %f %f %f %f %f\n", distances[0], distances[1], distances[2], distances[3], distances[4], distances[5], distances[6], distances[7]);
  printf("Object type: %d\n", obj_type);
  printf("Loaded: %d\n", loaded);
  printf("State: %d\n", state);
  printf("Query: %d\n",query);
  printf("Decision: %d\n", decision);
}

//Simple main loop
int main(){
  
  initalize();

  for(;;){
    
    msl=0; msr=0; 
    
    read_distance_sensors();
    
    receive_message_from_supervisor(); 
    
    perform_task_transition();     
          
    limit_speeds();
   
    set_speeds();
    
    if (ENB_LOG) {
    
      print_logs_on_console();
    
    }
       
    wb_robot_step(TIMESTEP); // Executing the simulation for TIMESTEP ms
    
  }
}  
  