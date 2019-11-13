// File:          advanced_genetic_algorithm.c
// Description:   Robot execution code for genetic algorithm
// Project:       Advanced exercises in Cyberbotics' Robot Curriculum
// Author:        Yvan Bourquin - www.cyberbotics.com
// Date:          January 6, 2010

#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/receiver.h>
#include <webots/emitter.h>
#include <webots/distance_sensor.h>
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <webots/gps.h>

#define NUM_SENSORS 11
#define NUM_WHEELS 2
#define GENOTYPE_SIZE (NUM_SENSORS * NUM_WHEELS)

// sensor to wheels multiplication matrix
// each each sensor has a weight for each wheel
double matrix[NUM_SENSORS][NUM_WHEELS];

// 3 IR ground color sensors
#define NB_GROUND_SENS 3
#define GS_WHITE 900
#define GS_LEFT 0
#define GS_CENTER 1
#define GS_RIGHT 2

#define NB_DIST_SENS 8

#define NUM_CHECKS 17

unsigned short gs_value[NB_GROUND_SENS]={0,0,0};

WbDeviceTag sensors[NUM_SENSORS];  // proximity sensors
WbDeviceTag receiver;              // for receiving genes from Supervisor
WbDeviceTag robot_emitter;
WbDeviceTag robot;
WbDeviceTag gps;

double f[1];
int mili = 0;
double cur_x, prev_x, cur_y, prev_y;

int cur_check = 0;

double checks_x[NUM_CHECKS] = {
//turn 1
0.246938,  0.245117, 0.213594,  0.054236, -0.150587, -0.243441,
//ob 1
-0.192012, -0.170405, -0.139799, -0.15743,
//turn 2
-0.212677, -0.0907201, 0.0653666,
//ob 2
0.0893589, 0.153887, 0.212742
};
double checks_z[NUM_CHECKS] = {
//turn 1
-0.042318, 0.049354, 0.207657,  0.298794, 0.273649,  0.176932,
//ob 1
0.0702441, 0.0473333, -0.00430759, -0.0220507,
//turn 2
-0.215883, -0.273285, -0.280998,
//ob 2
-0.189704, -0.184477, -0.211189
};


// check if a new set of genes was sent by the Supervisor
// in this case start using these new genes immediately
void check_for_new_genes() {
  if (wb_receiver_get_queue_length(receiver) > 0) {
  
    f[0] = 0;
    cur_check = 0;
    // check that the number of genes received match what is expected
    assert(wb_receiver_get_data_size(receiver) == GENOTYPE_SIZE * sizeof(double));
    
    // copy new genes directly in the sensor/actuator matrix
    // we don't use any specific mapping nor left/right symmetry
    // it's the GA's responsability to find a functional mapping
    memcpy(matrix, wb_receiver_get_data(receiver), GENOTYPE_SIZE * sizeof(double));
    
    // prepare for receiving next genes packet
    wb_receiver_next_packet(receiver);
  }
}

static double clip_value(double value, double min_max) {
  if (value > min_max)
    return min_max;
  else if (value < -min_max)
    return -min_max;

  return value;
}

void sense_compute_and_actuate() {
  // read sensor values
  double sensor_values[NUM_SENSORS];
  int i, j;
  for (i = 0; i < NUM_SENSORS; i++)
    sensor_values[i] = wb_distance_sensor_get_value(sensors[i]);
  
  for(i = NB_DIST_SENS; i < NUM_SENSORS; i ++)
    gs_value[i - NB_DIST_SENS] = sensor_values[i];
  
  //Detect if the robot is touch an obstacle
  int touching = 0;
  for(i=0; i < NB_DIST_SENS; i ++) {
    if(sensor_values[i] > 200) touching = 1;
  }
  //Detect if the robot is close to an abstacle
  int dodging = 0;
  if(!touching) {
    for(i=0; i < NB_DIST_SENS; i ++) {
      if(sensor_values[i] > 2000) dodging = 1;
    }
  }
  
  //Check and punish if the robot is not moving
  if(wb_differential_wheels_get_left_speed() >= 999 && wb_differential_wheels_get_right_speed() >= 999) {
    f[0] -= 3;
  }
  
  //Punishment for being stuck over the line
  if(touching && !dodging) {
    for(i=0; i < 3; i ++) 
     if(gs_value[i] < 500) f[0] -= 1;
  }
  //Reward for moving past an obstacle
  if(!touching && dodging) {
     if(gs_value[1] > 400) f[0] ++;
  }
  //Reward for following the line
  //Punish for loosing the line
  if(!touching && !dodging) {
    if(gs_value[1] < 400) f[0] ++;
    else f[0] -= 2;
    if(gs_value[0] < 400) f[0] += 2;
    else  f[0] -= 2;
    if(gs_value[2] < 400) f[0] ++;
    else f[0] -= 2;
  }
  
    /*
  for(i=0; i < 3; i ++) {
    if(gs_value[i] > 400 && gs_value[i] < 900) f[0] -= 2;
    printf("gs: %d\n", gs_value[i]);
  }
  printf("\n");
  */
  
  /*
  const double *cur_pos = wb_gps_get_values(gps);
  for(i=0; i < 3; i ++) {
    double dist = sqrt(pow((cur_pos[0] - checks_x[cur_check + i]), 2) + pow((cur_pos[2] - checks_z[cur_check + i]), 2));
    if(dist < 0.02) {
      f[0] += 500;
      printf("cc: %d\n", cur_check);
      if(cur_check < NUM_CHECKS) cur_check ++;
      else cur_check = 0;
      break;
    }
  }
  */
  
  
  wb_emitter_send(robot_emitter, f, sizeof(double));

  // compute actuation using Braitenberg's algorithm:
  // The speed of each wheel is computed by summing the value
  // of each sensor multiplied by the corresponding weight of the matrix.
  // By chance, in this case, this works without any scaling of the sensor values nor of the
  // wheels speed but this type of scaling may be necessary with a different problem
  double wheel_speed[NUM_WHEELS] = { 0.0, 0.0 };
  for (i = 0; i < NUM_WHEELS; i++)
    for (j = 0; j < NUM_SENSORS; j++)
      wheel_speed[i] += matrix[j][i] * sensor_values[j];
      
  // clip to e-puck max speed values to avoid warning
  wheel_speed[0] = clip_value(wheel_speed[0], 1000.0);
  wheel_speed[1] = clip_value(wheel_speed[1], 1000.0);

  // actuate e-puck wheels
  wb_differential_wheels_set_speed(wheel_speed[0], wheel_speed[1]);
}

int main(int argc, const char *argv[]) {

  f[0] = 0;
  
  wb_robot_init();  // initialize Webots

  // find simulation step in milliseconds (WorldInfo.basicTimeStep)
  int time_step = wb_robot_get_basic_time_step();
    
  // find and enable proximity sensors
  char name[32];
  int i;
  for (i = 0; i < NB_DIST_SENS; i++) {
    sprintf(name, "ps%d", i);
    sensors[i] = wb_robot_get_device(name); /* proximity sensors */
    wb_distance_sensor_enable(sensors[i],time_step);
  }
  for (; i < NUM_SENSORS; i++) {
    sprintf(name, "gs%d", i-NB_DIST_SENS);
    sensors[i] = wb_robot_get_device(name); /* ground sensors */
    wb_distance_sensor_enable(sensors[i],time_step);
  }
    
  // find and enable receiver
  receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver, time_step);
  
  robot = wb_robot_get_device("e-puck");
  
  
  
  //find and enable robot_emitter
  robot_emitter = wb_robot_get_device("emitter");
  
  gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, 1);
  
  // initialize matrix to zero, hence the robot 
  // wheels will initially be stopped
  memset(matrix, 0.0, sizeof(matrix));
  
  // run until simulation is restarted
  while (wb_robot_step(time_step) != -1) {
    check_for_new_genes();
    sense_compute_and_actuate();
  }
  
  wb_robot_cleanup();  // cleanup Webots
  return 0;            // ignored
}
