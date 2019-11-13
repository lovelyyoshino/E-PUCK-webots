#include <webots/robot.h>
#include <webots/differential_wheels.h>
#include <webots/accelerometer.h>
#include <webots/led.h>
#include <webots/distance_sensor.h>
#include <webots/light_sensor.h>
#include <webots/camera.h>
#include <stdio.h>

#define TIME_STEP     64
#define ON            1
#define OFF           0
#define NB_LEDS       10
#define NB_LIGHT_SENS 8
#define NB_DIST_SENS  8

int main(int argc, char *argv[]) {
  
  WbDeviceTag accelerometer;
  WbDeviceTag led[NB_LEDS];
  WbDeviceTag ps[NB_DIST_SENS];
  WbDeviceTag ls[NB_LIGHT_SENS];
  WbDeviceTag cam;

  const double *a;
  const unsigned char* im;
  short direction = 1;
  int it,m,n;
  long int sum;
  double count;
  int camera_width,camera_height;
  
  /* initialize Webots */
  wb_robot_init();

  /* get and enable devices */
  char text[5] = "led0";
  for(it=0; it<NB_LEDS; it++) {
    led[it] = wb_robot_get_device(text);
    text[3]++;
    wb_led_set(led[it],OFF);
  }
  char textPS[] = "ps0";
  for (it=0; it<NB_DIST_SENS; it++) {
    ps[it] = wb_robot_get_device(textPS);
    textPS[2]++;
    wb_distance_sensor_enable(ps[it],2*TIME_STEP);
  }

  char textLS[] = "ls0";
  for (it=0;it<NB_LIGHT_SENS;it++) {
    ls[it] = wb_robot_get_device(textLS);
    textLS[2]++;
    wb_light_sensor_enable(ls[it], 2*TIME_STEP);
  }
  wb_differential_wheels_enable_encoders(TIME_STEP);
  wb_differential_wheels_set_encoders(0.0,0.0);
  accelerometer = wb_robot_get_device("accelerometer");
  wb_accelerometer_enable(accelerometer,TIME_STEP);
  cam = wb_robot_get_device("camera");
  wb_camera_enable(cam,4*TIME_STEP);
  camera_width = wb_camera_get_width(cam);
  camera_height = wb_camera_get_height(cam);
  
  /* main loop */
  while (wb_robot_step(TIME_STEP)!=-1) {
    sum = 0;
    count = 0.0;
    
    im = wb_camera_get_image(cam);
    for (m=0;m<camera_width;m++)
      for (n=0;n<camera_height;n++)
        sum += wb_camera_image_get_grey(im,camera_width,m,n);
    
    a = wb_accelerometer_get_values(accelerometer);
    
    for (it=0;it<NB_DIST_SENS;it++) {
      count += wb_distance_sensor_get_value(ps[it]);
    }
    
    for(it=0;it<NB_LEDS;it++) {
      wb_led_set(led[it],OFF); 
    }
    
    if (wb_robot_get_mode()==1){
      wb_led_set(led[0],1);
    }
    
    if (a[0]<=0.0 && a[1]<=0.0) wb_led_set(led[1],1);
    if (a[0]<=0.0 && a[1]>0.0)  wb_led_set(led[3],1);
    if (a[0]>0.0 && a[1]>0.0)   wb_led_set(led[5],1);
    if (a[0]>0.0 && a[1]<=0.0)  wb_led_set(led[7],1);
    
    if (count > 7000){
      wb_led_set(led[8],1);
    }
    
    if (sum < 100000){
      wb_led_set(led[9],1);
    }
    
    if (wb_differential_wheels_get_left_encoder() > 1000.0 ||
        wb_differential_wheels_get_left_encoder() < -1000.0){
      if (direction == 1) direction = -1;
      else direction = 1;
      wb_differential_wheels_set_encoders(0.0,0.0);
      printf("Other direction\n");
    }
    
    wb_differential_wheels_set_speed(300.0*direction,-300.0*direction);
  }
  
  wb_robot_cleanup();
  
  return 0;
}
