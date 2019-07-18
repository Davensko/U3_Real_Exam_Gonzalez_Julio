/*
 * File:          U3_Real_Exam_Gonzalez_Julio.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */


#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/distance_sensor.h>

#include <stdio.h>
#include <math.h>

/*
 * You may want to add macros here.
 */
#define TIME_STEP 64
#define PI 3.14159

#define OBSTACLE_DISTANCE 0.2 
#define RANGE 0.4 
#define MAXRE 255  

#define OBSTACLE_DISTANCE_2 0.72 
#define RANGE2 2 
#define MAXRE2 1023

double encoder;
double dis;
double dis2;  

enum {
  GO,
  TURN,
  FREEWAY,
  OBSTACLE,
  LOOKING,
  ENEMY, 
  NOENEMY,
  STOP
};

double initial_angle_wheel1;

int checkForObstacles(WbDeviceTag sen_1) {
  double distance = wb_distance_sensor_get_value(sen_1);
  
  dis = (distance*RANGE)/MAXRE;
  
  //printf("Value of resolution: %f\n", distance);
  //printf("Distance: %f\n", dis); 
  
  if (dis > OBSTACLE_DISTANCE)
    return FREEWAY;
  else
    return OBSTACLE;
}

int checkForEnemy(WbDeviceTag distance_sensor) {
  double distance2 = wb_distance_sensor_get_value(distance_sensor);
  
  dis2 = (distance2*RANGE2)/MAXRE2;
  
  if(dis2 <= 0.7) {
    printf("Enemy: THA\n");
  }
 
  //printf("Distance Enemy: %f\n", dis2);
  
  if (dis2 > OBSTACLE_DISTANCE_2)
    return NOENEMY;
  else
    return ENEMY;
}

void goRobot(WbDeviceTag *wheels, double velocity) {
  wb_motor_set_position(wheels[0], INFINITY);
  wb_motor_set_velocity(wheels[0], 0);
  wb_motor_set_position(wheels[1], INFINITY);
  wb_motor_set_velocity(wheels[1], -velocity);
  wb_motor_set_position(wheels[2], INFINITY);
  wb_motor_set_velocity(wheels[2], velocity);
}

void stopRobot(WbDeviceTag *wheels) {
  wb_motor_set_velocity(wheels[0], 0);
  wb_motor_set_velocity(wheels[1], 0);
  wb_motor_set_velocity(wheels[2], 0);
}

void turnRight(WbDeviceTag *wheels) {

  wb_motor_set_velocity(wheels[0], 0.6);
  wb_motor_set_velocity(wheels[1], 0.6);
  wb_motor_set_velocity(wheels[2], 0.6);
}

void rotateSensor(WbDeviceTag *radar) {
  wb_motor_set_position(radar[0], INFINITY);
  wb_motor_set_velocity(radar[0], -3);
}

void stopSensor(WbDeviceTag *radar) {
  wb_motor_set_position(radar[0], INFINITY);
  wb_motor_set_velocity(radar[0], 0);
}

void rotateGun(WbDeviceTag *gun, double pos) {
  wb_motor_set_position(gun[0], pos);
  
}

double getAngleRobot(WbDeviceTag pos_sensor) {
  
  double angle, angle_wheel1;

  angle_wheel1 = wb_position_sensor_get_value(pos_sensor);
  angle = fabs(angle_wheel1 - initial_angle_wheel1);

  return angle;
}

double getAngle(WbDeviceTag radar_sensor) {
  double  angle2, radar_angle; 
  
  radar_angle = wb_position_sensor_get_value(radar_sensor);
  
  angle2 = radar_angle; 
  //printf("enco_radar: %f\n", radar_angle);
  
  return angle2; 
}


int main(int argc, char **argv)
{
  /* necessary to initialize webots stuff */
  wb_robot_init();

  WbDeviceTag wheel_1 = wb_robot_get_device("Wheel_1");
  WbDeviceTag wheel_2 = wb_robot_get_device("Wheel_2");
  WbDeviceTag wheel_3 = wb_robot_get_device("Wheel_3");

  WbDeviceTag wheels[3];
  wheels[0] = wheel_1;
  wheels[1] = wheel_2;
  wheels[2] = wheel_3;

  WbDeviceTag Radar = wb_robot_get_device("Radar");
  WbDeviceTag radar[2];
  radar[0] = Radar;
  
  WbDeviceTag Gun = wb_robot_get_device("Gun");
  WbDeviceTag gun[2];
  gun[0] = Gun; 
  
  // Encoder devices
  WbDeviceTag encoder = wb_robot_get_device("encoder_1");
  wb_position_sensor_enable(encoder, TIME_STEP);
  
  WbDeviceTag enco_radar = wb_robot_get_device("enco_radar");
  wb_position_sensor_enable(enco_radar, TIME_STEP);
  
  //WbDeviceTag enco_gun = wb_robot_get_device("enco_gun");
  //wb_position_sensor_enable(enco_gun, TIME_STEP);

  // Distance sensor devices
  WbDeviceTag dist_sensor = wb_robot_get_device("sen_1");
  wb_distance_sensor_enable(dist_sensor, TIME_STEP);
  
  WbDeviceTag rad = wb_robot_get_device("distance_sensor");
  wb_distance_sensor_enable(rad, TIME_STEP);
 
  short int ds_state,robot_state = GO;
  float velocity = 2.5;
  float angle;
  float angle2; 
  float pos; 
  int vueltas = 50;  

  while (wb_robot_step(TIME_STEP) != -1) {
    
    rotateSensor(radar);
      
    if (robot_state == GO) {
      ds_state = checkForObstacles(dist_sensor);
      
      if (ds_state == FREEWAY) {
        goRobot(wheels, velocity);
        angle = wb_position_sensor_get_value(encoder);
        //printf("Angle: %lf\n", angle);
        
      } else if (ds_state == OBSTACLE) {
        robot_state = TURN;
        stopRobot(wheels);
        initial_angle_wheel1 = wb_position_sensor_get_value(encoder);
      }
    } else if (robot_state == TURN) {
        turnRight(wheels);
        angle = getAngleRobot(encoder);
      
          if (angle >= PI ) {
            robot_state = LOOKING;  
            stopRobot(wheels);
          }
    }
    if (robot_state == LOOKING) {
      ds_state = checkForEnemy(rad); 
      
        if (ds_state == NOENEMY) {
          goRobot(wheels, velocity);
        }
        else if (ds_state == ENEMY) {
          ds_state = STOP;
          stopRobot(wheels);
          stopSensor(radar);
          angle2 = getAngle(enco_radar);
          pos = (vueltas + angle2)*-1; 
          //printf("Pos: %lf\n", pos); 
        }
        if (ds_state== STOP) {
       
          rotateGun(gun, pos);
        
            if (dis2 >= 0.7 && dis2 <= 0.5 ) {
              printf("Enemy: THA\n");
            }
            else if (dis2 <= 0.5 && dis2 >= 0.2) {
              printf("Enemy: THA THA\n");
            }
            else if (dis2 <= 0.2 && dis2 >= 0.1) {
              printf("Enemy: THA THA THA\n");
            }
        }
    } 
    
  };


  wb_robot_cleanup();

  return 0;
}
