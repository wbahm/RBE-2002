#ifndef RATES_H
#define RATES_H

/* This file has all the timing for the various control loop rates.
   Try not to change these unless you know what your doing because
   it might change system stability for example if you increase
   the motor velocity PID then we might not have many ticks to
   base speed off of.*/
   
#define ODOMETRY_FREQ 200 //hz
#define ODOMETRY_PERIOD_US 1000000/ODOMETRY_FREQ //cycle period in usec

#define MOTOR_CONTROLLER_FREQ 200 //hz
#define MOTOR_CONTROLLER_PERIOD_US 1000000/MOTOR_CONTROLLER_FREQ //cycle period in usec

#define WALL_FOLLOW_FREQ 60 //hz
#define WALL_FOLLOW_PERIOD_US 1000000/WALL_FOLLOW_FREQ //cycle period in usec

#endif
