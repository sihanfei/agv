#ifndef _CONTROL_ALGO_H
#define _CONTROL_ALGO_H
#include "ros/ros.h"


struct PID 
{
	float kp,ki,kd,preErr,Pout,Iout,Dout,Output,Out_max,Out_min,I_max,I_sum,D_max; //fCut = 20Hz,fCut,lastDiff,dt,
	double lasttime;
	// PID(){}
	// PID(float x1, float x2, float x3)
	// {
	// 	kp = x1;
	// 	kd = x2;
	// 	lasttime = x3;
	// }
	// ~PID(){}
} ; //PID

void pid_loop(PID* pid, float expect, float measure, double curtime);
float ValueConstrain(float val, float min, float max);

#endif

