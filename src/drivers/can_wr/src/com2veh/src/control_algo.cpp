#include "math.h"
#include "control_algo.h"

void pid_loop(PID* pid, float expect, float measure, double curtime)
{
	float err, diff, dt;
	err = expect - measure;
	dt = (float)(curtime - pid->lasttime);
	pid->lasttime = curtime;
	if(std::isnan(err)) return;

	diff = (err - pid->preErr) / dt;
	pid->preErr = err;

	pid->I_sum += err * dt;
	if(pid->I_sum > pid->I_max )
		pid->I_sum = pid->I_max;
	if(pid->I_sum < -pid->I_max )
		pid->I_sum = -pid->I_max;

	pid->Pout = pid->kp * err;
	pid->Iout = pid->ki * pid->I_sum;
	pid->Dout = pid->kd * diff;
	if(pid->Dout > pid->D_max) pid->Dout = pid->D_max;
	if(pid->Dout < -pid->D_max) pid->Dout = -pid->D_max;

	pid->Output = pid->Pout + pid->Dout + pid->Iout;
	//printf("kp is: %f \n", pid->kp);
	ValueConstrain(pid->Pout, pid->Out_min, pid->Out_max);
}

float ValueConstrain(float val, float min, float max)
{
	return (val < min) ? min : ((val > max) ? max : val);
}
