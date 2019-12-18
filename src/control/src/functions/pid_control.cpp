#include "pid_control.h"

using namespace std;
using namespace control;

namespace control
{
  PIDControl::PIDControl()
  {
    ROS_INFO("PIDControl constructor Fun.");
  }

  PIDControl::~PIDControl()
  {
    ROS_INFO("PIDControl destructor Fun.");
  }


  void PIDControl::init(const PIDConf &pid_conf)
  {
    previous_error_ = 0.0;
    previous_output_ = 0.0;
    integral_ = 0.0;
    first_hit_ = false;
    integrator_enabled_ = false;
    integrator_hold_ = false;
  
    integrator_saturation_high_ = 0.0;
    integrator_saturation_low_ = 0.0;
    integrator_saturation_status_ = 0;

    setPID(pid_conf);

  }

  void PIDControl::setPID(const PIDConf &pid_conf)
  {
    kp_ = pid_conf.kp;
    ki_ = pid_conf.ki;
    kd_ = pid_conf.kd;
    kaw_ = pid_conf.kaw;
    ROS_INFO("kp:%f,ki:%f,kd:%f",kp_,ki_,kd_);
  }

  void PIDControl::reSet()
  {
    previous_error_ = 0.0;
    previous_output_ = 0.0;
    integral_ = 0.0;
    first_hit_ = true;
    integrator_saturation_status_ = 0;
  }

  double PIDControl::control(const double error, const double dt)
  {
    if (dt <= 0) {
      ROS_INFO("dt:%lf <= 0, control = %lf",dt,previous_output_);
      return previous_output_;
    }

    double ans_PID = 0;
    double diff = 0;

    if (first_hit_) {
      first_hit_ = false;
    }
    else{
      diff = error - previous_error_;
    }

    if (!integrator_enabled_) {
      integral_ = 0;
    }
    else
    {
      integral_ += error * dt * ki_;
      if (!integrator_hold_ ) 
      {
        if (integral_ > integrator_saturation_high_) 
        {
          integral_ = integrator_saturation_high_;
          integrator_saturation_status_ = 1;
        }
        else if (integral_ < integrator_saturation_low_)
        {
          integral_ = integrator_saturation_low_;
          integrator_saturation_status_ = -1;
        }
        else
        {
          integrator_saturation_status_ = 0;
        }        
      }
    }

    ans_PID = error *kp_ + integral_ + diff * kd_;
    //ans_PID = 0.6*previous_output_ + 0.4*ans_PID;
    //ROS_INFO("PID ANS:%f,kp_%f %f, %f",ans_PID,kp_,error,previous_error_);
    previous_error_ = error;
    previous_output_ = ans_PID;
    return ans_PID;
  }

  int PIDControl::integratorSaturationStatus() const
  {
    return integrator_saturation_status_;
  }

  bool PIDControl::integratorHold() const
  {
    return integrator_hold_;
  }
  void PIDControl::setIntegratorHold(bool hold)
  {
    integrator_hold_ = hold;
  }
}
