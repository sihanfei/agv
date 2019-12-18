#ifndef PID_CONTROL_H_
#define PID_CONTROL_H_

#include "ros/ros.h"
#include <math.h>
#include "control_utils.h"

namespace control{
  class PIDControl{
    public:
      PIDControl();
      ~PIDControl();
      
      void init(const PIDConf &pid_conf);
      void setPID(const PIDConf &pid_conf);
      void reSet();
      virtual double control(const double error, const double dt);
      int integratorSaturationStatus() const;
      bool integratorHold() const;
      void setIntegratorHold(bool hold);
    
    protected:
      double kp_;
      double ki_;
      double kd_;
      double kaw_;
      double previous_error_;
      double previous_output_;
      double integral_;
      double integrator_saturation_high_;
      double integrator_saturation_low_;
      bool first_hit_;
      bool integrator_enabled_;
      bool integrator_hold_;
      int integrator_saturation_status_;
      double output_saturation_high_;
      double output_saturation_low_;
      int output_saturation_status_;

  };

}//end namespace control
#endif