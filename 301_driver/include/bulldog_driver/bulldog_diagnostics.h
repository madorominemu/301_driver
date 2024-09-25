#ifndef __BULLDOG_DIAGNOSTICS_H
#define __BULLDOG_DIAGNOSTICS_H

#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>

namespace bulldog
{
  class RobotStatusTask : public diagnostic_updater::DiagnosticTask
  {
    public:
      RobotStatusTask() : DiagnosticTask("Robot Status") {}
      void run(diagnostic_updater::DiagnosticStatusWrapper &stat);
      void update(const bool &sp) { _stop = sp; }

    private:
      bool _stop;
  };

  class BatteryStatusTask : public diagnostic_updater::DiagnosticTask
  {
    public:
      BatteryStatusTask() : DiagnosticTask("Battery Status") {}
      void run(diagnostic_updater::DiagnosticStatusWrapper &stat);
      void update(const float &vlt, const int &bcp) 
      { 
        _voltage = vlt; 
        _battery_capacity = bcp;
      }

    private:
      float _voltage;
      int _battery_capacity;
  };

  class MotorStatusTask : public diagnostic_updater::DiagnosticTask 
  {
    public:
      MotorStatusTask() : DiagnosticTask("Motor Status") {}
      void run(diagnostic_updater::DiagnosticStatusWrapper &stat);
      void update(const float &lmc, const float &rmc, const int64_t &lec, const int64_t &rec) 
      { 
        _left_motor_current = lmc;
        _right_motor_current = rmc; 
        _left_encoder_counts = lec;
        _right_encoder_counts = rec;
      }

    private:
      float _left_motor_current;
      float _right_motor_current;
      int64_t _left_encoder_counts;
      int64_t _right_encoder_counts;
  }; 

  class ControllerStatusTask : public diagnostic_updater::DiagnosticTask
  {
    public:
      ControllerStatusTask() : DiagnosticTask("Controller Status") {}
      void run(diagnostic_updater::DiagnosticStatusWrapper &stat);
      void update(const float &tmp) 
      { 
        _temp = tmp;
      }

    private:
      float _temp;
  };
}

#endif 
