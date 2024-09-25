
#include <bulldog_driver/bulldog_diagnostics.h>

namespace bulldog
{
  void RobotStatusTask::run(diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    if(_stop)
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Emergency Stop!");
    }
    else
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
    } 
  }

  void BatteryStatusTask::run(diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    if (_battery_capacity < 20)
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Low Battery!");
    }
    else
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
    }
    stat.add("Voltage (V)", _voltage);
    stat.add("Percent (%)", _battery_capacity);
    stat.add("Capacity (Ah)", 20); 
  }

  void MotorStatusTask::run(diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    if ((_left_motor_current > 30) || (_right_motor_current > 30))
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Over Motor Current!");
    }
    else
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
    }

    stat.add("Left Motor Current (A)", _left_motor_current);
    stat.add("Right Motor Current (A)", _right_motor_current); 
    stat.add("Left Motor Encoder Counts", _left_encoder_counts);
    stat.add("Right Motor Encoder Counts", _right_encoder_counts);
  }

  void ControllerStatusTask::run(diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "OK");
    stat.add("Controller Temp (C)", _temp); 
  }
}
