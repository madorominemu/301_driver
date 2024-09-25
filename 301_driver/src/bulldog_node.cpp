#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <bulldog_driver/bulldog_hw.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "bulldog_driver");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh, private_nh("~");
  std::string controller_port;
  std::string display_port;
  private_nh.param<std::string>("controller_port", controller_port, "/dev/ttyACM0");
  private_nh.param<std::string>("display_port", display_port, "/dev/ttyUSB0");

  double control_frequency, diagnostic_frequency;
  private_nh.param<double>("control_frequency", control_frequency, 15.0);
  private_nh.param<double>("diagnostic_frequency", diagnostic_frequency, 4.0);
  ros::Duration control_period(1 / control_frequency);
  ros::Duration diagnostic_period(1 / diagnostic_frequency);

  bulldog::BulldogHW bulldog(controller_port, display_port, diagnostic_period.toSec());
  controller_manager::ControllerManager cm(&bulldog, nh);
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  {
    ros::console::notifyLoggerLevelsChanged();
  }
  while (ros::ok())
  {
    bulldog.read();
    cm.update(ros::Time::now(), control_period);
    bulldog.write();
  }
}
