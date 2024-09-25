#include <bulldog_driver/bulldog_hw.h>

namespace bulldog
{

  BulldogHW::BulldogHW(std::string controller_port, std::string display_port, double diagnostic_period)
  {
    using namespace hardware_interface;
    ros::NodeHandle prave_nh("~");
    int ratio, encoder;
    prave_nh.param("ratio", ratio, int(15));       //减速比
    prave_nh.param("encoder", encoder, int(2500)); //编码器线束
    soc_sub = nh.subscribe("battery", 100, &BulldogHW::socCallback,this);
    device_sstatus_sub = nh.subscribe("send/motor_status", 100, &BulldogHW::motorCallback,this);
    motor_status_pub_ =nh.advertise<std_msgs::Int32>("recv/motor_status",10);
    pp_ = (2 * PI) / (encoder * 4 * ratio);
    sp_ = (2 * PI) / (ratio * 60);
    cp_ = (60 * ratio) / (2 * PI);
    motor_controller = new MotorController(controller_port);
    motor_controller->connect();
    uart_display = new UARTDisplay(display_port);
    uart_display->connect();
    uart_display->writeDisplay("SPG(3);\r\n");
    ros::Duration(1).sleep();

    left_encoder_speed = 0;
    right_encoder_speed = 0;

    left_encoder_counts = 0;
    right_encoder_counts = 0;
    last_left_encoder_counts = 0;
    last_right_encoder_counts = 0;
    init_left_encoder_counts = 0;
    init_right_encoder_counts = 0;
    read_left_encoder_counts = 0;
    read_right_encoder_counts = 0;

    motor_status_ = 1;

    joint_name_.resize(4);
    joint_position_.resize(4);
    joint_velocity_.resize(4);
    joint_effort_.resize(4);
    joint_velocity_command_.resize(4);

    joint_name_[0] = "front_left_wheel_joint";
    joint_name_[1] = "rear_left_wheel_joint";
    joint_name_[2] = "front_right_wheel_joint";
    joint_name_[3] = "rear_right_wheel_joint";

    for (unsigned int i = 0; i < joint_name_.size(); i++)
    {
      js_interface_.registerHandle(JointStateHandle(joint_name_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]));
      vj_interface_.registerHandle(JointHandle(js_interface_.getHandle(joint_name_[i]), &joint_velocity_command_[i]));
    }

    registerInterface(&js_interface_);
    registerInterface(&vj_interface_);

    updater.setHardwareID("Bulldog");
    updater.add(robot_status_task);
    updater.add(battery_status_task);
    updater.add(motor_status_task);
    updater.add(controller_status_task);
  }
  
  void BulldogHW::motorCallback(const std_msgs::Int32::ConstPtr& msg)
  {
      motor_status_ = motor_controller->enableDevice(msg->data);
  }
  

  void BulldogHW::read()
  {
    int *mototr_data = motor_controller->readCountAndSpeed();
    left_encoder_speed = mototr_data[2];
    right_encoder_speed = mototr_data[3];
    if (left_encoder_speed != -1 && right_encoder_speed != -1)
    {
      joint_velocity_[0] = left_encoder_speed * sp_;
      joint_velocity_[1] = joint_velocity_[0];
      joint_velocity_[2] = -right_encoder_speed * sp_;
      joint_velocity_[3] = joint_velocity_[2];
    }
    read_left_encoder_counts = mototr_data[0];
    read_right_encoder_counts = mototr_data[1];
    if (read_left_encoder_counts == 0 && read_right_encoder_counts == 0)
    {
      init_left_encoder_counts = last_left_encoder_counts;
      init_right_encoder_counts = last_right_encoder_counts;
    }
    left_encoder_counts = read_left_encoder_counts + init_left_encoder_counts;
    right_encoder_counts = read_right_encoder_counts + init_right_encoder_counts;
    joint_position_[0] = left_encoder_counts * pp_;
    joint_position_[1] = joint_position_[0];
    joint_position_[2] = -right_encoder_counts * pp_;
    joint_position_[3] = joint_position_[2];
    last_left_encoder_counts = left_encoder_counts;
    last_right_encoder_counts = right_encoder_counts;
  }

  void BulldogHW::write()
  {
    motor_controller->setMotorSpeed(joint_velocity_command_[0] * cp_, -joint_velocity_command_[2] * cp_);
  }

}
