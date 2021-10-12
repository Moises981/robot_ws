#include <pluginlib/class_list_macros.h>
#include <robot_drive_controller/robot_drive_controller.h>

namespace robot_drive_controller {
// Constructor
RobotDriveController::RobotDriveController() : cmd_vel_timeout_{1.0} {}

// Init Controller
bool RobotDriveController::init(hardware_interface::VelocityJointInterface* hw,
                                ros::NodeHandle& root_nh,
                                ros::NodeHandle& controller_nh) {
  // Get Wheel Controllers
  if (!(getWheel("right_wheel", right_wheel_, controller_nh, *hw) &&
        getWheel("left_wheel", left_wheel_, controller_nh, *hw))) {
    return false;
  }

  // Get Wheel Basic Parameters
  if (!(controller_nh.getParam("wheel_radius", wheel_radius_) &&
        controller_nh.getParam("wheel_separation", wheel_separation_))) {
    ROS_WARN_STREAM("Wheel Parameters are not specified in config file");
    return false;
  }
  // Get Optional Parameters
  controller_nh.getParam("cmd_vel_timeout", cmd_vel_timeout_);

  // Set odometry Wheel Parameters
  odometry_.setParameters(wheel_radius_, wheel_separation_);

  // Subscribe to cmd_vel Topic
  sub_cmdVel_ = root_nh.subscribe("cmd_vel", 1,
                                  &RobotDriveController::cmdVelCallback, this);

  pub_odom_ = root_nh.advertise<nav_msgs::Odometry>("odom", 1);

  return true;
}

// Update Controller
void RobotDriveController::update(const ros::Time& time,
                                  const ros::Duration& period) {
  // Calculate each wheel velocity
  // Equation: vr|vl = lin ± ang * wheel_separation/2
  const double wr =
      (cmd_vel_.lin + cmd_vel_.ang * wheel_separation_ / 2) / wheel_radius_;
  const double wl =
      (cmd_vel_.lin - cmd_vel_.ang * wheel_separation_ / 2) / wheel_radius_;
  // Set velocities
  right_wheel_.setCommand(wr);
  left_wheel_.setCommand(wl);
  // Check if last command is old
  const double dt = (time - cmd_vel_.timestamp).toSec();
  if (dt > cmd_vel_timeout_) {
    cmd_vel_.lin = 0.0;
    cmd_vel_.ang = 0.0;
  }
  // Get Wheel Positions
  const double right_wheel_pos = right_wheel_.getPosition();
  const double left_wheel_pos = left_wheel_.getPosition();
  // Calculate position and orientation in XY Plane
  odometry_.computePositionAndOrientation(right_wheel_pos, left_wheel_pos,
                                          time);
  // Publish odometry
  pub_odom_.publish(odometry_.getOdomMessage("base_link", "odom", time));
}

// Callback to Set Velocities
void RobotDriveController::cmdVelCallback(
    const geometry_msgs::Twist::ConstPtr msg) {
  cmd_vel_.lin = msg->linear.x;
  cmd_vel_.ang = msg->angular.z;
  cmd_vel_.timestamp = ros::Time::now();
}

// Get Controller
bool RobotDriveController::getWheel(
    const std::string param, hardware_interface::JointHandle& controller,
    ros::NodeHandle& controller_nh,
    hardware_interface::VelocityJointInterface& hw) {
  // Temporal Variable
  std::string wheel_name;
  // Get wheel name
  if (!controller_nh.getParam(param, wheel_name)) {
    ROS_ERROR_STREAM("Parameter " << param << " is not in config file.");
    return false;
  }
  // Get wheel controller
  controller = hw.getHandle(wheel_name);
  // Console
  ROS_INFO_STREAM("Adding controller to: " << wheel_name);
  return true;
}
}  // namespace robot_drive_controller

PLUGINLIB_EXPORT_CLASS(robot_drive_controller::RobotDriveController,
                       controller_interface::ControllerBase);