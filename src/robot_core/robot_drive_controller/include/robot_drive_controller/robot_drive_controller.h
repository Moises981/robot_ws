#pragma once
#include <controller_interface/controller.h>
#include <geometry_msgs/Twist.h>
#include <hardware_interface/joint_command_interface.h>
#include <robot_drive_controller/odometry.h>
#include <nav_msgs/Odometry.h>

namespace robot_drive_controller {
class RobotDriveController : public controller_interface::Controller<
                                 hardware_interface::VelocityJointInterface> {
 public:
  // Empty Constructor
  RobotDriveController();

  /**
   * @brief Init Controller
   * @param *hw: Velocity Controllers
   * @param &root_nh: Main Node
   * @param &controller_nh: Controller Node
   * @return Controller Status
   **/
  bool init(hardware_interface::VelocityJointInterface *hw,
            ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;

  /**
   * @brief Update Controller
   * @param &time: Current Ros Time
   * @param &period: Duration of update
   **/
  void update(const ros::Time &time, const ros::Duration &period) override;

 private:
  // Wheel controllers
  hardware_interface::JointHandle right_wheel_;
  hardware_interface::JointHandle left_wheel_;
  // Wheel Parameters
  double wheel_radius_;
  double wheel_separation_;
  // Optional Parameters
  double cmd_vel_timeout_;
  // Topics related
  ros::Subscriber sub_cmdVel_;
  ros::Publisher pub_odom_;
  // Command Struct
  struct Command {
    double lin;
    double ang;
    ros::Time timestamp;
    Command() : lin{0.0}, ang{0.0}, timestamp{0.0} {}
  };
  // Command Variable
  Command cmd_vel_;
  // Odometry related
  Odometry odometry_;

 private:
  /**
   * @brief Velocities Command Topic Subscriber
   * @param msg: Rosmsg Twist
   **/
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr msg);
  /**
   * @brief Get Wheel Controller From Config File
   * @param param: Parameter Name in Yaml
   * @param &controller: Joint Controller
   * @param &controller_nh: Controller Node in Config File
   * @param *hw: Robot Hardware
   **/
  bool getWheel(const std::string param,
                hardware_interface::JointHandle &controller,
                ros::NodeHandle &controller_nh,
                hardware_interface::VelocityJointInterface &hw);
};
}  // namespace robot_drive_controller