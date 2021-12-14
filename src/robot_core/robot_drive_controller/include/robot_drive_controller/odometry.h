#pragma once
#include <nav_msgs/Odometry.h>
#include <ros/time.h>

namespace robot_drive_controller {
class Odometry {
 public:
  /**
   * @brief Set parameters related to wheel
   * @param wheel_radius: Wheel Radius
   * @param wheel_separation: Separation between wheels
   **/
  void setParameters(const double wheel_radius, const double wheel_separation_);
  /**
   * @brief Compute Orientation and Position in XY Plane
   * @param right_wheel_pos: Right Wheel Position
   * @param left_wheel_pos: Left Wheel Position
   **/
  void computePositionAndOrientation(const double right_wheel_pos,
                                     const double left_wheel_pos,
                                     const ros::Time& time);

  nav_msgs::Odometry getOdomMessage(const std::string base_frame,
                                    const std::string odom_frame,
                                    const ros::Time& time);

 private:
  // Parameters related wheel
  double wheel_radius_;
  double wheel_separation_;
  int skip = 0;
  // Structs
  struct Odom {
    // Pose
    double x;
    double y;
    // Velocities
    double v;
    double w;
    // Orientation
    double theta;
    Odom() : x{0.0}, y{0.0}, theta{0.0}, v{0.0}, w{0.0} {}
  };
  // Variables
  Odom odom_;
  Odom old_odom_;
  // Temporal variables
  double right_wheel_pos_prev;
  double left_wheel_pos_prev;
  ros::Time timestamp_prev_;
};
}  // namespace robot_drive_controller