#include <robot_drive_controller/odometry.h>
#include <ros/console.h>
#include <ros/publisher.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Float64.h>

namespace robot_drive_controller {
// Set wheel Parameters
void Odometry::setParameters(const double wheel_radius,
                             const double wheel_separation) {
  wheel_radius_ = wheel_radius;
  wheel_separation_ = wheel_separation;
}
// Calculate Position and Orientation
void Odometry::computePositionAndOrientation(const double left_wheel_pos,
                                             const double right_wheel_pos,
                                             const ros::Time& time) {
  /** Calculate distance between before and now
   * Variables:
   * lt : Differential Distance from wheel center
   * lr : Differential Distance from left wheel
   * ll : Differential Distance from left wheel
   * Equations:
   * lt = (lr+ll)/2
   **/

  // Differential time
  const double dt = (time - timestamp_prev_).toSec();

  // Differential Linear
  const double lr = (right_wheel_pos - right_wheel_pos_prev) * wheel_radius_ ;
  const double ll = (left_wheel_pos - left_wheel_pos_prev) * wheel_radius_ ;
  const double lt = (ll + lr) / 2;
  // Differential Angle
  const double th = (ll - lr) / wheel_separation_;

  // Integration with Runge-Kutta 2
  const double old_theta = odom_.theta;
  odom_.theta += (th+odom_.w*dt)/2;  
  odom_.x += ( lt*cos(odom_.theta) + odom_.v*cos(old_theta)*dt )/2;
  odom_.y += ( lt*sin(odom_.theta) + odom_.v*sin(old_theta)*dt )/2;
  odom_.v = lt/dt;
  odom_.w = th/dt;

  // Save Previous Positions
  right_wheel_pos_prev = right_wheel_pos;
  left_wheel_pos_prev = left_wheel_pos;
  // Save Previouos Time
  timestamp_prev_ = time;
}

nav_msgs::Odometry Odometry::getOdomMessage(const std::string base_frame,
                                            const std::string odom_frame,
                                            const ros::Time& time) {
  nav_msgs::Odometry odom;
  odom.header.frame_id = odom_frame;
  odom.header.stamp = time;
  odom.child_frame_id = base_frame;
  odom.pose.pose.position.x = odom_.x;
  odom.pose.pose.position.y = odom_.y;
  odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(odom_.theta);
  odom.twist.twist.linear.x = odom_.v;
  odom.twist.twist.angular.z = odom_.w;
  return odom;
}

}  // namespace robot_drive_controller