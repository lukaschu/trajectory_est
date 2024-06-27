#ifndef TRAJ_ESTIMATOR_NODE
#define TRAJ_ESTIMATOR_NODE

#include <chrono>
#include <memory>
#include <string>
#include <map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "mocap4r2_msgs/msg/rigid_bodies.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "trajectory_est/kalman_filter.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include "custom_messages/msg/predictedtrajectory.hpp"

using namespace std::chrono_literals;

class TrajectoryEstimator : public rclcpp::Node
{
public:
    TrajectoryEstimator();
    void Addobstacles();
    void body_info_callback(const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg);
    void set(const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg);
    void kalmanFilterCallback();

private:
    std::vector<KalmanFilter> dynamic_obstacles; // vector with KF and estimation functionality
    rclcpp::Publisher<custom_messages::msg::Predictedtrajectory>::SharedPtr prediction_publisher_; // Publisher
    rclcpp::Subscription<mocap4r2_msgs::msg::RigidBodies>::SharedPtr markers_subscription_rb_; // Subscriber
    rclcpp::TimerBase::SharedPtr kalman_filter_timer_; // Subscriber(timer)
    bool tracking_initialized = false;
    double updating_period; // period after which KF is recomputed
    unsigned int Nr_of_obstacles; // parameter defined in yaml file

    // For tf 
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
}; 

#endif