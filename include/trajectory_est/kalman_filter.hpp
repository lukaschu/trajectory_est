#ifndef KALMAN_FILTER
#define KALMAN_FILTER

#include <chrono>
#include <memory>
#include <string>
#include <numeric>
#include <vector>
#include <map>
#include "mocap4r2_msgs/msg/rigid_bodies.hpp"
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include "ament_index_cpp/get_package_share_directory.hpp"

class KalmanFilter
{
public:
    KalmanFilter();
    void set_statespace(const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg, int i);
    void set_initial(const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg, int i);
    void define_dynamics(double updating_period);
    void LowPassFilter();
    Eigen::Matrix<double, 3, 4> quat_derivative_to_omega(Eigen::Quaterniond quat);
    Eigen::Matrix<double, 13, 1> ModelStep(Eigen::Matrix<double, 13, 1> current_state); 
    Eigen::Matrix<double,13,1> Run_KF();
    std::vector<Eigen::Matrix<double,13,1>> predict_trajectory();

private:
    // state space information 
    Eigen::Vector3d position;
    Eigen::Vector3d last_position;
    Eigen::Quaterniond pose_quat; 
    Eigen::Quaterniond last_pose_quat; 
    Eigen::Vector3d velocity;
    Eigen::Vector4d quat_der;
    Eigen::Vector3d omega;
    double time;
    double last_time;
    unsigned int counter = 0;

    // values for low pass filter and velocity estimation
    Eigen::Vector3d *vel_ptr; 
    Eigen::Vector4d *quat_der_ptr;

    // Dynamics
    Eigen::Matrix<double, 6, 6> A_pos_vel = Eigen::Matrix<double, 6, 6>::Identity();
    Eigen::Matrix<double, 13, 13> H = Eigen::Matrix<double, 13, 13>::Identity(); // observation model
    Eigen::Matrix<double, 13, 13> Q = Eigen::Matrix<double, 13, 13>::Zero(); // Model noise
    Eigen::Matrix<double, 13, 13> R = Eigen::Matrix<double, 13, 13>::Zero(); // Measurement noise
    double discret_time; // corresponds to the KF updating period

    // Probabilistic quantities needed in Kalman filter
    Eigen::Matrix<double, 13, 1> predicted_state;
    Eigen::Matrix<double, 13, 13> posterior_cov = Eigen::Matrix<double, 13, 13>::Zero(); 
    Eigen::Matrix<double, 13, 13> kalman_gain;
    
    //YAML::Node config = YAML::LoadFile("/home/lukas_schueepp/workfolder/opti_ws/src/trajectory_est/config/kalman_filter.yaml");

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("trajectory_est");
    std::string yaml_path = package_share_directory + "/src/trajectory_est/config/kalman_filter.yaml";
    YAML::Node config = YAML::LoadFile(yaml_path);
    
     // Load params 
     float model_noise_pos = config["model_noise_pos"].as<float>();
     float model_noise_vel = config["model_noise_vel"].as<float>();
     float model_noise_quat = config["model_noise_quat"].as<float>();
     float model_noise_omega = config["model_noise_omega"].as<float>();

     float meas_noise_pos = config["meas_noise_pos"].as<float>();
     float meas_noise_vel = config["meas_noise_vel"].as<float>();
     float meas_noise_quat = config["meas_noise_quat"].as<float>();
     float meas_noise_omega = config["meas_noise_omega"].as<float>();

     float init_noise_pos = config["init_noise_pos"].as<float>();
     float init_noise_vel = config["init_noise_vel"].as<float>();
     float init_noise_quat = config["init_noise_quat"].as<float>();
     float init_noise_omega = config["init_noise_omega"].as<float>();

     int prediction_steps = config["prediction_steps"].as<int>();
     int low_pass_order = config["low_pass_order"].as<int>();
     bool use_KF = config["use_KF"].as<bool>();
};

#endif