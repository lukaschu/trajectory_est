#include "trajectory_est/kalman_filter.hpp"
#include <iostream>

/*
This class contains the functionality to estimate the state using an extended Kalman Filter
and to push the current state forward using only the model.
Lukas Schüepp
*/

KalmanFilter::KalmanFilter()
{    
     // Allocate dynamic space for diff. values
     vel_ptr = new Eigen::Vector3d[low_pass_order];
     quat_der_ptr = new Eigen::Vector4d[low_pass_order];
}

/*
Initial state is set as soon as object is detected in room
*/
void KalmanFilter::set_initial(const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg, int i)
{
     const auto& rigid_body = msg->rigidbodies[i];
     const auto& pose = rigid_body.pose;

     position  << pose.position.x, pose.position.y, pose.position.z;
     pose_quat.x() = pose.orientation.x;
     pose_quat.y() = pose.orientation.y;
     pose_quat.z() = pose.orientation.z;
     pose_quat.w() = pose.orientation.w;

     // Assume zero velocity
     velocity =  Eigen::Vector3d::Zero();
     omega =  Eigen::Vector3d::Zero();

     // Set initial value for KF
     predicted_state << position, velocity, pose_quat.coeffs(), omega;

     time =  msg->header.stamp.sec + msg->header.stamp.nanosec / pow(10,9);
     
}

/*
Save the observed position and orientation from the tracking system
*/
void KalmanFilter::set_statespace(const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg, int i)
{
     const auto& rigid_body = msg->rigidbodies[i];
     const auto& pose = rigid_body.pose;

     last_position = position;
     last_pose_quat = pose_quat;

     // Recover position from track system 
     position  << pose.position.x, pose.position.y, pose.position.z;
     pose_quat.x() = pose.orientation.x;
     pose_quat.y() = pose.orientation.y;
     pose_quat.z() = pose.orientation.z;
     pose_quat.w() = pose.orientation.w;

     // Save the timestamp
     last_time = time;
     time =  msg->header.stamp.sec + msg->header.stamp.nanosec / pow(10,9);
     double time_diff = time - last_time;
     
     // Approximate velocity and quaternion derivative
     Eigen::Vector3d current_velocity = (position - last_position) / time_diff; 
     Eigen::Vector4d current_derivative_quat = (pose_quat.coeffs() - last_pose_quat.coeffs()) / time_diff;

     vel_ptr[counter] = current_velocity;
     quat_der_ptr[counter] = current_derivative_quat;

     // increase counter
     counter += 1;
     counter %= low_pass_order;
}

/*
All relevant values for the system dynamics and the KF are defined
*/
void KalmanFilter::define_dynamics(double updating_period)
{   
     
     // Define missing values, (Position/ velocity dynamics is linear)
     for(unsigned int i = 0; i < 3; ++i)
     {     
          A_pos_vel(i, 3 + i) = updating_period;
     }

     discret_time = updating_period;

     // Define model covariance, noise covariance and covariance at initial step (For KF)
     for(unsigned int i = 0; i < 3; ++i)
     {     
          posterior_cov(i,i) = init_noise_pos;
          posterior_cov(i + 3, i + 3) = init_noise_vel;
          posterior_cov(i + 6, i + 6) = init_noise_quat;
          posterior_cov(i + 10, i + 10) = init_noise_omega;

          Q(i,i) = model_noise_pos;
          Q(i + 3, i + 3) = model_noise_vel;
          Q(i + 6, i + 6) = model_noise_quat;
          Q(i + 10, i + 10) = model_noise_omega;

          R(i,i) = meas_noise_pos;
          R(i + 3, i + 3) = meas_noise_vel;
          R(i + 6, i + 6) = meas_noise_quat;
          R(i + 10, i + 10) = meas_noise_omega;
     }

     // manuall b.c 4 quaternions
     posterior_cov(9, 9) = init_noise_quat; 
     Q(9, 9) = model_noise_quat;
     R(9, 9) = meas_noise_quat;
}

/*
Put last few observation through a lowpass filter (for velocity and omega estimation)
*/
void KalmanFilter::LowPassFilter()
{    
     // Sum up the last few observations and average them (simple low pass filter)
     velocity = std::accumulate(vel_ptr, vel_ptr + low_pass_order, Eigen::Vector3d(0.0, 0.0, 0.0)) / low_pass_order;
     quat_der = std::accumulate(quat_der_ptr, quat_der_ptr + low_pass_order, Eigen::Vector4d(0.0, 0.0, 0.0, 0.0)) / low_pass_order;

     Eigen::Matrix<double, 3, 4> Transition_matrix = quat_derivative_to_omega(pose_quat);
     omega = 2 * Transition_matrix * quat_der; // transform derivative of quaternion to angular velocity

     // Memory management
     delete [] vel_ptr;
     delete [] quat_der_ptr;

     // Allocate new space
     vel_ptr = new Eigen::Vector3d[low_pass_order];
     quat_der_ptr = new Eigen::Vector4d[low_pass_order];

     counter = 0;
}

// Calculates omega given the current quaternions and the corresponding derivative
Eigen::Matrix<double, 3, 4> KalmanFilter::quat_derivative_to_omega(Eigen::Quaterniond quat)
{    
     Eigen::Matrix<double, 3, 4> Transition;
     Transition <<  quat.w(), -quat.z(), quat.y(), -quat.x(),
                    quat.z(), quat.w(), -quat.x(), -quat.y(),
                    -quat.y(), -quat.x(), quat.w(), -quat.z();

     return Transition;
}

/*
pushed the current state one step forward according to the (nonlinear) model
*/
Eigen::Matrix<double, 13, 1> KalmanFilter::ModelStep(Eigen::Matrix<double, 13, 1> current_state)
{    
     // Push forward position and velocity dynamics
     Eigen::Matrix<double, 6, 1> next_pos_vel = A_pos_vel * current_state.block<6,1>(0,0);

     Eigen::Quaterniond current_quat;
     current_quat.x() = current_state(6,0);
     current_quat.y() = current_state(7,0);
     current_quat.z() = current_state(8,0);
     current_quat.w() = current_state(9,0);

     // Push forward quaternion dynamics
     Eigen::Vector3d current_omega = current_state.block<3,1>(10,0);
     Eigen::Matrix<double, 4, 1> next_quat = current_state.block<4,1>(6,0) + 
          0.5 * discret_time * quat_derivative_to_omega(current_quat).transpose() * current_omega;
     
     // Next quaternion needs to be normalized!!
     next_quat.normalize();

     // Return full next state
     Eigen::Matrix<double, 13, 1> next_state;
     next_state << next_pos_vel, next_quat, omega;

     return next_state;
}

/*
updates the estimated state and covariance (heart of the KF)
*/
Eigen::Matrix<double,13,1> KalmanFilter::Run_KF()
{    
     Eigen::Matrix<double,13,1> Observed_state;
     Observed_state << position, velocity, pose_quat.coeffs(), omega;

     // Apply Model update first with previous estimation (predicted_state)
     Eigen::Matrix<double,13,1> next_model_state = ModelStep(predicted_state);

     // Linearization of state matrix
     Eigen::Matrix<double, 13, 13> A_linearized = Eigen::Matrix<double, 13, 13>::Identity(); 
     A_linearized.block<6,6>(0,0) = A_pos_vel;

     // This is linearization of: quat+ = f(quat, omega), at last timestep
     double t = discret_time;
     A_linearized.block<4,7>(6,6) << 1, -0.5*t*predicted_state(12,0), 0.5*t*predicted_state(11,0), 0.5*t*predicted_state(10,0), -0.5*t*predicted_state(6,0),
                    -0.5*t*predicted_state(7,0), -0.5*t*predicted_state(8,0),
                    0.5*t*predicted_state(12,0), 1, -0.5*t*predicted_state(10,0), 0.5*t*predicted_state(11,0), 0.5*t*predicted_state(9,0),
                    0.5*t*predicted_state(8,0), -0.5*t*predicted_state(7,0),
                    -0.5*t*predicted_state(11,0), 0.5*t*predicted_state(10,0), 1, 0.5*t*predicted_state(12,0), -0.5*t*predicted_state(8,0),
                    0.5*t*predicted_state(9,0), 0.5*t*predicted_state(6,0),
                    -0.5*t*predicted_state(10,0), -0.5*t*predicted_state(11,0), -0.5*t*predicted_state(12,0), 1, 0.5*t*predicted_state(7,0),
                    -0.5*t*predicted_state(6,0), 0.5*t*predicted_state(9,0);

     // Calculate model covariance
     Eigen::Matrix<double, 13, 13> P_k = A_linearized * posterior_cov * A_linearized.transpose() + Q;

     // Calculate Kalman gain
     kalman_gain = P_k * H.transpose() * (H * P_k * H.transpose() + R).inverse();

     // Update posterior covariance
     posterior_cov = (Eigen::Matrix<double, 13, 13>::Identity() - kalman_gain * H) * P_k;

     // Finally we define the expected state
     Eigen::Matrix<double, 13, 1> Expected_state = next_model_state + kalman_gain * (Observed_state - H * next_model_state);

     return Expected_state;
}

/*
This function gets called by the main node. It runs the low pass filter
and the Kalman filter (if needed)
*/
std::vector<Eigen::Matrix<double,13,1>> KalmanFilter::predict_trajectory()
{   
     LowPassFilter();
     if (use_KF)
     {    
          predicted_state = Run_KF();
     }
     else
     {    // take observed states directly
          predicted_state << position, velocity, pose_quat.coeffs(), omega;
     }

     std::vector<Eigen::Matrix<double,13,1>> future_states(prediction_steps);
     future_states[0] = predicted_state;

     // Push the system forward with the nonlinear system dyanmics
     for(unsigned int i = 1; i < future_states.size(); ++i)
     {
          future_states[i] = ModelStep(future_states[i-1]);
     }
     return future_states;
}