#include "trajectory_est/traj_estimator_node.hpp"
using namespace std::chrono_literals;
using std::placeholders::_1;

// name of node is estimator
TrajectoryEstimator::TrajectoryEstimator() 
: Node("estimator")
{   
    // Publishes the basic functionality for plotting the prediction (TF)
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    //Define subscriber
    markers_subscription_rb_ = this->create_subscription<mocap4r2_msgs::msg::RigidBodies>(
    "rigid_bodies", 1000, std::bind(&TrajectoryEstimator::body_info_callback, this, _1));

    // Define publisher
    prediction_publisher_ = this->create_publisher<custom_messages::msg::Predictedtrajectory>("predicted_trajectories", 1000);

    // parameter definitions (see ros_node_params.yaml)
    this->declare_parameter("updating_period", rclcpp::PARAMETER_DOUBLE);
    get_parameter("updating_period", updating_period); 

    this->declare_parameter("Nr_of_obstacles", rclcpp::PARAMETER_INTEGER);
    get_parameter("Nr_of_obstacles", Nr_of_obstacles);

    // Adds obstacles according to the parameter defined above
    Addobstacles();
}

// Define function that adds obstacles
void TrajectoryEstimator::Addobstacles()
{   
    for(unsigned int i=0; i < Nr_of_obstacles; ++i)
    { 
        dynamic_obstacles.push_back(KalmanFilter());
        //Note: All obstacles have the same system transition. So this loop calculates the same 
        dynamic_obstacles[dynamic_obstacles.size() - 1].define_dynamics(updating_period);
    }
}

// sets the current position, pose (quaternion), velocity, 
void TrajectoryEstimator::set(const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg)
{   
    if(tracking_initialized) // All objects initilaized
    {   
        for(unsigned int i=0; i < dynamic_obstacles.size(); ++i)
        {
            dynamic_obstacles[i].set_statespace(msg, i);
        }
    }

    else if(msg->rigidbodies.size() == Nr_of_obstacles) // Means we can initialize the kalman filter
    {   
        tracking_initialized = true;
        for(unsigned int i = 0; i < dynamic_obstacles.size(); ++i)
        {
            dynamic_obstacles[i].set_initial(msg, i);
        }
        // start the kalman filter
        // After "updating_period" time, the callback function will be called for the first time!
        std::cerr<< "KALMAN FILTER START" <<std::endl;
        kalman_filter_timer_ = this->create_wall_timer(std::chrono::duration<double>(updating_period), 
            std::bind(&TrajectoryEstimator::kalmanFilterCallback, this));
    }
}

// Define callback functions
void TrajectoryEstimator::body_info_callback(const mocap4r2_msgs::msg::RigidBodies::SharedPtr msg) 
{   
    set(msg); 
}

void TrajectoryEstimator::kalmanFilterCallback()
{   
    // Call Kalman filter in here and after that, publish the calculated trajectory (TODO).
    for(unsigned int i=0; i< dynamic_obstacles.size(); ++i)
    {   
        auto start = std::chrono::system_clock::now(); // Extract the current time
        auto time_to_start = start.time_since_epoch(); // epoch was started soemwhere in 1970

        auto seconds_to_start = std::chrono::duration_cast<std::chrono::seconds>(time_to_start);
        auto nanoseconds_to_start = std::chrono::duration_cast<std::chrono::nanoseconds>(time_to_start);
        //std::chrono::duration<double> additional_duration_per_step = std::chrono::duration<double>(updating_period);
        
        // Contains the predicted trajectory
        std::vector<Eigen::Matrix<double,13,1>> Predicted_Trajectory = dynamic_obstacles[i].predict_trajectory();

        // we plug the predicted trajectory into the custom message type
        auto message = custom_messages::msg::Predictedtrajectory();
        // give the message the obstacle id
        message.obstacle_id = i; 

        // We get the predicted data 
        auto sub_message = geometry_msgs::msg::PoseStamped();
        
        for(unsigned int j=0; j < Predicted_Trajectory.size(); ++j) // j corresponds to the step forward
        {   
            // extract current time at prediction

            sub_message.header.stamp.sec = seconds_to_start.count(); //updating_period * std::pow(10,3) * j; // how far in the future do we predict
            sub_message.header.stamp.nanosec = nanoseconds_to_start.count();
            sub_message.header.frame_id = std::to_string(j); // what obstacle

            sub_message.pose.position.x = Predicted_Trajectory[j](0,0);
            sub_message.pose.position.y = Predicted_Trajectory[j](1,0);
            sub_message.pose.position.z = Predicted_Trajectory[j](2,0);
    
            sub_message.pose.orientation.x = Predicted_Trajectory[j](6,0);
            sub_message.pose.orientation.y = Predicted_Trajectory[j](7,0);
            sub_message.pose.orientation.w = Predicted_Trajectory[j](9,0);

            message.trajectory.push_back(sub_message);
        }

        prediction_publisher_->publish(message); 

        // Is only needed for plotting (TF)
        // Currently the 8th prediction is plotted
        //unsigned int last = Predicted_Trajectory.size();

    //     geometry_msgs::msg::TransformStamped t;

    //     t.header.stamp = this->get_clock()->now();
    //     t.header.frame_id = "world";
    //     t.child_frame_id = "future robot";

    //     t.transform.translation.x = Predicted_Trajectory[8](0,0);
    //     t.transform.translation.y = Predicted_Trajectory[8](1,0);
    //     t.transform.translation.z = Predicted_Trajectory[8](2,0);
    
    //     t.transform.rotation.x = Predicted_Trajectory[8](6,0);
    //     t.transform.rotation.y = Predicted_Trajectory[8](7,0);
    //     t.transform.rotation.z = Predicted_Trajectory[8](8,0);
    //     t.transform.rotation.w = Predicted_Trajectory[8](9,0);

    //     tf_broadcaster_->sendTransform(t);

    //     t.header.stamp = this->get_clock()->now();
    //     t.header.frame_id = "world";
    //     t.child_frame_id = "robot";

    //     t.transform.translation.x = Predicted_Trajectory[0](0,0);
    //     t.transform.translation.y = Predicted_Trajectory[0](1,0);
    //     t.transform.translation.z = Predicted_Trajectory[0](2,0);
    
    //     t.transform.rotation.x = Predicted_Trajectory[0](6,0);
    //     t.transform.rotation.y = Predicted_Trajectory[0](7,0);
    //     t.transform.rotation.z = Predicted_Trajectory[0](8,0);
    //     t.transform.rotation.w = Predicted_Trajectory[0](9,0);

    //     tf_broadcaster_->sendTransform(t);
    }
}