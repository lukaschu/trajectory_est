# trajectory_est
Estimating a trajectory with a Kalman filter given the current state 

Tis repo is supposed to run along the optitrack_ros2 package. Essentially it subscribes to messages sent from the optitrack system, estimates the state 
space using a kalman filter and subsequently, publishes future predicted states. Following params can be chosen:

    updating_period: 0.5 (in seconds, determines the update by the KF)
    Nr_of_obstacles: 1 
    prediction_steps: 10 (how many future steps should be predicted --> in time: prediction_steps * updating period
    low_pass_order: 5 (order of lowpass filter that is used for the velocity/angular-velocity estimation
    use_KF: true (false: we take onyl the observation and ignore the model)
    
    PARAMS FOR KF
    model_noise_pos: 0.01
    model_noise_vel: 0.25
    model_noise_quat: 0.01
    model_noise_omega: 0.25

    meas_noise_pos: 0.001
    meas_noise_vel: 0.1
    meas_noise_quat: 0.001
    meas_noise_omega: 0.1

    init_noise_pos: 0.001
    init_noise_vel: 0.75
    init_noise_quat: 0.001
    init_noise_omega: 0.75

    
To establish a connection between the Optitrack system and the system that runs this repo, the firewall on the windows computer shoudl be 
swithced off. Furthermore one has to give both the optitrack computer and the computer that runs the systema a static IP adress (plug in link)
In Notive, go to edit->settings->streaming. Now choose the local Ip adress as local-inference and Unicast as Transmission type. 

DESCRIPTION
1. Prerequisities:
   First clone define a workspace and install all relevnt packages (three in total)
   ```
   mkdir optitrack_ws
   cd optitrack_ws
   mkdir src && cd src
   git clone git@github.com:lukaschu/trajectory_est.git
   git clone git@github.com:lukaschu/optitrack_ros2.git
   git clone (custom_message)
   cd ..
   rosdep install --from-paths src --ignore-src -r -y
   ```
2. Adjust the IP adress
   Make sure the firewall of the microsoft computer running the motion tracker is switched off
   Now give define a static IP adress for each computer
   Now go to: edit->settings->streaming, and choose the local IP adress at "Local Interface", choose unicast as "Transmission Type"
   Furthermore in the mocap4r2_optitrack_driver package go to mocap4r2_optitrack_driver/config/mocap4r2_optitrack_driver_params.yaml and change the server adress as well as the local adress to the ones you gave both systems.
3. Build the workspace
   ```
   cd optitrack_ws
   colcon build
   ```
   We also need to give the manual path to the NAT_NET executable. You can add the line to the bashrc file if you want (adjust PATH_TO_WORKSPACE below). Otherwise
   you need to do this everytime you run in a new terminal.
   ```
   export LD_LIBRARY_PATH=PATH_TO_WORKSPACE/opti_ws/src/optitrack_ros2/mocap4ros2_optitrack/mocap4r2_optitrack_driver/NatNetSDK/lib//:$LD_LIBRARY_PATH
   ```
5. Run the optitrack driver:
   We can now launch the driver
   ```
   ros2 launch mocap4r2_optitrack_driver optitrack2.launch.py
   ros2 lifecycle set /mocap4r2_optitrack_driver_node activate
   ```
6. Run the Kalman Filter
   ```
    ros2 launch trajectory_est trajectory_est_launch.py
   ```
    There will now be a topic called: /predicted_trajectories which contains the predicted trajectories of the defined objects.
   Note that for each object a sepperate message is published to the topic. The object_id variable which is contained in the message serves
   as a unique identifier.
   
   
   
   
   
