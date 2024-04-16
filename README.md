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
