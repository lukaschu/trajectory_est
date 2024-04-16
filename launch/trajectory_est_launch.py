import launch
import launch_ros.actions
import os
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_path = FindPackageShare(package='trajectory_est').find('trajectory_est') 
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='trajectory_est',
            executable='trajectory_est',
            name='trajectory_est',
            parameters=[os.path.join(pkg_path, 'config/ros_node_params.yaml')] 
        )
    ])