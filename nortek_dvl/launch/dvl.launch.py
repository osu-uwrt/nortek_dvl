import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        # declare the launch args to read for this file
        launch.actions.DeclareLaunchArgument(
            'address',
            default_value='uwrt-dvl',
            description='Address of DVL'),
        launch.actions.DeclareLaunchArgument(
            'port',
            default_value='9004',
            description='Port of DVL'),
        launch.actions.DeclareLaunchArgument(
            'timeout',
            default_value='500',
            description='maxiumm time in miliseconds for a packet recieve from the DVL during normal operation'),
        launch.actions.DeclareLaunchArgument(
            'max_connect_time',
            default_value='100',
            description='Maximum time elapsed in seconds before connection attempt on startup is aborted'),
        launch.actions.DeclareLaunchArgument(
            'min_connect_time',
            default_value='5',
            description='Minimum time elapsed in seconds before connection attempt on startup is attempted'),
        launch.actions.DeclareLaunchArgument(
            'frame_id',
            default_value='dvl_link',
            description='TF frame in message headerss'),
        launch.actions.DeclareLaunchArgument(
            'sonar_frame_id',
            default_value='dvl_sonar%d_link',
            description='TF frame in message headers'),
        launch.actions.DeclareLaunchArgument(
            'use_enu',
            default_value='true',
            description='Whether to report twist in ENU frame'),

        # create the nodes    
        launch_ros.actions.Node(
            package='nortek_dvl',
            executable='dvl',
            name='dvl',
            respawn=True,
            output='screen',
            
            # use the parameters on the node
            parameters = [
                {'address': launch.substitutions.LaunchConfiguration('address')},
                {'port': launch.substitutions.LaunchConfiguration('port')},
                {'timeout': launch.substitutions.LaunchConfiguration('timeout')},
                {'max_connect_time': launch.substitutions.LaunchConfiguration('max_connect_time')},
                {'min_connect_time': launch.substitutions.LaunchConfiguration('min_connect_time')},
                {'frame_id': launch.substitutions.LaunchConfiguration('frame_id')},
                {'sonar_frame_id': launch.substitutions.LaunchConfiguration('sonar_frame_id')},
                {'use_enu': launch.substitutions.LaunchConfiguration('use_enu')},
            ]
        )
    ])