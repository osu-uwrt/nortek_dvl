from socket import gethostbyname
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from launch.substitutions import LaunchConfiguration as LC
from launch.actions import OpaqueFunction

def eval_hostname(context, *args, **kwargs):
    hostName = LC('address').perform(context)
    
    #do lookup
    ip_address_here = hostName
    if not "." in hostName:
        try:
            ip_address_here = str(gethostbyname(hostName))

        except Exception as e:
            print(f"Failed to look up hostname {hostName}. error: {e}")
            exit(-1)

    
    node = launch_ros.actions.Node(
            package='nortek_dvl',
            executable='dvl',
            name='dvl',
            respawn=True,
            output='screen',
            
            # use the parameters on the node
            parameters = [
                {'address': ip_address_here},
                {'port': int(LC('port').perform(context))},
                {'timeout': int(LC('timeout').perform(context))},
                {'max_connect_time': int(LC('max_connect_time').perform(context))},
                {'min_connect_time': int(LC('min_connect_time').perform(context))},
                {'frame_id': LC('frame_id').perform(context)},
                {'sonar_frame_id': LC('sonar_frame_id').perform(context)},
                {'use_enu': bool(LC('use_enu').perform(context))},
            ]
        )
    
    return [node]
    

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
        OpaqueFunction(function=eval_hostname),   
        
    ])