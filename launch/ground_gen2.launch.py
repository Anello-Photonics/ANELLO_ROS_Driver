"""ROS 2 launch entry point; same arguments and topics as ROS 1."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from pathlib import Path
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ground_gen2.ntrip import DEFAULTS as NTRIP_DEFAULTS


def generate_launch_description():
    defaults = dict(transport='serial', port='/dev/ttyUSB0', baudrate=921600,
                    bind_address='0.0.0.0', udp_port=19550, multicast_group='',
                    multicast_interface='0.0.0.0', source_ip='', nmea_host='', nmea_port=19551, frame_id='ground_gen2')
    defaults.update(NTRIP_DEFAULTS)
    parameters = {name: ParameterValue(LaunchConfiguration(name), value_type=type(value))
                  for name, value in defaults.items()}
    arguments = [DeclareLaunchArgument(name, default_value=str(value).lower() if isinstance(value, bool) else str(value))
                 for name, value in defaults.items()]
    def launch_node(context):
        node_parameters = [parameters]
        config_file = LaunchConfiguration('config_file').perform(context)
        if config_file:
            path = Path(config_file).expanduser()
            if not path.is_file():
                raise ValueError('Configuration file does not exist: ' + str(path))
            # Last parameter source wins, matching the ROS 1 launch file.
            node_parameters.append(str(path))
        return [Node(package='anello_ros_driver', executable='ground_gen2_ros2.py',
                     name='ground_gen2', output='screen', parameters=node_parameters)]

    return LaunchDescription([
        *arguments,
        DeclareLaunchArgument('config_file', default_value='',
                              description='Optional YAML parameter file; file values take precedence'),
        OpaqueFunction(function=launch_node),
    ])
