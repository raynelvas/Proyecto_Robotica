from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
from launch.substitutions import Command, FindExecutable
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Configuración básica
    robot_count = 2  # Número de robots
    urdf_file = os.path.join(
        get_package_share_directory('diffdrive_description'),
        'urdf',
        'my_robot.urdf.xacro'
    )
    rviz_config = os.path.join(
        get_package_share_directory('diffdrive_description'),
        'rviz',
        'multi_robot_view.rviz'
    )

    ld = LaunchDescription()

    for i in range(1, robot_count + 1):
        namespace = f"robot{i}"
        tf_prefix = f"{namespace}/"  # Usar _ en lugar de / para TF

        robot_group = GroupAction([
            PushRosNamespace(namespace),
            
            # Robot State Publisher con parámetros correctos
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                parameters=[{
                    'robot_description': Command([
                        FindExecutable(name='xacro'), ' ', 
                        urdf_file, ' ',
                        'tf_prefix:=', tf_prefix
                    ]),
                    'frame_prefix': tf_prefix  # Mejor práctica ROS 2
                }]
            ),

            # Joint State Publisher (opcional)
            Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                name='joint_state_publisher',
                parameters=[{'source_list': ['joint_states']}]
            )
        ])
        ld.add_action(robot_group)

    # RViz global
    """ld.add_action(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
            parameters=[{'use_sim_time': False}]
        )
    )"""

    return ld