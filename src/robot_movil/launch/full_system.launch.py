"""
Launch file con ejecución temporizada de nodos
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart


def generate_launch_description():
    pkg_dir = get_package_share_directory('robot_movil')
    
    # Definir los delays en segundos
    DELAY_CAMARA = 0.0      # Inmediato
    DELAY_ROBOTCAR = 5.0    # 2 segundos después de cámara
    DELAY_CUBORVIZ = 7.0    # 5 segundos después de inicio
    DELAY_GUI = 10.0         # 8 segundos después de inicio
    DELAY_BT_MAIN = 12.0    # 10 segundos después de inicio

    # Nodo cámara (se ejecuta inmediatamente)
    camara_node = Node(
        package='robot_movil', 
        executable='camara', 
        name='april_tag_camera_node', 
        output='screen'
    )

    tf_origen_node = Node(
        package='robot_movil', 
        executable='TForigen', 
        name='tf_origen_robot_node', 
        output='screen'
    )

    static_camara1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camara_to_odom',
        arguments=[
            '--x', '0.02',        # Desplazamiento en X (metros)
            '--y', '-0.01',        # Desplazamiento en Y
            '--z', '0.0',       # Desplazamiento en Z
            '--yaw', '0.0',      # Rotación en Z (radianes)
            '--pitch', '0.0',    # Rotación en Y
            '--roll', '0.0',     # Rotación en X
            '--frame-id', 'robot1/camara_link',
            '--child-frame-id', 'robot1/odom'
        ]
    )

    static_camara2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_camara_to_odom',
        arguments=[
            '--x', '-0.12',        # Desplazamiento en X (metros)
            '--y', '-0.08',        # Desplazamiento en Y
            '--z', '0.0',       # Desplazamiento en Z
            '--yaw', '0.0',      # Rotación en Z (radianes)
            '--pitch', '0.0',    # Rotación en Y
            '--roll', '0.0',     # Rotación en X
            '--frame-id', 'robot2/camara_link',
            '--child-frame-id', 'robot2/odom'
        ]
    )

    # Robotcar con delay
    robotcar_launch = TimerAction(
        period=DELAY_ROBOTCAR,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_dir, 'launch', 'robotcar.launch.py')
                )
            )
        ]
    )

    # Cuborviz con delay
    cuborviz_node = TimerAction(
        period=DELAY_CUBORVIZ,
        actions=[
            Node(
                package='robot_movil', 
                executable='cuborviz', 
                name='cuborviz', 
                output='screen'
            )
        ]
    )

    # GUI con delay
    gui_node = TimerAction(
        period=DELAY_GUI,
        actions=[
            Node(
                package='behavior_tree_multi_robot', 
                executable='gui', 
                name='gui', 
                output='screen'
            )
        ]
    )

    # BT Main con delay
    bt_main_node = TimerAction(
        period=DELAY_BT_MAIN,
        actions=[
            Node(
                package='behavior_tree_multi_robot', 
                executable='bt_main', 
                name='bt_main', 
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        camara_node,
        static_camara1,
        static_camara2,
        tf_origen_node,
        robotcar_launch,
        cuborviz_node,
        gui_node,
        bt_main_node
    ])