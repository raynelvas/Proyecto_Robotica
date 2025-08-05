"""
Nodo que espera señal de inicio desde GUI
@param node: Nodo ROS2 para crear suscripción
"""
import py_trees
import rclpy
from std_msgs.msg import Bool
from msg_nuevos.msg import AprilTagWorldArray

class WaitForStart(py_trees.behaviour.Behaviour):
    def __init__(self, name="Start", node=None):
        super().__init__(name)
        self.node = node
        self.start_received = False
        self.required_tags = {
            'robots': ['robot1', 'robot2'],
            'cubes': [f'cubo{i}' for i in range(1, 6)],
            'deposit': ['deposito']
        }
        self.detected_tags = set()

        # Suscriptor al botón de start
        self.start_sub = self.node.create_subscription(
            Bool,
            '/boton_start',
            self.start_callback,
            10
        )

        # Suscriptor a los AprilTags
        self.tags_sub = self.node.create_subscription(
            AprilTagWorldArray,
            '/apriltag_world_array',
            self.tags_callback,
            10
        )

    def start_callback(self, msg):
        self.start_received = msg.data
        if self.start_received:
            self.node.get_logger().info("Señal START recibida")

    def tags_callback(self, msg):
        """Actualiza los tags detectados"""
        current_tags = {tag.nombre for tag in msg.tags}
        self.detected_tags = current_tags

    def check_required_tags(self):
        """Verifica si existen los tags requeridos"""
        has_robot = any(robot in self.detected_tags for robot in self.required_tags['robots'])
        has_cube = any(cube in self.detected_tags for cube in self.required_tags['cubes'])
        has_deposit = any(dep in self.detected_tags for dep in self.required_tags['deposit'])
        
        return has_robot and has_cube and has_deposit

    def initialise(self):
        self.start_received = False
        self.detected_tags = set()

    def update(self):
        if not self.start_received:
            return py_trees.common.Status.RUNNING
            
        if self.check_required_tags():
            self.node.get_logger().info("Todos los elementos requeridos detectados")
            return py_trees.common.Status.SUCCESS
        else:
            self.node.get_logger().warn("Faltan elementos requeridos (robot, cubo o depósito)")
            return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        """Limpieza al finalizar"""
        self.node.get_logger().debug(f"WaitForStart: Terminando con estado {new_status}")