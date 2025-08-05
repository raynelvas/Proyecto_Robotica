
# Nodo condicional: devuelve SUCCESS si otro robot está presente
import py_trees
from msg_nuevos.msg import AprilTagWorldArray

class CheckOtherRobotPresent(py_trees.behaviour.Behaviour):
    def __init__(self, name="CheckOtherRobotPresent", node=None):
        super().__init__(name)
        self.node = node
        self.blackboard = py_trees.blackboard.Client()
        self.blackboard.register_key(key="order_info", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="other_robot", access=py_trees.common.Access.WRITE)

        self.tags_dict = {}
        self.subscription = self.node.create_subscription(
            AprilTagWorldArray,
            '/apriltag_world_array',
            self.callback_tags,
            10
        )

    def callback_tags(self, msg):
        self.tags_dict.clear()
        for tag in msg.tags:
            self.tags_dict[tag.nombre] = (tag.posx, tag.posy, tag.yaw)

    def update(self):
        try:
            selected_robot, _ = self.blackboard.order_info
            # Buscar si existe el otro robot en el diccionario de tags
            other_robot = None
            for nombre in ["robot1", "robot2"]:
                if nombre != selected_robot:
                    # Verificar si el robot Y su tag de origen existen
                    if nombre in self.tags_dict and f"{nombre}_origen" in self.tags_dict:
                        other_robot = nombre
                        break

            if other_robot:
                self.blackboard.other_robot = other_robot
                self.node.get_logger().info(f"Detectado {other_robot} y su posición de origen, preparando para la subrama")
                return py_trees.common.Status.SUCCESS
            else:
                self.node.get_logger().info("No se detectó otro robot con su origen, se salta la subrama")
                return py_trees.common.Status.FAILURE

        except Exception as e:
            self.node.get_logger().error(f"Error en CheckOtherRobotPresent: {e}")
            return py_trees.common.Status.FAILURE
