# reset_transform.py

import py_trees
from msg_nuevos.srv import ResetTransform
from rclpy.task import Future
from msg_nuevos.msg import AprilTagWorldArray


class ResetTransformNode(py_trees.behaviour.Behaviour):
    def __init__(self, name="ResetTransformNode", node=None):
        super().__init__(name)
        self.node = node
        self.client = None
        self.future = None
        self.sent_request = False
        self.robot_name = None
        self.other_robot_name = None
        self.other_robot_present = False
        self.other_robot_request_sent = False
        self.other_robot_future = None

        self.blackboard = py_trees.blackboard.Client()
        self.blackboard.register_key(
            key="order_info", access=py_trees.common.Access.READ
        )

        # Subscribe to AprilTag detections
        self.tag_subscription = self.node.create_subscription(
            AprilTagWorldArray,
            '/apriltag_world_array',
            self.callback_tags,
            10
        )
        self.current_tags = []

    def callback_tags(self, msg):
        """Store current AprilTag detections"""
        self.current_tags = msg.tags

    def initialise(self):
        self.sent_request = False
        self.future = None
        self.other_robot_present = False
        self.other_robot_request_sent = False
        self.other_robot_future = None

        # Leer robot del blackboard
        try:
            self.robot_name = self.blackboard.order_info[0]
            # Determine the other robot name
            if self.robot_name == "robot1":
                self.other_robot_name = "robot2"
            else:
                self.other_robot_name = "robot1"
        except Exception as e:
            self.node.get_logger().error(f"Error leyendo order_info del blackboard: {e}")
            self.robot_name = None
            self.other_robot_name = None

        if self.robot_name:
            # Check if the other robot is present in AprilTag detections
            for tag in self.current_tags:
                if tag.nombre == self.other_robot_name:
                    self.other_robot_present = True
                    break

            self.client = self.node.create_client(ResetTransform, 'reset_robot_transform')
            while not self.client.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().info('Esperando servicio reset_robot_transform...')
        else:
            self.node.get_logger().error("No se encontró el nombre del robot en el blackboard.")

    def update(self):
        if not self.robot_name:
            return py_trees.common.Status.FAILURE

        # First, handle the main robot request
        if not self.sent_request:
            request = ResetTransform.Request()
            request.nombre_robot = self.robot_name
            self.future = self.client.call_async(request)
            self.sent_request = True
            return py_trees.common.Status.RUNNING

        # Then handle the other robot if present
        if self.other_robot_present and not self.other_robot_request_sent:
            request = ResetTransform.Request()
            request.nombre_robot = self.other_robot_name
            self.other_robot_future = self.client.call_async(request)
            self.other_robot_request_sent = True
            return py_trees.common.Status.RUNNING

        # Check if all requests are done
        all_done = self.future.done()
        if self.other_robot_present:
            all_done = all_done and self.other_robot_future.done()

        if all_done:
            try:
                # Check main robot result
                main_response = self.future.result()
                if not main_response.success:
                    self.node.get_logger().warn(f"No se pudo publicar la transformación para {self.robot_name}")
                    return py_trees.common.Status.FAILURE

                # Check other robot result if present
                if self.other_robot_present:
                    other_response = self.other_robot_future.result()
                    if not other_response.success:
                        self.node.get_logger().warn(f"No se pudo publicar la transformación para {self.other_robot_name}")
                        # We still return SUCCESS since the main robot was successful
                
                self.node.get_logger().info(f"Transformación publicada correctamente para {self.robot_name}" + 
                                         (f" y {self.other_robot_name}" if self.other_robot_present else ""))
                return py_trees.common.Status.SUCCESS

            except Exception as e:
                self.node.get_logger().error(f"Error al llamar al servicio: {e}")
                return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING