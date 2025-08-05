import rclpy
from rclpy.node import Node
import numpy as np
import math
import tf_transformations
from geometry_msgs.msg import TransformStamped
import tf2_ros
from msg_nuevos.msg import AprilTagWorldArray, PosRelativa
from msg_nuevos.srv import ResetTransform

from std_msgs.msg import Bool
import time

from concurrent.futures import Future

class AprilTagTFNode(Node):
    def __init__(self):
        super().__init__('april_tag_tf_node')
        
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.published_transforms = {}

        self.last_tags_msg = None

        self.srv_reset_transform = self.create_service(
            ResetTransform,
            'reset_robot_transform',
            self.handle_reset_transform
        )

        
        self.tag_sub = self.create_subscription(
            AprilTagWorldArray,
            'apriltag_world_array',
            self.tag_callback,
            10)
        
        self.sub_reset_robot1= self.create_subscription(
            Bool,
            "robot1/reset/response_srv",
            self.reset_robot1_callback,
            10)
        
        self.sub_reset_robot2 = self.create_subscription(
            Bool,
            "robot2/reset/response_srv",
            self.reset_robot2_callback,
            10)

        self.tag_pub_rel = self.create_publisher(PosRelativa, 'pos_relativa', 10)
        self.publisher_robot1 = self.create_publisher(Bool, "robot1/reset/request_srv", 10)
        self.publisher_robot2 = self.create_publisher(Bool, "robot2/reset/request_srv", 10)
        
        self.pos_rel_msg = PosRelativa()
        self.reset_robot1_msg = Bool()
        self.reset_robot2_msg = Bool()
        self.response_robot1 = False
        self.response_robot2 = False
        self.timer_rel = self.create_timer(0.1, self.posicion_relativa)
        
        self.get_logger().info("Inicializando nodo de transformaciones AprilTags...")

    def posicion_relativa(self):
        self.tag_pub_rel.publish(self.pos_rel_msg)
    
    def reset_robot1_callback(self, msg: Bool):
        self.get_logger().info(f"Callback recibido! Valor: {msg.data}")
        if msg.data:
            self.response_robot1 = True
            self.get_logger().info("Flag response_robot1 actualizado a True")
    
    def reset_robot2_callback(self, msg: Bool):
        if msg.data:
            self.response_robot2 = True

    def tag_callback(self, msg):
        self.last_tags_msg = msg
        for tag_msg in msg.tags:
            nombre = tag_msg.nombre
            
            # Solo para robots: publicar transformada estática una vez
            # Versión que detecta robots principales (sin sufijo _origen)
            if ("robot" in nombre 
                and not nombre.endswith('_origen') 
                and nombre not in self.published_transforms):
                x = tag_msg.posx
                y = tag_msg.posy
                yaw = tag_msg.yaw
                
                quat = tf_transformations.quaternion_from_euler(0, 0, yaw)

                if(nombre == 'robot1'):
                    self.response_robot1 = False
                    self.reset_robot1_msg.data = True
                    self.publisher_robot1.publish(self.reset_robot1_msg)

                elif(nombre == 'robot2'):
                    self.response_robot2 = False
                    self.reset_robot2_msg.data = True
                    self.publisher_robot2.publish(self.reset_robot2_msg)

                tf_msg = TransformStamped()
                tf_msg.header.stamp = self.get_clock().now().to_msg()
                tf_msg.header.frame_id = 'world'
                tf_msg.child_frame_id = f'{nombre}/camara_link'

                tf_msg.transform.translation.x = x
                tf_msg.transform.translation.y = y
                tf_msg.transform.translation.z = 0.0
                tf_msg.transform.rotation.x = quat[0]
                tf_msg.transform.rotation.y = quat[1]
                tf_msg.transform.rotation.z = quat[2]
                tf_msg.transform.rotation.w = quat[3]

                self.static_broadcaster.sendTransform(tf_msg)
                self.published_transforms[nombre] = True

                if nombre == "robot1":
                    self.pos_rel_msg.posx1 = x
                    self.pos_rel_msg.posy1 = y
                    self.pos_rel_msg.yaw1 = yaw
                
                if nombre == "robot2":
                    self.pos_rel_msg.posx2 = x
                    self.pos_rel_msg.posy2 = y
                    self.pos_rel_msg.yaw2 = yaw

                self.get_logger().info(f"TF estática publicada para {nombre}")
                self.get_logger().info(f"X: {x}")
                self.get_logger().info(f"Y: {y}")

    def handle_reset_transform(self, request, response):
        nombre_robot = request.nombre_robot.lower()
        intentos = 0
        max_intentos = 2

        while intentos < max_intentos:
            tag_msg = self.buscar_tag(nombre_robot)
            if tag_msg:
                self.get_logger().info(f"Transformada solicitada por servicio para {nombre_robot}")
                self.publicar_transformada(tag_msg)
                response.success = True
                return response
            else:
                self.get_logger().warn(f"{nombre_robot} no encontrado en tags. Reintentando...")
                time.sleep(2.0)
                intentos += 1

        self.get_logger().error(f"{nombre_robot} no disponible después de {max_intentos} intentos.")
        response.success = False
        return response

    def buscar_tag(self, nombre_robot):
        if self.last_tags_msg:
            for tag in self.last_tags_msg.tags:
                if tag.nombre == nombre_robot:
                    return tag
        return None

    def publicar_transformada(self, tag_msg):
        nombre = tag_msg.nombre
        x = tag_msg.posx
        y = tag_msg.posy
        yaw = tag_msg.yaw

        quat = tf_transformations.quaternion_from_euler(0, 0, yaw)

        if nombre == 'robot1':
            self.response_robot1 = False
            self.reset_robot1_msg.data = True
            self.publisher_robot1.publish(self.reset_robot1_msg)
            self.get_logger().info("Esperando respuesta del robot1...")
            """while not self.response_robot1:
                rclpy.spin_once(self)
                time.sleep(0.1)"""
        elif nombre == 'robot2':
            self.response_robot2 = False
            self.reset_robot2_msg.data = True
            self.publisher_robot2.publish(self.reset_robot2_msg)
            self.get_logger().info("Esperando respuesta del robot2...")
            """while not self.response_robot2:
                rclpy.spin_once(self)
                time.sleep(0.1)"""

        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = 'world'
        tf_msg.child_frame_id = f'{nombre}/camara_link'

        tf_msg.transform.translation.x = x
        tf_msg.transform.translation.y = y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.x = quat[0]
        tf_msg.transform.rotation.y = quat[1]
        tf_msg.transform.rotation.z = quat[2]
        tf_msg.transform.rotation.w = quat[3]

        self.static_broadcaster.sendTransform(tf_msg)
        self.published_transforms[nombre] = True

        if nombre == "robot1":
            self.pos_rel_msg.posx1 = x
            self.pos_rel_msg.posy1 = y
            self.pos_rel_msg.yaw1 = yaw
        elif nombre == "robot2":
            self.pos_rel_msg.posx2 = x
            self.pos_rel_msg.posy2 = y
            self.pos_rel_msg.yaw2 = yaw

        self.get_logger().info(f"TF estática publicada para {nombre}")

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagTFNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()