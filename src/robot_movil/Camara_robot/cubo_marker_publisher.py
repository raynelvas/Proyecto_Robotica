import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from msg_nuevos.msg import AprilTagWorldArray
import tf_transformations

class CuboPublisher(Node):
    def __init__(self):
        super().__init__('cubo_publisher')
        self.publisher = self.create_publisher(Marker, '/visualization_marker', 10)
        self.subscription = self.create_subscription(
            AprilTagWorldArray,
            '/apriltag_world_array',
            self.listener_callback,
            10
        )
        self.br = TransformBroadcaster(self)
        # Diccionarios separados por tipo de objeto
        self.active_objects = {
            'cubo': set(),
            'deposito': set(),
            'origen': set()
        }
        self.last_seen = {}

    def listener_callback(self, msg):
        # Procesamos cada tipo de objeto por separado
        self.process_objects(msg, 'cubo', Marker.CUBE, 0.1, 0.1, 0.1)
        self.process_objects(msg, 'deposito', Marker.CUBE, 0.2, 0.2, 0.02)
        self.process_objects(msg, 'origen', Marker.CYLINDER, 0.05, 0.05, 0.1)

    def process_objects(self, msg, object_type, marker_type, scale_x, scale_y, scale_z):
        current_objects = set()
        
        for tag in msg.tags:
            if object_type in tag.nombre.lower():
                current_objects.add(tag.nombre)
                self.last_seen[tag.nombre] = self.get_clock().now().seconds_nanoseconds()[0]
                
                self.publish_marker(
                    tag, 
                    marker_type, 
                    scale_x, scale_y, scale_z,
                    scale_z/2,  # Posición Z (altura)
                    object_type
                )
                
                self.publish_tf(tag, object_type)

        # Eliminar objetos que ya no están presentes
        disappeared_objects = self.active_objects[object_type] - current_objects
        for obj_name in disappeared_objects:
            self.delete_marker(obj_name)
        
        self.active_objects[object_type] = current_objects

    def publish_marker(self, tag, marker_type, scale_x, scale_y, scale_z, z_pos, object_type):
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()

        # Modificar el nombre solo para origenes
        if object_type == 'origen':
            marker.ns = tag.nombre.replace('robot', 'r')
        else:
            marker.ns = tag.nombre
        
        yaw = tag.yaw                
        quat = tf_transformations.quaternion_from_euler(0, 0, yaw)
        
        marker.id = tag.id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.position.x = tag.posx
        marker.pose.position.y = tag.posy
        marker.pose.position.z = z_pos
        marker.pose.orientation.x = quat[0]
        marker.pose.orientation.y = quat[1]
        marker.pose.orientation.z = quat[2]
        marker.pose.orientation.w = quat[3]
        marker.scale.x = scale_x
        marker.scale.y = scale_y
        marker.scale.z = scale_z
        marker.color = ColorRGBA(
            r=tag.r/255,
            g=tag.g/255,
            b=tag.b/255,
            a=1.0
        )
        marker.lifetime.sec = 1
        self.publisher.publish(marker)

    def publish_tf(self, tag, object_type):
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'world'
       
        # Modificar el nombre solo para origenes
        if object_type == 'origen':
            # Reemplazar "robot" por "r" en el nombre del origen
            tf.child_frame_id = tag.nombre.replace('robot', 'r')
        else:
            tf.child_frame_id = tag.nombre

        yaw = tag.yaw                
        quat = tf_transformations.quaternion_from_euler(0, 0, yaw)

        tf.transform.translation.x = tag.posx
        tf.transform.translation.y = tag.posy
        tf.transform.translation.z = 0.0
        tf.transform.rotation.x = quat[0]
        tf.transform.rotation.y = quat[1]
        tf.transform.rotation.z = quat[2]
        tf.transform.rotation.w = quat[3]
        self.br.sendTransform(tf)

    def delete_marker(self, object_name):
        """Elimina un marcador específico"""
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = object_name
        marker.id = 0  # Mismo ID usado al crearlo
        marker.action = Marker.DELETE
        self.publisher.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = CuboPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()