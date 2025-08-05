import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from pupil_apriltags import Detector
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import tf_transformations

from msg_nuevos.msg import AprilTagWorld, AprilTagWorldArray

class AprilTagCameraNode(Node):
    def __init__(self):
        super().__init__('april_tag_camera_node')
        
        # Configuración de tags para robots y cubos
        self.tag_config = {
            # Robots - Colores distintivos y contrastantes (mantenidos igual)
            9: ("robot1", (0, 120, 255)),    # Azul brillante
            1: ("robot2", (255, 50, 50)),    # Rojo vivo
            
            # Cubos - Colores completamente diferentes y contrastantes
            7: ("cubo1", (255, 0, 255)),     # Magenta
            6: ("cubo2", (255, 255, 0)),     # Amarillo
            458: ("cubo3", (0, 255, 255)),     # Cian
            459: ("cubo4", (255, 128, 0)),     # Naranja intenso
            463: ("cubo5", (128, 0, 255)),     # Violeta
            
            # Depósito - Color distintivo (mantenido igual)
            5: ("deposito", (255, 153, 204)), # Rosa pastel
            
            # Zonas de origen - Tonos morados/púrpura (mantenidos igual)
            3: ("robot1_origen", (180, 0, 180)), # Púrpura
            4: ("robot2_origen", (153, 255, 204)), # Verde menta pastel 
        }

        # Calibración de cámara
        calibration_file = 'src/robot_movil/Camara_robot/parametros_calibracion.npz'
        with np.load(calibration_file) as X:
            self.camera_matrix, self.dist_coeffs = X['mtx'], X['dist']
        
        self.tag_size = 0.075
        self.detector = Detector(families='tag36h11')
        self.cap = cv2.VideoCapture("/dev/video2")
        #self.cap = cv2.VideoCapture("http://192.168.100.164:8080/video")
        
        self.tag_pub = self.create_publisher(AprilTagWorldArray, 'apriltag_world_array', 10)
        self.image_pub = self.create_publisher(Image, 'tag_image', 10)
        self.bridge = CvBridge()

        self.timer = self.create_timer(0.1, self.detect_tags)
        self.get_logger().info("Inicializando nodo de cámara AprilTags...")

        #self.window_name = 'Detección de Tags'
        #cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        #cv2.resizeWindow(self.window_name, 800, 600)

    def detect_tags(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("No se pudo leer el frame de la cámara.")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = self.detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=(self.camera_matrix[0,0], self.camera_matrix[1,1],
                           self.camera_matrix[0,2], self.camera_matrix[1,2]),
            tag_size=self.tag_size
        )

        msg_array = AprilTagWorldArray()

        for det in detections:
            tag_id = det.tag_id
            if tag_id not in self.tag_config:
                continue

            # 1. Invertir coordenada Y en la traslación
            det.pose_t[1][0] = -det.pose_t[1][0]  # Inversión del eje Y

            # 2. Calcular rotación corregida
            # Extraer ángulos de Euler (roll, pitch, yaw) desde la matriz de rotación
            euler = tf_transformations.euler_from_matrix(det.pose_R, 'sxyz')
            roll, pitch, yaw = euler
            corrected_yaw = -yaw

            # Convertir a cuaternión corregido
            quat_corrected = tf_transformations.quaternion_from_euler(
                roll,
                pitch,
                corrected_yaw,
                'sxyz'
            )

            # Actualizar la matriz de rotación (opcional, solo si otros componentes la necesitan)
            det.pose_R = tf_transformations.quaternion_matrix(quat_corrected)[:3, :3]

            nombre, color = self.tag_config[tag_id]
            b, g, r = color

            corners = det.corners.astype(int)
            cv2.polylines(frame, [corners], isClosed=True, color=color, thickness=2)
            center = (int(det.center[0]), int(det.center[1]))
            cv2.circle(frame, center, 5, color, -1)
            text_pos = (corners[0][0], corners[0][1] - 10)
            cv2.putText(frame, nombre, text_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

            # Agregar a mensaje de salida
            tag_msg = AprilTagWorld()
            tag_msg.id = tag_id
            tag_msg.nombre = nombre
            tag_msg.posx = float(det.pose_t[0][0])
            tag_msg.posy = float(det.pose_t[1][0])            
            tag_msg.yaw = corrected_yaw
            tag_msg.r = r
            tag_msg.g = g
            tag_msg.b = b

            msg_array.tags.append(tag_msg)

        self.tag_pub.publish(msg_array)

        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        image_msg.header.stamp = self.get_clock().now().to_msg()
        self.image_pub.publish(image_msg)

        """cv2.imshow(self.window_name, frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.destroy_node()
            rclpy.shutdown()
            self.cap.release()
            cv2.destroyAllWindows()"""

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()