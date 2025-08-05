# input_coordinates.py
import py_trees
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener
from msg_nuevos.msg import AprilTagWorldArray, PosRelativa  # Ajusta el import según tu paquete
import tf_transformations
import math

class InputCoordinates(py_trees.behaviour.Behaviour):
    def __init__(self, name="InputCoordinates", node=None,target_type=None):  # Añade parámetro
        super().__init__(name)
        self.node = node
 
        # Declarar puerto de salida
        self.blackboard = py_trees.blackboard.Client()
        self.blackboard.register_key(key="order_info",access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="other_robot", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="goal_coords",access=py_trees.common.Access.WRITE)
        
        self.robot_name = None
        self.target_name = None
        self.target_type = target_type

        self.tx = None
        self.ty = None
        self.yaw = None
        self.done_init = False
        self.done_callback = False

        #Parametros de corrección
        self.x1 = 0.02
        self.y1 = -0.07

         # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node, spin_thread=False)

        # Variable para almacenar datos del callback
        self.tags_dict = {}  # {tag_id: (x, y)}

        self.node.create_subscription(
            AprilTagWorldArray,
            '/apriltag_world_array',
            self.callback_tags,
            10
        )

        self.node.create_subscription(
            PosRelativa,
            '/pos_relativa',
            self.pos_rel_callaback,
            10
        )
        

    def initialise(self):
        # Se llama automáticamente antes de cada ejecución
        if(self.target_type =='cubo'):
            self.robot_name,self.target_name = self.blackboard.order_info
        elif(self.target_type =='deposito'):
            self.robot_name,_ = self.blackboard.order_info
            self.target_name = self.target_type 
        elif(self.target_type =='origen'):
            self.robot_name,_ = self.blackboard.order_info
            self.target_name = f'{self.robot_name}_origen'
        elif(self.target_type =='other_origin'):
            self.robot_name = self.blackboard.other_robot
            self.target_name = f'{self.robot_name}_origen'
        self.done_init = True
        pass

    def pos_rel_callaback(self, msg):
        if(self.robot_name=='robot1' and self.done_init):
            self.tx = msg.posx1
            self.ty = msg.posy1
            self.yaw = msg.yaw1
            self.done_callback = True

        if(self.robot_name=='robot2' and self.done_init):
            self.tx = msg.posx2
            self.ty = msg.posy2
            self.yaw = msg.yaw2
            self.done_callback = True

    def callback_tags(self, msg):
        # Almacena las coordenadas absolutas de cada tag
        self.tags_dict.clear()
        for tag in msg.tags:
            self.tags_dict[tag.nombre] = (tag.posx, tag.posy)

    def update(self):        
        try:            
             # Verificar condiciones necesarias (todas deben ser True)
            if not (self.done_init and 
                    self.done_callback and 
                    self.robot_name in self.tags_dict and 
                    self.target_name in self.tags_dict and
                    self.tx is not None and
                    self.ty is not None and
                    self.yaw is not None):
                self.node.get_logger().warn("Esperando datos iniciales...", throttle_duration_sec=1)
                return py_trees.common.Status.RUNNING

            #pos_robot = self.tags_dict[self.robot_name]  # (x, y)
            pos_target = self.tags_dict[self.target_name]     # (x, y)

            target_x, target_y = pos_target

            # Obtener la transformada de world -> robotX/odom
            try:
                #self.node.get_logger().info(f"Posición tx:{self.tx}, ty:{self.ty}")

                # Transformar punto del target a frame del robot (odom)
                dx = target_x - self.tx
                dy = target_y - self.ty

                rel_x = math.cos(self.yaw) * dx + math.sin(self.yaw) * dy - self.x1
                rel_y = -math.sin(self.yaw) * dx + math.cos(self.yaw) * dy - self.y1

                # Guardar coordenadas relativas
                self.blackboard.goal_coords = [rel_x, rel_y]
                self.node.get_logger().info(f"Destino relativo al robot: X={rel_x:.2f}, Y={rel_y:.2f}")
                return py_trees.common.Status.SUCCESS
        
            except Exception as e:
                self.node.get_logger().error(f"Error al obtener transformada TF: {e}")
                return py_trees.common.Status.RUNNING
            
        except ValueError:
            self.node.get_logger().error("Coordenadas inválidas!")
            return py_trees.common.Status.FAILURE


    def terminate(self, new_status):
        # Se llama cuando el estado está por cambiar
        self.done_init = False
        self.done_callback = False
        if new_status == py_trees.common.Status.INVALID:
            self.node.get_logger().error("Estado invalido")