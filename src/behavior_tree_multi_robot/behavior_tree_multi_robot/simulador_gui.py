#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from msg_nuevos.srv import ElegirRobot
from msg_nuevos.msg import AprilTagWorldArray
from sensor_msgs.msg import Image

import threading
import tkinter as tk
from tkinter import ttk
from PIL import Image as PILImage, ImageTk
from cv_bridge import CvBridge
import cv2
import numpy as np
from datetime import datetime
from std_msgs.msg import Bool  # Importa el tipo de mensaje
from msg_nuevos.msg import Estado

class SimuladorGUI(Node):
    def __init__(self):
        super().__init__('simulador_gui')

        # Sincronización GUI
        self.gui_ready = threading.Event()

        # Servicio
        self.srv = self.create_service(ElegirRobot, 'elegir_robot', self.callback)

        # Subscripciones
        self.create_subscription(AprilTagWorldArray, '/apriltag_world_array', self.callback_tags, 10)
        self.create_subscription(Image, '/tag_image', self.callback_image, 10)

        # Agrega esto para suscribirte al estado del brazo
        self.brazo_status_sub = self.create_subscription(
            Estado,
            'estado_robot',
            self.brazo_status_callback,
            10
        )
        self.brazo_activo = False  # Variable para rastrear el estado

        # Agrega esto para crear el publicador
        self.boton_publisher = self.create_publisher(Bool, '/boton_start', 10)

        self.bridge = CvBridge()
        self.robot_seleccionado = None
        self.cubo_seleccionado = None
        self.esperando_respuesta = threading.Event()

        self.lista_robots = []
        self.lista_cubos = []
        self.cv_image = np.zeros((1080, 1040, 3), dtype=np.uint8)

        # GUI en hilo separado
        gui_thread = threading.Thread(target=self.crear_gui)
        gui_thread.daemon = True
        gui_thread.start()

    def crear_gui(self):
        self.root = tk.Tk()
        self.root.title("Simulador de Robots Multiagente")
        self.root.geometry("1200x700")
        self.root.configure(bg="#2c3e50")
        
        # Configurar estilo
        self.style = ttk.Style()
        self.style.theme_use('clam')
        self.style.configure(".", background="#2c3e50", foreground="#ecf0f1")
        self.style.configure("TFrame", background="#2c3e50")
        self.style.configure("TLabel", background="#2c3e50", foreground="#ecf0f1", font=("Arial", 12))
        self.style.configure("TButton", font=("Arial", 12, "bold"), padding=10, background="#3498db")
        self.style.configure("Title.TLabel", font=("Arial", 16, "bold"))
        self.style.configure("Listbox", background="#34495e", foreground="#ecf0f1", font=("Arial", 11))
        self.style.configure("Status.TLabel", font=("Arial", 14, "bold"), foreground="#f39c12")
        self.style.configure("TLabelFrame", background="#2c3e50", foreground="#ecf0f1", font=("Arial", 12, "bold"))
        self.style.map("TButton", background=[("active", "#2980b9")])
        
        # Marco principal
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=20)
        
        # Marco para la imagen (izquierda)
        img_frame = ttk.Frame(main_frame)
        img_frame.grid(row=0, column=0, rowspan=2, padx=10, pady=10, sticky="nsew")

        # --- Columna derecha reorganizada ---
        right_column = ttk.Frame(main_frame)
        right_column.grid(row=0, column=1, padx=10, pady=10, sticky="nsew")

        # Marco para  Botón de inicio (derecha)
        start_frame = ttk.Frame(right_column)
        start_frame.grid(row=0, column=0, padx=10, pady=5, sticky="nsew")
        
        # Marco para listas (derecha)
        lists_frame = ttk.Frame(right_column)
        lists_frame.grid(row=1, column=0, padx=10, pady=10, sticky="nsew")

        # Marco para botón y selección (debajo de las listas)
        selection_frame = ttk.Frame(right_column)
        selection_frame.grid(row=2, column=0, padx=10, pady=10, sticky="nsew")
        
        # Configurar pesos de la cuadrícula
        main_frame.columnconfigure(0, weight=3)
        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(0, weight=5)
        main_frame.rowconfigure(1, weight=1)
        
        # --- Panel de imagen ---
        img_title = ttk.Label(img_frame, text="VISUALIZACIÓN DE CÁMARA", style="Title.TLabel")
        img_title.pack(pady=(0, 10))
        
        self.panel_imagen = tk.Label(img_frame, bg="#34495e", bd=2, relief=tk.SUNKEN)
        self.panel_imagen.pack(fill=tk.BOTH, expand=True)
        
        self.label = ttk.Label(img_frame, text="Esperando solicitud...", style="Status.TLabel")
        self.label.pack(pady=10)
        
        # --- Lista de cubos (derecha) ---
        cubos_frame = ttk.LabelFrame(lists_frame, text="CUBOS DETECTADOS", padding=10)
        cubos_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        self.lista_cubos_box = tk.Listbox(cubos_frame, width=25, height=6, 
                                        bg="#34495e", fg="#ecf0f1", font=("Arial", 11),
                                        selectbackground="#3498db", selectmode=tk.SINGLE,
                                        bd=0, highlightthickness=0, exportselection=False)
        self.lista_cubos_box.pack(fill=tk.BOTH, expand=True)
        
        scroll_cubos = ttk.Scrollbar(cubos_frame, orient=tk.VERTICAL, command=self.lista_cubos_box.yview)
        scroll_cubos.pack(side=tk.RIGHT, fill=tk.Y)
        self.lista_cubos_box.config(yscrollcommand=scroll_cubos.set)
        
        # --- Lista de robots (derecha) ---
        robots_frame = ttk.LabelFrame(lists_frame, text="ROBOTS DETECTADOS", padding=10)
        robots_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        self.lista_robots_box = tk.Listbox(robots_frame, width=25, height=3, 
                                         bg="#34495e", fg="#ecf0f1", font=("Arial", 11),
                                         selectbackground="#3498db", selectmode=tk.SINGLE,
                                         bd=0, highlightthickness=0, exportselection=False)
        self.lista_robots_box.pack(fill=tk.BOTH, expand=True)
        
        scroll_robots = ttk.Scrollbar(robots_frame, orient=tk.VERTICAL, command=self.lista_robots_box.yview)
        scroll_robots.pack(side=tk.RIGHT, fill=tk.Y)
        self.lista_robots_box.config(yscrollcommand=scroll_robots.set)
        
        # --- Panel de selección actual (centro-derecha) ---
        seleccion_frame = ttk.LabelFrame(selection_frame, text="SELECCIÓN ACTUAL", padding=10)
        seleccion_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        # Frame para robot seleccionado
        robot_frame = ttk.Frame(seleccion_frame)
        robot_frame.pack(fill=tk.X, pady=5)
        ttk.Label(robot_frame, text="🤖 Robot:", font=("Arial", 12)).pack(side=tk.LEFT)
        self.robot_seleccionado_label = ttk.Label(robot_frame, text="Ninguno", font=("Arial", 12, "bold"), foreground="#3498db")
        self.robot_seleccionado_label.pack(side=tk.LEFT, padx=10)
        
        # Frame para cubo seleccionado
        cubo_frame = ttk.Frame(seleccion_frame)
        cubo_frame.pack(fill=tk.X, pady=5)
        ttk.Label(cubo_frame, text="📦 Cubo:", font=("Arial", 12)).pack(side=tk.LEFT)
        self.cubo_seleccionado_label = ttk.Label(cubo_frame, text="Ninguno", font=("Arial", 12, "bold"), foreground="#e67e22")
        self.cubo_seleccionado_label.pack(side=tk.LEFT, padx=10)
        
        # Botón de confirmación
        self.confirmar_btn = ttk.Button(selection_frame, text="CONFIRMAR SELECCIÓN", 
                                      state=tk.DISABLED, command=self.confirmar_seleccion)
        self.confirmar_btn.pack(fill=tk.X, pady=10)

        # --- Añade esto: Label para el estado del brazo ---
        self.brazo_status_label = ttk.Label(
            selection_frame,
            text="🦾 Brazo: INACTIVO",
            style="Status.TLabel",
            foreground="red"  # Rojo cuando está inactivo
        )
        self.brazo_status_label.pack(fill=tk.X, pady=5)


        # Agrega esto en la sección de botones (después del botón de confirmación)
        self.start_btn = ttk.Button(start_frame, text="INICIAR", 
                                state=tk.NORMAL, command=self.publicar_boton)
        self.start_btn.pack(fill=tk.X, pady=10)
        
        # Iniciar actualización de imagen
        self.actualizar_imagen()
        self.gui_ready.set()
        self.root.mainloop()

    def actualizar_estado_brazo(self):
        """Actualiza el Label del estado del brazo en la GUI"""
        if self.brazo_activo:
            self.brazo_status_label.config(
                text="🦾 Brazo: MOVIÉNDOSE",
                foreground="green"  # Verde cuando está activo
            )
        else:
            self.brazo_status_label.config(
                text="🦾 Brazo: INACTIVO",
                foreground="red"
            )

    def brazo_status_callback(self, msg):
        """Callback para actualizar el estado del brazo"""
        self.brazo_activo = msg.brazo
        if hasattr(self, 'root'):
            self.root.after(0, self.actualizar_estado_brazo)

    def callback_tags(self, msg):
        """Callback para actualizar las listas de robots y cubos detectados"""
        if not self.gui_ready.wait(timeout=0.1):
            return

        nuevos_cubos = []
        nuevos_robots = []
        self.tag_colors = {}  # Diccionario para almacenar colores por nombre


        for tag in msg.tags:
            # Almacenar color en formato hexadecimal
            color_hex = f'#{tag.r:02x}{tag.g:02x}{tag.b:02x}'
            self.tag_colors[tag.nombre] = color_hex

            # Excluir cualquier tag que contenga 'origen'
            if "origen" in tag.nombre.lower():
                continue

            if "robot" in tag.nombre.lower() :
                nuevos_robots.append(tag.nombre)
            elif "cubo" in tag.nombre.lower():
                nuevos_cubos.append(tag.nombre)

        self.lista_cubos = nuevos_cubos
        self.lista_robots = nuevos_robots

        if hasattr(self, 'root'):
            self.root.after(0, self.actualizar_listas)

    def callback_image(self, msg):
        """Callback para actualizar la imagen de la cámara"""
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error al convertir imagen: {e}")

    def actualizar_imagen(self):
        """Actualiza periódicamente la imagen en la GUI"""
        try:
            image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2RGB)
            image = PILImage.fromarray(image)
            image = image.resize((1040, 780))
            imgtk = ImageTk.PhotoImage(image=image)

            self.panel_imagen.imgtk = imgtk
            self.panel_imagen.config(image=imgtk)
        except Exception as e:
            self.get_logger().warn(f"No se pudo mostrar imagen: {e}")

        self.root.after(100, self.actualizar_imagen)

    def actualizar_listas(self):
        """Actualiza las listas de robots y cubos en la GUI"""
        self.lista_cubos_box.delete(0, tk.END)
        for nombre in sorted(set(self.lista_cubos)):
            self.lista_cubos_box.insert(tk.END, nombre)

        self.lista_robots_box.delete(0, tk.END)
        for nombre in sorted(set(self.lista_robots)):
            self.lista_robots_box.insert(tk.END, nombre)

    def confirmar_seleccion(self):
        try:
            robot_idx = self.lista_robots_box.curselection()
            cubo_idx = self.lista_cubos_box.curselection()

            if len(robot_idx) != 1 or len(cubo_idx) != 1:
                self.label.config(text="Selecciona 1 robot y 1 cubo.")
                return

            self.robot_seleccionado = self.lista_robots_box.get(robot_idx[0])
            self.cubo_seleccionado = self.lista_cubos_box.get(cubo_idx[0])

            # Obtener colores del diccionario
            robot_color = self.tag_colors.get(self.robot_seleccionado, "#3498db")  # Azul por defecto
            cubo_color = self.tag_colors.get(self.cubo_seleccionado, "#e67e22")   # Naranja por defecto

            # Actualizar interfaz con colores
            self.label.config(text=f"✅ Selección confirmada: {self.robot_seleccionado} → {self.cubo_seleccionado}")
            self.confirmar_btn.config(state=tk.DISABLED)
            self.start_btn.config(state=tk.NORMAL)
            
            # Resaltar selección con sus colores reales
            self.lista_robots_box.itemconfig(robot_idx[0], bg=robot_color, fg="white")
            self.lista_cubos_box.itemconfig(cubo_idx[0], bg=cubo_color, fg="white")
            
            # Actualizar panel de selección
            self.robot_seleccionado_label.config(text=self.robot_seleccionado, foreground=robot_color)
            self.cubo_seleccionado_label.config(text=self.cubo_seleccionado, foreground=cubo_color)
            
            self.esperando_respuesta.set()
            
        except Exception as e:
            self.get_logger().error(f"Error al confirmar selección: {e}")

    def publicar_boton(self):
        """Publica True cuando se presiona el botón"""
        msg = Bool()
        msg.data = True
        self.boton_publisher.publish(msg)
        self.get_logger().info("Mensaje de start TRUE publicado")
        
        # Opcional: Deshabilitar el botón temporalmente
        #self.root.after(3000, lambda: self.start_btn.config(state=tk.NORMAL))
    
    def callback(self, request, response):
        self.get_logger().info("Solicitud recibida para elegir robot y cubo")

        # Resetear selección
        self.robot_seleccionado = None
        self.cubo_seleccionado = None
        self.esperando_respuesta.clear()

        # Resetear interfaz
        self.root.after(0, lambda: self.label.config(text="Selecciona 1 robot y 1 cubo y confirma."))
        self.root.after(0, lambda: self.confirmar_btn.config(state=tk.NORMAL))
        self.root.after(0, lambda: self.start_btn.config(state=tk.DISABLED))
        self.root.after(0, lambda: self.robot_seleccionado_label.config(text="Ninguno"))
        self.root.after(0, lambda: self.cubo_seleccionado_label.config(text="Ninguno"))
        
        # Resetear colores de las listas
        self.root.after(0, self.resetear_colores_listas)

        self.esperando_respuesta.wait()

        response.robot_namespace = self.robot_seleccionado
        response.cubo = self.cubo_seleccionado
        return response

    def resetear_colores_listas(self):
        """Restablece los colores de las listas a los originales"""
        for i in range(self.lista_robots_box.size()):
            self.lista_robots_box.itemconfig(i, bg="#34495e", fg="#ecf0f1")
        
        for i in range(self.lista_cubos_box.size()):
            self.lista_cubos_box.itemconfig(i, bg="#34495e", fg="#ecf0f1")

def main(args=None):
    rclpy.init(args=args)
    node = SimuladorGUI()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()