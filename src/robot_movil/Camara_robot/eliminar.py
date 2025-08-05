import matplotlib.pyplot as plt
import numpy as np
import matplotlib.patches as patches

# Configuración del gráfico
plt.figure(figsize=(10, 8))
plt.grid(True)
plt.xlim(-1, 3)
plt.ylim(-1, 3)
plt.gca().set_aspect('equal')
plt.title("Aproximación corregida - Robot frente al cubo")
plt.xlabel("X (m)")
plt.ylabel("Y (m)")

# Parámetros del sistema
cube_center = np.array([2.0, 2.0])  # Posición central del cubo [x,y]
cube_yaw = np.radians(30)           # Orientación del cubo (30 grados convertidos a radianes)
cube_size = 0.1                     # Tamaño del cubo (lado)
robot_length = 0.5                  # Largo del robot
robot_width = 0.3                   # Ancho del robot
safety_offset = 0.5                 # Distancia de seguridad entre robot y cubo
approach_radius = 1.5               # Radio de la zona de aproximación

# Función para dibujar elementos
def draw_entity(pos, yaw, color, label, is_robot=True):
    if is_robot:
        # Cuerpo del robot (rectángulo)
        body = patches.Rectangle(
            (pos[0]-robot_width/2, pos[1]-robot_length/2), 
            robot_length, robot_width,
            angle=np.degrees(yaw),
            linewidth=2, edgecolor=color, facecolor='none'
        )
        plt.gca().add_patch(body)
        arrow_direction = np.array([np.cos(yaw), np.sin(yaw)])  # Frente del robot
    else:
        # Cuerpo del cubo (polígono)
        corners = np.array([[-cube_size/2, -cube_size/2], [cube_size/2, -cube_size/2], 
                          [cube_size/2, cube_size/2], [-cube_size/2, cube_size/2]])
        rot_matrix = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        rotated = pos + np.dot(corners, rot_matrix.T)
        body = plt.Polygon(rotated, closed=True, fill=True, color=color, alpha=0.7)
        plt.gca().add_patch(body)
        arrow_direction = -np.array([np.cos(yaw), np.sin(yaw)])  # Normal del cubo
    
    # Flecha CENTRAL para ambos
    plt.arrow(
        pos[0], pos[1],  # Siempre comienza en el centro
        arrow_direction[0]*0.45,  # Longitud (45% del tamaño del objeto)
        arrow_direction[1]*0.45,
        head_width=0.1,
        head_length=0.12,
        color='black',
        zorder=10,
        length_includes_head=True
    )
    
    # Etiqueta
    plt.text(pos[0], pos[1]+0.2, label, 
             ha='center', va='bottom',
             fontsize=9,
             bbox=dict(facecolor='white', alpha=0.7, edgecolor='none'))

# Función mejorada para selección de cara
def select_face(cube_pos, cube_yaw, robot_pos):
    # Vectores normales a cada cara
    normals = [
        np.array([np.cos(cube_yaw), np.sin(cube_yaw)]),    # Cara frontal
        np.array([-np.sin(cube_yaw), np.cos(cube_yaw)]),   # Cara izquierda
        np.array([np.sin(cube_yaw), -np.cos(cube_yaw)])    # Cara derecha
    ]
    
    # Seleccionar la cara más favorable (que requiera menor giro)
    best_idx = np.argmax([
        np.dot((cube_pos - robot_pos)/np.linalg.norm(cube_pos - robot_pos), n) 
        for n in normals
    ])
    
    best_normal = normals[best_idx]
    goal_pos = cube_pos - (cube_size/2 + safety_offset) * best_normal
    
    return goal_pos, np.arctan2(best_normal[1], best_normal[0])

# Calcular trayectoria
goal_pos, goal_yaw = select_face(cube_center, cube_yaw, np.array([0.0, -1.0]))

# Puntos intermedios
direction = (cube_center - np.array([0.0, -1.0])) / np.linalg.norm(cube_center - np.array([0.0, -1.0]))
first_stop = cube_center - direction * approach_radius

# Dibujar elementos
draw_entity(cube_center, cube_yaw, 'red', 'Cubo', is_robot=False)
plt.gca().add_patch(plt.Circle(cube_center, approach_radius, fill=False, 
                             linestyle='--', color='gray', alpha=0.5))

# Dibujar robots
draw_entity([0, -1], np.radians(45), 'blue', 'Inicio')
draw_entity(first_stop, np.arctan2(direction[1], direction[0]), 'cyan', 'Intermedio')
draw_entity(goal_pos, goal_yaw, 'purple', 'Final')

# Dibujar trayectoria (suavizada con curva)
from scipy.interpolate import make_interp_spline
x = np.array([0, first_stop[0], goal_pos[0]])
y = np.array([-1, first_stop[1], goal_pos[1]])
t = np.linspace(0, 1, 300)
spline = make_interp_spline([0, 0.5, 1], np.column_stack((x, y)), k=2)
smoothed = spline(t)
plt.plot(smoothed[:,0], smoothed[:,1], 'g--', alpha=0.7, label='Trayectoria')

plt.legend(loc='upper left')
plt.show()