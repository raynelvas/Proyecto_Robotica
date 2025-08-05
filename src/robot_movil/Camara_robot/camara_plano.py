import cv2
import numpy as np
from pupil_apriltags import Detector
import math

def rotation_matrix_to_euler_angles(R):
    """Convierte matriz de rotación a ángulos de Euler (en grados)"""
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.degrees([x, y, z])

def main():
    cap = cv2.VideoCapture("/dev/video2")
    if not cap.isOpened():
        print("No se pudo acceder a la cámara.")
        return

    with np.load('src/robot_movil/Camara_robot/parametros_calibracion.npz') as X:
        camera_matrix, dist_coeffs = X['mtx'], X['dist']

    tag_size = 0.06  # 6 cm

    at_detector = Detector(
        families='tag36h11',
        nthreads=1,
        quad_decimate=1.0,
        quad_sigma=0.0,
        refine_edges=True,
        decode_sharpening=0.25,
        debug=False
    )

    objetos = {
        1: "CARRO",
        2: "OBJETO 1",
        3: "OBJETO 2",
        4: "OBJETO 3"
    }

    ubicaciones = {
        5: "UBICACIÓN 1",
        6: "UBICACIÓN 2",
        7: "UBICACIÓN 3",
        8: "UBICACIÓN 4"
    }

    window_name = 'Detección de Objetos y Ubicaciones'
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 800, 600)

    # Ventana para mapa 2D
    map_width, map_height = 600, 600
    map_window_name = "Mapa 2D (x, y)"
    cv2.namedWindow(map_window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(map_window_name, map_width, map_height)

    # Parámetros para dibujar puntos en mapa
    scale = 400  # Escala: 1 metro = 400 píxeles
    offset_x = 100
    offset_y = map_height - 100  # Origen en esquina inferior izquierda con márgenes

    print("Sistema activo. Presiona 'q' para salir.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("No se pudo leer el frame.")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        detections = at_detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=(camera_matrix[0, 0], camera_matrix[1, 1],
                           camera_matrix[0, 2], camera_matrix[1, 2]),
            tag_size=tag_size
        )

        # Crear imagen negra para el mapa y dibujar ejes
        map_img = np.zeros((map_height, map_width, 3), dtype=np.uint8)
        # Ejes X y Y en color gris claro
        cv2.line(map_img, (0, offset_y), (map_width, offset_y), (100, 100, 100), 1)
        cv2.line(map_img, (offset_x, 0), (offset_x, map_height), (100, 100, 100), 1)
        cv2.putText(map_img, "Y", (offset_x + 10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200,200,200), 1)
        cv2.putText(map_img, "X", (map_width - 20, offset_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200,200,200), 1)

        for detection in detections:
            tag_id = detection.tag_id
            corners = detection.corners.astype(int)
            center = (int(detection.center[0]), int(detection.center[1]))

            cv2.polylines(frame, [corners], isClosed=True, color=(0, 255, 0), thickness=2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)

            texto = f"ID: {tag_id}"
            color = (0, 0, 0)

            if tag_id in objetos:
                color = (255, 0, 0)
                texto = f"{objetos[tag_id]} (ID: {tag_id})"
                cv2.rectangle(frame, (corners[0][0]-10, corners[0][1]-30),
                              (corners[2][0]+10, corners[2][1]+10), color, 2)
            elif tag_id in ubicaciones:
                color = (0, 255, 255)
                texto = f"{ubicaciones[tag_id]} (ID: {tag_id})"
                radius = int(max(abs(corners[0][0] - corners[2][0]),
                                 abs(corners[0][1] - corners[2][1])) // 2)
                cv2.circle(frame, center, radius, color, 2)

            cv2.putText(frame, texto, (corners[0][0], corners[0][1] - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

            # Estimación de pose
            t = detection.pose_t  # vector de traslación (x, y, z)
            R = detection.pose_R
            euler = rotation_matrix_to_euler_angles(R)

            print(f"\n📍 TAG ID {tag_id}")
            print(f"🔹 Posición [x, y, z] en metros: {t.ravel()}")
            print(f"🔄 Rotación [roll, pitch, yaw] en grados: {euler}")

            # Dibujar punto en mapa 2D
            x_m, y_m = t[0][0], t[1][0]  # posición en metros
            px = int(offset_x + x_m * scale)
            py = int(offset_y - y_m * scale)  # invertir eje y para que arriba sea positivo

            cv2.circle(map_img, (px, py), 7, (0, 255, 0), -1)
            cv2.putText(map_img, f"ID {tag_id}", (px + 10, py - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        cv2.imshow(window_name, frame)
        cv2.imshow(map_window_name, map_img)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    print("Sistema cerrado.")

if __name__ == '__main__':
    main()
