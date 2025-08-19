import cv2
import numpy as np
import glob
import os

# Configuración del patrón de calibración
num_esquinas_x = 6  # columnas de esquinas internas
num_esquinas_y = 4   # filas de esquinas internas
tamano_cuadro_mm = 29  # tamaño de cada cuadro en milímetros
# Ruta relativa a la carpeta donde se guardan las imágenes
ruta_imgs = 'calibracion_imgs/*.jpg'

# Puntos 3D reales del patrón
objp = np.zeros((num_esquinas_y * num_esquinas_x, 3), np.float32)
objp[:, :2] = np.mgrid[0:num_esquinas_x, 0:num_esquinas_y].T.reshape(-1, 2)
objp *= tamano_cuadro_mm

# Listas para almacenar puntos del mundo real y sus proyecciones en las imágenes
objpoints = []
imgpoints = []

imagenes = glob.glob(ruta_imgs)

if not imagenes:
    print("❌ No se encontraron imágenes en la carpeta 'calibracion_imgs/'.")
    exit()

for fname in imagenes:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCorners(gray, (num_esquinas_x, num_esquinas_y), None)

    if ret:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1),
            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        )
        imgpoints.append(corners2)

        # Visualizar para validar detección
        img_drawn = cv2.drawChessboardCorners(img, (num_esquinas_x, num_esquinas_y), corners2, ret)
        cv2.imshow('Esquinas detectadas', img_drawn)
        cv2.waitKey(200)

cv2.destroyAllWindows()

# Calibración de la cámara
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)

print("\n✅ Calibración completada")
print(f"\n📷 Matriz de cámara (camera_matrix):\n{mtx}")
print(f"\n🔧 Coeficientes de distorsión:\n{dist.ravel()}")

# Guardar parámetros en un archivo .npz
np.savez('parametros_calibracion.npz', mtx=mtx, dist=dist)
print("\n💾 Parámetros guardados en 'parametros_calibracion.npz'")
