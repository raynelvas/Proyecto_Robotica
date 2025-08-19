import os

# Carpeta donde están las fotos
carpeta = "robot_movil/Camara_robot/calibracion_imgs"

# Listar todas las imágenes y ordenarlas por fecha de creación
imagenes = sorted(
    [f for f in os.listdir(carpeta) if f.lower().endswith((".jpg", ".png"))],
    key=lambda x: os.path.getctime(os.path.join(carpeta, x))
)

# Renombrar secuencialmente
for i, nombre in enumerate(imagenes, start=1):
    extension = os.path.splitext(nombre)[1]  # .jpg, .png, etc.
    nuevo_nombre = f"img{i}{extension}"
    ruta_antigua = os.path.join(carpeta, nombre)
    ruta_nueva = os.path.join(carpeta, nuevo_nombre)
    os.rename(ruta_antigua, ruta_nueva)
    print(f"{nombre} → {nuevo_nombre}")

print("✅ Imágenes renombradas correctamente.")
