# Flujo Completo de Recolección de Datos para Triple Cámara + Doble Brazo Robótico Maestro-Esclavo

## 📋 Descripción General del Sistema

Este sistema se utiliza para recolectar datos de aprendizaje por demostración robótica, que incluyen:
- **3 cámaras Intel RealSense** (izquierda, centro, derecha)
- **2 brazos robóticos Piper maestro-esclavo** (brazo izquierdo, brazo derecho)
- **Formato de datos**: HDF5 (incluye imágenes RGB/profundidad, estado de las articulaciones, pose del extremo final)

---

## Configuración del Entorno

### Instalación del SDK de Intel RealSense

```bash
# 1. Agregar el repositorio de software de Intel RealSense
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" | \
sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt update

# 2. Instalar el SDK de RealSense
sudo apt install librealsense2-dkms librealsense2-utils librealsense2-dev -y

# 3. Verificar la instalación (ejecutar después de conectar la cámara)
realsense-viewer  # Abre el visor de cámaras, deberías poder ver las imágenes


# 4. Verificar los dispositivos RealSense conectados
rs-enumerate-devices
```

### Instalación de Herramientas de Comunicación CAN

```bash
# Instalar el paquete de herramientas CAN
sudo apt install can-utils ethtool -y

# Cargar el módulo del núcleo gs_usb
sudo modprobe gs_usb

# Configurar la carga automática de gs_usb al iniciar
echo "gs_usb" | sudo tee -a /etc/modules

# Verificar el dispositivo CAN (ejecutar después de conectar el dispositivo USB-CAN)
lsusb | grep -i can
# Deberías ver el dispositivo convertidor CAN
```

### Configuración del Entorno de Python

```bash
# 1. Asegurar el uso de Python 3.10+
python3 --version
# Debería mostrar: Python 3.10.x

# 2. Instalar pip
sudo apt install python3-pip -y

# 3. Instalar las librerías de Python necesarias para la recolección de datos
pip3 install h5py dm_env numpy opencv-python

# 4. Instalar la librería de puente ROS2 Python
sudo apt install ros-humble-cv-bridge python3-cv-bridge -y

# 5. Verificar la instalación
python3 -c "import h5py; import numpy; import cv2; print('Python dependencies OK')"
```

### Compilación del Espacio de Trabajo de las Cámaras RealSense

```bash
# 1. Entrar al espacio de trabajo de las cámaras
cd ~/code/cobot_magic_ros2/camera_ws

# 2. Instalar dependencias
sudo apt install ros-humble-realsense2-camera-msgs -y

# 3. Compilar
colcon build

# 4. Verificar que la compilación fue exitosa
source install/setup.bash
ros2 pkg list | grep realsense
# Deberías ver:
# realsense2_camera
# realsense2_camera_msgs
# realsense2_description
```

### Compilación del Espacio de Trabajo del Brazo Piper

```bash
# 1. Entrar al espacio de trabajo de Piper
cd ~/code/cobot_magic_ros2/Piper_ros2_humble

# 2. Compilar
colcon build

# 3. Verificar que la compilación fue exitosa
source install/setup.bash
ros2 pkg list | grep piper
# Deberías ver:
# piper
# piper_msgs
```

### Configuración del Mapeo de Dispositivos CAN

Antes de iniciar los brazos robóticos, es necesario configurar la relación de mapeo entre los dispositivos CAN y los brazos izquierdo y derecho.

```bash
cd ~/code/cobot_magic_ros2/Piper_ros2_humble

# 1. Ver el script de configuración CAN actual
cat can_config.sh

# 2. Insertar el primer dispositivo USB-CAN y ver su dirección USB
sudo ethtool -i can0 | grep bus
# Anotar la dirección mostrada en bus-info, por ejemplo: 1-8:1.0

# 3. Insertar el segundo dispositivo USB-CAN en un puerto USB diferente y ver la dirección
sudo ethtool -i can1 | grep bus
# Anotar la dirección mostrada en bus-info, por ejemplo: 1-9.1:1.0

# 4. Editar can_config.sh y modificar el mapeo de USB_PORTS
# Busca estas líneas y reemplázalas con las direcciones USB reales:
#   USB_PORTS["1-8:1.0"]="can_left:1000000"
#   USB_PORTS["1-9.1:1.0"]="can_right:1000000"
```

**Notas Importantes:**
- `can_left` corresponde al brazo robótico izquierdo.
- `can_right` corresponde al brazo robótico derecho.
- Las direcciones USB deben coincidir con los puertos USB donde se insertaron físicamente.
- La tasa de baudios se establece en 1000000 (1Mbps).

### Configuración del Mapeo de Números de Serie de las Cámaras

```bash
cd ~/code/cobot_magic_ros2/camera_ws/src/realsense-ros/realsense2_camera/launch

# 1. Ver los números de serie de las cámaras conectadas
rs-enumerate-devices | grep "Serial Number"
# Anotar los números de serie de las tres cámaras

# 2. Editar el archivo de lanzamiento de múltiples cámaras
nano multi_camera.launch.py
# O usa tu editor preferido

# 3. En el archivo, busca la sección de configuración de cámaras y modifica los números de serie por los valores reales:
#   camera_left:   serial_no='<número_serie_cámara_izq>'
#   camera_middle: serial_no='<número_serie_cámara_centro>'
#   camera_right:  serial_no='<número_serie_cámara_der>'
```

### Lista de Verificación de Configuración del Entorno

Antes de comenzar la recolección de datos, confirma los siguientes puntos:

- [ ] ROS2 Humble está instalado y se puede ejecutar `ros2 --version`
- [ ] Intel RealSense SDK está instalado y se puede ejecutar `realsense-viewer`
- [ ] Herramientas CAN instaladas y se puede ejecutar `candump --help`
- [ ] Dependencias de Python instaladas (h5py, dm_env, numpy, opencv-python)
- [ ] camera_ws compilado exitosamente
- [ ] Piper_ros2_humble compilado exitosamente
- [ ] Las tres cámaras RealSense están conectadas y son reconocidas por `rs-enumerate-devices`
- [ ] Los dos dispositivos USB-CAN están conectados y configurados en `can_config.sh`
- [ ] Los números de serie de las cámaras están configurados en `multi_camera.launch.py`
- [ ] El brazo maestro y el esclavo están conectados mediante el cable de aviación

Una vez completados estos pasos, el entorno está configurado y puedes proceder al flujo de recolección de datos.

---

## I. Preparación y Verificación del Sistema

### 1.1 Verificación de Conexiones de Hardware

```bash
# Verificar que las tres cámaras RealSense estén conectadas
rs-enumerate-devices
# Deberías ver los números de serie de las tres cámaras,
# luego modifica los números de serie en el archivo camera_ws\src\realsense-ros\realsense_camera\launch\multi_camera.launch.py

# Verificar dispositivos CAN
lsusb | grep -i can
# Deberías ver dos dispositivos USB a CAN; puedes determinar cuál es cuál desconectando y conectando, luego modifica los parámetros correspondientes en can_config.sh
```

### 1.2 Instalación de Dependencias Necesarias

```bash
# Dependencias de ROS2
sudo apt install ros-humble-realsense2-camera ros-humble-cv-bridge

# Herramientas de comunicación CAN
sudo apt install can-utils ethtool

# Dependencias de Python
pip3 install h5py dm_env numpy opencv-python
```

---

## II. Configuración de Dispositivos CAN (Doble Brazo)

**⚠️ IMPORTANTE**: Se deben configurar puertos CAN independientes para los brazos izquierdo y derecho.

### 2.1 Método de Configuración

```bash
cd cobot_magic_ros2/Piper_ros2_humble
```

---

## III. Inicio de las Tres Cámaras

### 3.1 Inicio del Nodo de Cámara

Abre la **Terminal 1**:

```bash
cd cobot_magic_ros2/camera_ws
source install/setup.bash  

# Iniciar las tres cámaras RealSense (izquierda, centro, derecha)
ros2 launch realsense2_camera multi_camera.launch.py
```

### 3.2 Verificación del Inicio de las Cámaras

Abre una nueva terminal:

```bash
# Verificar los tópicos de las cámaras
ros2 topic list | grep camera

# Deberías ver:
# /camera_left/color/image_raw
# /camera_left/depth/image_rect_raw
# /camera_middle/color/image_raw
# /camera_middle/depth/image_rect_raw
# /camera_right/color/image_raw
# /camera_right/depth/image_rect_raw
```

---

## IV. Inicio del Doble Brazo Robótico

### 4.1 Inicio del Nodo del Brazo Robótico

Abre la **Terminal 2**:

```bash
cd Piper_ros2_humble
bash can_config.sh
source install/setup.bash

# Iniciar el sistema de doble brazo maestro-esclavo (modo 0: modo de recolección de datos)
ros2 launch piper start_ms_piper.launch.py mode:=0 auto_enable:=false
```

### 4.2 Descripción de Parámetros

- `mode:=0` - Modo de recolección de datos, lee los estados de las articulaciones del brazo maestro y esclavo.
- `auto_enable:=false` - No encender automáticamente (más seguro).

### 4.3 Requisitos de Preparación del Hardware

- ✅ El brazo maestro y el esclavo están conectados mediante el **cable de aviación**.
- ✅ El brazo esclavo **seguirá los movimientos del brazo maestro** (modo teleoperación).
- ✅ Asegurar que haya suficiente espacio de actividad alrededor de los brazos robóticos.

### 4.4 Verificación del Inicio del Brazo Robótico

Abre una nueva terminal:

```bash
# Verificar los tópicos del brazo robótico
ros2 topic list | grep -E "(master|puppet)"

# Deberías ver:
# /master/joint_left          # Estado de articulación brazo maestro izquierdo
# /master/joint_right         # Estado de articulación brazo maestro derecho
# /puppet/joint_left          # Estado de articulación brazo esclavo izquierdo
# /puppet/joint_right         # Estado de articulación brazo esclavo derecho
# /puppet/end_pose_left       # Pose del extremo final brazo esclavo izquierdo
# /puppet/end_pose_right      # Pose del extremo final brazo esclavo derecho

# Verificar la frecuencia de publicación de datos de articulación
ros2 topic hz /puppet/joint_left
# Debería haber una publicación de datos constante

# Ver el estado de la articulación en tiempo real
ros2 topic echo /master/joint_left --once
```

---

## V. Inicio del Script de Recolección de Datos

### 5.1 Comando de Recolección Básico

Abre la **Terminal 3**:

```bash
cd /home/yiqun/code/cobot_magic_ros2

# Comando de recolección básico (guarda imágenes RGB)
python3 collect_data/collect_data.py \
    --dataset_dir ~/robot_data \
    --task_name place_shoe \
    --episode_idx 0 \
    --max_timesteps 500 \
    --arm_type piper \
    --frame_rate 30 \
    --use_depth_image False \
    --use_forward_kinematics True
```

### 5.2 Comando de Recolección de Alta Calidad

```bash
# Recolección de alta calidad (incluye mapas de profundidad y cinemática)
python3 collect_data/collect_data.py \
    --dataset_dir ~/robot_data \
    --task_name grab_cup \
    --episode_idx 0 \
    --max_timesteps 800 \
    --arm_type piper \
    --frame_rate 30 \
    --use_depth_image True \
    --use_forward_kinematics True
```

### 5.3 Descripción de Parámetros

| Parámetro | Descripción | Ejemplo |
|------------|-------------|---------|
| `--dataset_dir` | Directorio donde se guardan los datos (se crea automáticamente) | `~/robot_data` |
| `--task_name` | Nombre de la tarea | `place_shoe` |
| `--episode_idx` | Índice del episodio actual en el conjunto de datos (comienza en 0) | `0, 1, 2...` |
| `--max_timesteps` | Número máximo de frames a recolectar | `500` (aprox. 17 seg @ 30fps) |
| `--arm_type` | Tipo de brazo robótico | `piper` |
| `--frame_rate` | Tasa de frames de recolección | `30` |
| `--use_depth_image` | Recolectar imágenes de profundidad | `True/False` |
| `--use_forward_kinematics` | Calcular cinemática directa y extrínsecos de la cámara | `True/False` |

### 5.4 Lista de Tareas Predefinidas

| Nombre de la Tarea | Descripción de la Tarea |
|-------------------|-------------------------|
| `place_shoe` | Agarrar un zapato con un brazo y colocarlo sobre la alfombrilla |
| `pick_cup` | Agarrar una taza y elevarla a la altura objetivo |
| `open_door` | Agarrar el pomo y abrir la puerta |
| `close_drawer` | Agarrar el pomo y empujar hasta cerrar completamente |
| `wipe_table` | Limpiar el área marcada con una esponja haciendo movimientos circulares |
| `plug_cable` | Recoger el conector e insertarlo en la toma |
| `fold_towel` | Recoger la toalla y doblarla a lo largo de la línea central |
| `pour_water` | Agarrar la botella y verter el agua en la taza |

---

## VI. Proceso de Recolección de Datos

### 6.1 Flujo de Recolección

**1. Tras iniciar el script**, se mostrará:
```
[32mParámetros DH del brazo Piper cargados (7 articulaciones + extremo)[0m
[32mSistema de coordenadas global: sistema de base del brazo izquierdo (base_link)[0m
[32mDesplazamiento base brazo izq: [0. 0. 0.][0m
[32mDesplazamiento base brazo der: [ 0.  -0.6  0. ][0m
Esperando sincronización de datos...
```

**2. Una vez completada la sincronización**, comienza la recolección:
```
Iniciando recolección de datos (Presiona Ctrl+C para detener prematuramente)
Progreso: 1/500 frames
Progreso: 2/500 frames
...
```

**3. Durante la recolección**:
- Manipula manualmente el **brazo maestro** para ejecutar la tarea.
- El **brazo esclavo** seguirá automáticamente los movimientos del brazo maestro.
- Las tres cámaras graban simultáneamente imágenes RGB/profundidad.

**4. Finalización de la recolección**:
```
¡Recolección completada! Total recolectado: 500 frames
Guardando datos en: /home/yiqun/robot_data/place_shoe/episode_0.hdf5
¡Datos guardados exitosamente!
```

### 6.2 Detención Prematura

Si necesitas finalizar la recolección antes de tiempo, presiona `Ctrl+C`; los datos recolectados hasta ese momento se guardarán.

---

## VII. Verificación de Datos

### 7.1 Ver los Archivos de Datos Recolectados

```bash
# Verificar los archivos de datos
ls -lh ~/robot_data/place_shoe/
# Deberías ver: episode_0.hdf5

# Verificar el tamaño del archivo
du -h ~/robot_data/place_shoe/episode_0.hdf5
```

### 7.2 Ver la Estructura del Archivo HDF5

```bash
python3 << 'EOF'
import h5py

with h5py.File('~/robot_data/place_shoe/episode_0.hdf5', 'r') as f:
    print("Estructura del dataset:")
    def print_structure(name, obj):
        print(name)
    f.visititems(print_structure)

    # Ver dimensiones de los datos
    print("\nDimensiones de los datos:")
    print(f"Posición articulaciones: {f['observations/qpos'].shape}")
    print(f"RGB Cámara Izq: {f['observations/camera_left/rgb'].shape}")
    print(f"RGB Cámara Centro: {f['observations/camera_middle/rgb'].shape}")
    print(f"RGB Cámara Der: {f['observations/camera_right/rgb'].shape}")
    print(f"Timestamps: {f['time_stamps'].shape}")

    # Ver metadatos
    print("\nMetadatos de la tarea:")
    print(f"Nombre de la tarea: {f['meta_data/task_name'][()]}")
    print(f"Total de frames: {f['meta_data/num_steps'][()]}")
EOF
```

### 7.3 Descripción de la Estructura de Datos

```
episode_0.hdf5
├── observations/
│   ├── qpos [N, 14]              # Posición de articulaciones de ambos brazos (6 art + 1 pinza por brazo)
│   ├── qvel [N, 14]              # Velocidad de articulaciones de ambos brazos
│   ├── effort [N, 14]            # Torque de articulaciones de ambos brazos
│   ├── end_pose_left [N, 7]      # Pose del extremo brazo izq (xyz + cuaternión)
│   ├── end_pose_right [N, 7]     # Pose del extremo brazo der
│   ├── camera_left/
│   │   ├── rgb [N, H, W, 3]      # Imágenes RGB cámara izq
│   │   ├── depth [N, H, W]       # Mapa de profundidad cámara izq (opcional)
│   │   ├── extrinsic [N, 4, 4]   # Matriz de extrínsecos de la cámara
│   │   └── intrinsic_cv [3, 3]  # Matriz de intrínsecos de la cámara
│   ├── camera_middle/           # Datos cámara centro (misma estructura que arriba)
│   └── camera_right/            # Datos cámara der (misma estructura que arriba)
├── meta_data/
│   ├── task_name                 # Nombre de la tarea
│   ├── instruction               # Descripción de la tarea
│   ├── user_name                 # Nombre de usuario
│   ├── uuid                      # Identificador único
│   └── num_steps                 # Total de frames
└── time_stamps [N]               # Secuencia de timestamps
```

---

## VIII. Recolecciones Múltiples (Conjuntos de Datos Masivos)

### 8.1 Ejemplo de recolección de múltiples episodios

```bash
# Episodio 0
python3 collect_data/collect_data.py \
    --dataset_dir ~/robot_data \
    --task_name place_shoe \
    --episode_idx 0 \
    --max_timesteps 500

# Episodio 1 (misma tarea, trayectoria diferente)
python3 collect_data/collect_data.py \
    --dataset_dir ~/robot_data \
    --task_name place_shoe \
    --episode_idx 1 \
    --max_timesteps 500

# Episodio 2
python3 collect_data/collect_data.py \
    --dataset_dir ~/robot_data \
    --task_name place_shoe \
    --episode_idx 2 \
    --max_timesteps 500
```

### 8.2 Estructura Final de Datos

```
~/robot_data/
├── place_shoe/
│   ├── episode_0.hdf5
│   ├── episode_1.hdf5
│   └── episode_2.hdf5
├── pick_cup/
│   ├── episode_0.hdf5
│   ├── episode_1.hdf5
│   └── episode_2.hdf5
└── open_door/
    ├── episode_0.hdf5
    └── episode_1.hdf5
```
