# Usa la imagen oficial de ROS 2 Jazzy con herramientas de escritorio
FROM osrf/ros:jazzy-desktop

# Instala todas las dependencias necesarias de una vez
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    net-tools \
    iputils-ping \
    tree \
    python3-matplotlib \
 && rm -rf /var/lib/apt/lists/*

# Crea y establece el directorio de trabajo para el workspace
WORKDIR /ros2_ws

# Copia tu código fuente al directorio 'src' del workspace en el contenedor
COPY src/ /ros2_ws/src/

# Activa el entorno de ROS 2 para poder usar sus comandos durante la construcción
SHELL ["/bin/bash", "-c"]
# Compila el proyecto con Colcon [cite: 344, 348]
RUN source /opt/ros/jazzy/setup.bash && colcon build

# Script de entrada: Ejecuta el script oficial de ROS y luego activa un terminal bash
# Esto asegura que el entorno de ROS esté listo cuando inicies el contenedor
CMD ["/ros_entrypoint.sh", "bash"]