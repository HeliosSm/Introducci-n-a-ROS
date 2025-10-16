# Taller 2 - Redes de Sensores: Introducción a ROS 2 con Docker

Este repositorio contiene el desarrollo completo del **Taller 2 de Redes de Sensores**, cuyo objetivo fue implementar un entorno de trabajo basado en **ROS 2 Jazzy** dentro de un contenedor **Docker**, aplicando los principios de comunicación distribuida mediante **DDS (Data Distribution Service)** sobre el protocolo **RTPS (Real-Time Publish-Subscribe Protocol)**.

## 📘 Descripción General

El proyecto consistió en la creación de un **workspace ROS 2** dentro del contenedor, donde se desarrollaron tres nodos principales en Python:

- **sensor_node**: simula un sensor de temperatura que publica valores aleatorios entre 20°C y 30°C cada segundo.  
- **reader_node**: recibe y muestra por consola los datos publicados por el nodo sensor.  
- **plotter_node**: escucha los datos del sensor, genera un gráfico cada 5 segundos y guarda la imagen resultante en una carpeta compartida con el sistema anfitrión (`/ros2_ws/data`).

Además, se configuró un entorno de desarrollo completamente automatizado mediante un archivo `Dockerfile`, junto con los archivos de instalación y dependencias (`setup.py`, `package.xml`, `setup.cfg`), garantizando portabilidad y replicabilidad.

## ⚙️ Estructura del Proyecto

El espacio de trabajo (`ros2_ws`) contiene el paquete principal `sensor_program`, con la siguiente estructura:

```

ros2_ws/
│
├── src/
│   └── sensor_program/
│       ├── sensor_program/
│       │   ├── sensor_node.py
│       │   ├── reader_node.py
│       │   ├── plotter_node.py
│       │   └── **init**.py
│       ├── package.xml
│       ├── setup.py
│       └── setup.cfg
│
└── Dockerfile

````

## 🐳 Configuración del Contenedor

El contenedor se construye a partir de la imagen oficial `osrf/ros:jazzy-desktop`, incluyendo todas las dependencias necesarias para compilar y ejecutar el proyecto:

```bash
docker build -t ros2_taller2:v1 .
docker run -it --name ros2_ws -v "${PWD}/shared_data:/ros2_ws/data" ros2_taller2:v1 bash
````

Este comando monta una carpeta compartida que permite guardar y visualizar los resultados del nodo `plotter_node` directamente desde el sistema anfitrión.

## 🧩 Ejecución de los Nodos

Dentro del contenedor, se debe compilar el proyecto con:

```bash
cd /ros2_ws
colcon build
source install/setup.bash
```

Luego, se ejecutan los nodos en terminales separadas:

```bash
# Terminal 1
ros2 run sensor_program sensor_node

# Terminal 2
ros2 run sensor_program reader_node

# Terminal 3
ros2 run sensor_program plotter_node
```

El nodo `plotter_node` generará cada 5 segundos un archivo `sensor_plot.png` dentro del volumen compartido, mostrando el historial de temperatura recibido.

## 🧠 Análisis del Tráfico con Wireshark

Para validar la comunicación entre los nodos, se analizó el tráfico de red utilizando **Wireshark**. Se observaron los mensajes RTPS transmitidos a través de los puertos **7400–7500**, correspondientes a los canales de control y usuario de **DDS**, verificando la transmisión de mensajes tipo `DATA(p)` y `INFO_TS`.
Esto permitió evidenciar cómo **DDS** maneja el descubrimiento de participantes, la sincronización de datos y la transmisión confiable mediante el protocolo **UDP** en el modelo TCP/IP.

## 🧰 Tecnologías Utilizadas

* **ROS 2 Jazzy Jalisco**
* **Python 3**
* **Docker**
* **Matplotlib**
* **Wireshark**

## 📚 Conclusión

Este taller permitió implementar un entorno ROS 2 modular, automatizado y reproducible dentro de Docker, comprendiendo a fondo el flujo de comunicación distribuida entre nodos mediante DDS/RTPS.
El proyecto evidencia la importancia de la virtualización, la interoperabilidad y el análisis de red en el desarrollo de sistemas robóticos y de sensores en tiempo real.
Asimismo, el uso de contenedores garantiza portabilidad y control de versiones, mientras que el análisis de tráfico en Wireshark refuerza la comprensión práctica del modelo TCP/IP aplicado a sistemas distribuidos modernos.

---

**Autor:** Erick Ramón
**Correo:** [erick.ramon@ucuenca.edu.ec](mailto:erick.ramon@ucuenca.edu.ec)
**Universidad de Cuenca – Facultad de Ingeniería – Ingeniería en Telecomunicaciones**

```
```
