# Instrucciones para el uso de comandos

A continuación se presentan las instrucciones para utilizar diferentes comandos en el entorno de ROS (Robot Operating System). Estos comandos se utilizan para realizar diferentes tareas relacionadas con el control y la percepción de un robot TurtleBot en un mundo simulado.

## Lanzar mundo

**Título:** Lanzar mundo

**Descripción:** Este comando se utiliza para lanzar el mundo simulado en el que operará el robot TurtleBot.

**Comando:**

```
roslaunch turtlebot3_gazebo turtlebot3_puzzlebot_world_aruco.launch
```
## Lanzar ARUCO DETECT

**Título:** Lanzar ARUCO DETECT

**Descripción:** Este comando se utiliza para lanzar la detección de marcadores ARUCO. Los marcadores ARUCO son códigos bidimensionales que se utilizan para la detección y el seguimiento visual en robótica.

**Comando:**

```
roslaunch aruco_detect aruco_detect.launch
```

## Lanzar fiducial SLAM y RVIZ

**Título:** Lanzar fiducial SLAM y RVIZ

**Descripción:** Este comando se utiliza para lanzar el algoritmo SLAM (Simultaneous Localization and Mapping) basado en fiduciales, que permite estimar la posición del robot y construir un mapa del entorno utilizando marcadores fiduciales. También se lanza RVIZ, una herramienta de visualización en ROS.

**Comandos:**

```
roslaunch fiducial_slam fiducial_slam.launch
```

```
roslaunch fiducial_slam fiducial_rviz.launch
```
## Evitar errores del modelo

**Título:** Evitar errores del modelo

**Descripción:** Este comando se utiliza para evitar posibles errores relacionados con el modelo del robot al lanzar RVIZ y el fiducial SLAM. El comando ejecuta un nodo llamado "robot_state_publisher" que publica el estado del modelo del robot.

**Comando:**

```
rosrun robot_state_publisher robot_state_publisher
```

## Verificar el flujo de datos del filtro

**Título:** Verificar el flujo de datos del filtro

**Descripción:** Este comando se utiliza para verificar si el filtro de datos está recibiendo y procesando correctamente la información. El comando ejecuta un nodo llamado "kalman.py" del paquete "turtlebot_kalman".

**Comando:**

```
rosrun turtlebot_kalman kalman.py
```
