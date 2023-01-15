## Práctica 3 Robots Móviles

### ROS Noetic, Turtlebot3

Este repositorio contiene un paquete de ROS creado para el movimiento autónomo de un robot entre coordenadas aleatorias en un mapa, y la detección de landmarks con la cámara (color rojo). Para su funcionamiento, lanzar en la terminal:
```
roslaunch turtlebot3_gazebo turtlebot3_world.launch
roslaunch turtlebot3_navigation turtlebot3_navigation.launch
cd (carpeta donde esté el paquete pr3)
rosrun pr3 pr3_node.py
```
