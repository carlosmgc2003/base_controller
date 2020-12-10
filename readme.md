# Base Controller

## Facultad de Ingenieria del Ejercito

Paquete ROS que mapea el input de un nodo geometry_msgs/twist a uno o mas nodos Vesc-ROS-FW-3.33 y direction (de Guillermo Muena), ambos requeridos. La finalidad es poder manejar una robot basado en estos drivers de motor.

### Requisitos

* ROS Melodic
* Alguna fuente de geometry_msgs/twist
* raess1/Vesc-ROS-FW-3.33
* carlosmgc2003/direction_ros

### Instalacion

```bash
TODO

```

### Utilizacion
```bash
rosrun base_controller base_controller_node
```
### Para utilizar con un Joystick
Previamente:
```bash
rosrun joy joy_node
rosrun teleop_twist_joy teleop_node _axis_angular:=3 _enable_button:=5 _scale_linear:=0.3 _scale_angular:=1.0
```

### Pendiente

