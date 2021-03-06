# Base Controller

## Facultad de Ingenieria del Ejercito

Paquete ROS que mapea el input de un nodo geometry_msgs/twist a uno o mas nodos Vesc-ROS-FW-3.33 y direction (de Guillermo Muena), ambos requeridos. La finalidad es poder manejar una robot basado en estos drivers de motor.

### Requisitos

* ROS Melodic
* Alguna fuente de geometry_msgs/twist
* raess1/Vesc-ROS-FW-3.33
* carlosmgc2003/direction_ros
* Algun nodo que publique en /obstacle std_msgs/Int32

### Instalacion

```bash
cd ~/catkin_ws/src
git clone https://github.com/carlosmgc2003/base_controller.git
cd ..
catkin_make -DCATKIN_WHITELIST_PACKAGES="base_controller"

```

### Utilizacion
Iniciar los driver de VESC y Direction.
```bash
rosrun base_controller base_controller_node
```
### Para utilizar con un Joystick

Previamente a iniciar el base controller:
```bash
rosrun joy joy_node
rosrun teleop_twist_joy teleop_node _axis_angular:=2 _enable_button:=7 _scale_linear:=0.3 _scale_angular:=1.0
```
### Para utilizar con un teclado

Previamente a iniciar el base controller:
```bash
sudo apt install ros-melodic-teleop-twist-keyboard
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```


### Para frenarlo simulando el mensaje de un sensor (ej: LIDAR)

```bash
rostopic pub /obstacle std_msgs/Int32 "data: 1"
```
