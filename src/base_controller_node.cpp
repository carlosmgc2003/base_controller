
#include "../include/base_controller.hpp"

BaseController::BaseController() {
    // Manejador del Nodo
    n = ros::NodeHandle();
    // Instancia privada solamente para acceder a los parametros
    private_n = ros::NodeHandle("~");
    // Inicializacion de las subscripciones
    twist = n.subscribe<geometry_msgs::Twist>("cmd_vel", 10, &BaseController::twistMsgCallback, this);
    collision = n.subscribe<std_msgs::Int32>("obstacle", 10, &BaseController::collisionMsgCallback, this);
    endOfRace = n.subscribe<std_msgs::Int32>("end_of_race", 10, &BaseController::endOfRaceMsgCallback, this);
    
    // Inicializacion de las publicaciones
    vescMotor =  n.advertise<std_msgs::Float64>("commands/motor/duty_cycle", 10);
    ackermannDir = n.advertise<std_msgs::Int32>("direction",10);

    // Inicializacion de las variables de estado
    this->msgMotor.data = 0.0;
    this->msgAckermann.data = 0;
    this->obstacle = false;
    this->right_eor = false;
    this->left_eor = false;
}

void BaseController::twistMsgCallback(const geometry_msgs::Twist::ConstPtr& msg){
    // Tomo la magnitud del vector velocidad
    this->msgMotor.data = msg->linear.x;
    // Tomo la varicion en azimut del vector velocidad 
    double direction = msg->angular.z;
    // Discretizo el azimut [-1, 0, 1]
    if(direction > 0.0) {
        this->msgAckermann.data = LEFT_END;
    } else if(direction < 0.0){
        this->msgAckermann.data = RIGHT_END;
    } else {
        this->msgAckermann.data = 0;
    }
    // Teniendo los datos del Joystick llamamos a motor driver a ver que hacemos.
    // De esta manera solo se impacta en los motores si realmente hay ordenes
    motorDriver();
}
void BaseController::collisionMsgCallback(const std_msgs::Int32::ConstPtr& msg){
    // Si se detecta obstaculo se levanta el flag correspondiente.
    if(msg->data == 1) {
        this->obstacle = true;
    } else {
        this->obstacle = false;
    }
}

void BaseController::endOfRaceMsgCallback(const std_msgs::Int32::ConstPtr& msg){
    // Si llega el mensaje de final de carrera se levanta el flag correspondiente
    if(msg->data == -1) {
        this->left_eor = true;
    } else if (msg->data == 1) {
        this->right_eor = true;
    }
}


void BaseController::motorDriver(){
    // Magia: hacemos un spinOnce para recoger todos los callback antes de mandar la orden
    // a los motores. Esto impide que nos perdamos de algun evento importante.
    ros::spinOnce();
    // Si no hay obstaculo
    if(!this->obstacle) {
        // Publico directo al motor
        vescMotor.publish(this->msgMotor);
    } else {
        // Si hay obstaculo solo puedo retroceder.
        if(this->msgMotor.data > 0.0){
            ROS_INFO("No avanzo mas! Obstaculo al frente!");
            this->msgMotor.data = 0.0;
        }
        vescMotor.publish(this->msgMotor);
    }

    // Si llegue al EOR y quiero seguir doblando a la derecha
    if(this->right_eor && this->msgAckermann.data == RIGHT_END) {
        ROS_INFO("No giro mas! Limite de Giro a la Derecha!");
        // No giro mas
        this->msgAckermann.data = 0;
        // Si llegue al EOR y quiero seguir doblando a la izquierda
    } else if(this->left_eor && this->msgAckermann.data == LEFT_END) {
        // No giro mas
        this->msgAckermann.data = 0;
        ROS_INFO("No giro mas! Limite de Giro a la Izquierda!");
    }
    // Si no hay levantado ningun EOR, mando nomas el mensaje de girar.
    ackermannDir.publish(this->msgAckermann);

    // Reset de las flags de EOR.
    this->right_eor = false;
    this->left_eor = false;
}

int main(int argc, char **argv) {
    ros::init(argc,argv,"base_controller");
    BaseController BC = BaseController();
    ros::spin();
    return 0;
}