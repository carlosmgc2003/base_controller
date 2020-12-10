
#include "../include/base_controller.hpp"

BaseController::BaseController() {
    // Manejador del Nodo
    n = ros::NodeHandle();
    // Instancia privada solamente para aceder a los parametros
    private_n = ros::NodeHandle("~");
    
    twist = n.subscribe<geometry_msgs::Twist>("cmd_vel", 10, &BaseController::twistMsgCallback, this);
    collision = n.subscribe<std_msgs::Int32>("obstacle", 10, &BaseController::collisionMsgCallback, this);
    endOfRace = n.subscribe<std_msgs::Int32>("end_of_race", 10, &BaseController::endOfRaceMsgCallback, this);
    vescMotor =  n.advertise<std_msgs::Float64>("commands/motor/duty_cycle", 10);
    ackermannDir = n.advertise<std_msgs::Int32>("direction",10);
    this->msgMotor.data = 0.0;
    this->msgAckermann.data = 0;
    this->obstacle = false;
    this->right_eor = false;
    this->left_eor = false;
}

void BaseController::twistMsgCallback(const geometry_msgs::Twist::ConstPtr& msg){
    this->msgMotor.data = msg->linear.x;
    double direction = msg->angular.z;
    if(direction > 0.0) {
        this->msgAckermann.data = LEFT_END;
    } else if(direction < 0.0){
        this->msgAckermann.data = RIGHT_END;
    } else {
        this->msgAckermann.data = 0;
    }
    // Teniendo los datos del Joystick llamamos a motor driver a ver que hacemos.
    
    motorDriver();
}
void BaseController::collisionMsgCallback(const std_msgs::Int32::ConstPtr& msg){
    if(msg->data == 1) {
        this->obstacle = true;
    } else {
        this->obstacle = false;
    }
}

void BaseController::endOfRaceMsgCallback(const std_msgs::Int32::ConstPtr& msg){
    if(msg->data == -1) {
        this->right_eor = true;
    } else if (msg->data == 1) {
        this->left_eor = true;
    }
}


void BaseController::motorDriver(){
    ros::spinOnce();
    if(!this->obstacle) {
        vescMotor.publish(this->msgMotor);
        
    } else {
        if(this->msgMotor.data > 0.0){
            ROS_INFO("No avanzo mas! Obstaculo al frente!");
            this->msgMotor.data = 0.0;
        }
        vescMotor.publish(this->msgMotor);
    }
    if(this->right_eor && this->msgAckermann.data == RIGHT_END) {
        ROS_INFO("No giro mas! Toque el fin de carrera derecho!");
        this->msgAckermann.data = 0;
    } else if(this->left_eor && this->msgAckermann.data == LEFT_END) {
        this->msgAckermann.data = 0;
        ROS_INFO("No giro mas! Toque el fin de carrera izquierdo!");
    }
    ackermannDir.publish(this->msgAckermann);
    this->right_eor = false;
    this->left_eor = false;
}

int main(int argc, char **argv) {
    ros::init(argc,argv,"base_controller");
    BaseController BC = BaseController();
    ros::spin();
    return 0;
}