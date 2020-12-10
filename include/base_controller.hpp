#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>

const int RIGHT_END = 1;
const int LEFT_END = -1;

class BaseController {
    ros::NodeHandle n;
    ros::NodeHandle private_n;
    // Subscriptores
    ros::Subscriber twist;
    ros::Subscriber collision;
    ros::Subscriber endOfRace;
    // Publicadores
    ros::Publisher vescMotor;
    ros::Publisher ackermannDir;
    // Mensajes
    std_msgs::Float64 msgMotor;
    std_msgs::Int32 msgAckermann;
    // Variables de estado
    bool obstacle;
    bool right_eor;
    bool left_eor;

public:
    BaseController();
    void twistMsgCallback(const geometry_msgs::Twist::ConstPtr&);
    void collisionMsgCallback(const std_msgs::Int32::ConstPtr&);
    void endOfRaceMsgCallback(const std_msgs::Int32::ConstPtr&);
    void motorDriver();
};