#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>

class BaseController {
    ros::NodeHandle n;
    ros::NodeHandle private_n;
    ros::Subscriber twist;
    ros::Subscriber collision;
    ros::Publisher vescMotor;
    ros::Publisher ackermannDir;
    std_msgs::Float64 msgMotor;
    std_msgs::Int32 msgAckermann;
    bool obstacle;

public:
    BaseController();
    void twistMsgCallback(const geometry_msgs::Twist::ConstPtr&);
    void collisionMsgCallback(const std_msgs::Int32::ConstPtr&);
    void motorDriver();
};