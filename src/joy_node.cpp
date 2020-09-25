#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <ros/console.h>
#include <string>

class TeleopJoy
{
public:
    TeleopJoy();

private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);

    ros::NodeHandle nh_;

    int linear_, angular_;
    double l_scale_{0.5}, a_scale_{0.5};
    ros::Publisher vel_pub_;
    ros::Subscriber joy_sub_;
};

TeleopJoy::TeleopJoy() : linear_(1),
                         angular_(0)
{

    nh_.param("axis_linear", linear_, linear_);
    nh_.param("axis_angular", angular_, angular_);
    nh_.param("scale_angular", a_scale_, a_scale_);
    nh_.param("scale_linear", l_scale_, l_scale_);

    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 1, &TeleopJoy::joyCallback, this);
}

void TeleopJoy::joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
{
    geometry_msgs::Twist twist;
    twist.angular.z = a_scale_ * joy->axes[angular_];
    twist.linear.x = l_scale_ * joy->axes[linear_];
    twist.linear.y = 0.0;
    ROS_INFO("Twist msg:");
    ROS_INFO("x: [%f], y: [%f], z: [%f]", twist.linear.x, twist.linear.z, twist.angular.z);
    vel_pub_.publish(twist);
}

int main(int argc, char **argv)
{
    ROS_INFO("Init Node");
    ros::init(argc, argv, "teleop_turtle");
    TeleopJoy teleop_joy;

    ros::spin();
}