#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "velocity_publisher");
    ros::NodeHandle nh;

    // 创建一个发布者，发布到/cmd_vel话题，消息类型为geometry_msgs::Twist
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // 设定发布频率
    ros::Rate loop_rate(10);  // 10 Hz

    while (ros::ok()) {
        // 创建并填充Twist消息
        geometry_msgs::Twist msg;
        msg.linear.x = 1.0;  // 设定线速度
        // msg.angular.z = 0.5;  // 设定角速度

        // 发布消息
        vel_pub.publish(msg);

        // 短暂休眠以维持循环频率
        loop_rate.sleep();
    }

    return 0;
}
