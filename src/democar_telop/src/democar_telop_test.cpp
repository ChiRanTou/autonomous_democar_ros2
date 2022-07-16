/*小车模型的手动控制文件*/

//载入C++自带头文件
#include <iostream>
#include <string>
#include <math.h>

//载入ros相关头文件
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std;
//占位符
using std::placeholders::_1;

//继承Node类
class DemocarTelop : public rclcpp::Node
{
    public:
        //构造函数
        //Node中的构造函数也继承过来
        DemocarTelop(string name):Node(name)
        {
            //显示启动信息
            RCLCPP_INFO(this->get_logger(),"Starting Democar Telop Testing...");

            //设置发布控制指令的话题
            pub_cmd = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",1);

            //设置发布信息的话题
            pub_info = this->create_publisher<std_msgs::msg::String>("pub_location",1);

            //设置订阅的位姿信息
            sub_odom = this->create_subscription<nav_msgs::msg::Odometry>("odom",1,bind(&DemocarTelop::odom_callback,this,_1));

        }

        ~DemocarTelop()
        {
            RCLCPP_INFO(this->get_logger(),"Exit Democar Telop Testing...");
        }

    private:
        //订阅里程计的回调函数
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom);

        //制动函数
        void stopCar();

        //走直线
        void moveStraight();

        //转弯
        void goTurn();

        //转弯
        void moveAndTurn();     

        //申明一个用于发布控制指令的变量
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd;

        //申明一个用于发布信息的变量
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_info;

        //申明一个订阅位置的变量
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;
};

int main(int argc,char **argv)
{
    //初始化ros节点
    rclcpp::init(argc,argv);
    //产生一个节点
    //std::make_shared<类名>(节点名称)
    auto node = std::make_shared<DemocarTelop>("democar_telop");
    //运行这个节点
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;

}

/*对申明的函数进行定义*/
void DemocarTelop::odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom)
{
    //往前走
    this->goTurn();
    //获取往前走后的位姿
    float x = odom->pose.pose.position.x;
    float y = odom->pose.pose.position.y;
    float z = odom->pose.pose.position.z;
    float q0 = odom->pose.pose.orientation.x;
    float q1 = odom->pose.pose.orientation.y;
    float q2 = odom->pose.pose.orientation.z;
    float q3 = odom->pose.pose.orientation.w;
    //打印信息
    RCLCPP_INFO(this->get_logger(),"Position: x=%f ,y=%f, z=%f",x,y,z);
    RCLCPP_INFO(this->get_logger(),"Orientation: q0=%f , q1=%f, q2=%f, q3=%f",q0,q1,q2,q3);

}
        
//制动函数
void DemocarTelop::stopCar()
{
    geometry_msgs::msg::Twist cmd_vel;
    //设置制动值
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    //发布制动指令
    pub_cmd->publish(cmd_vel);
}

//走直线
void DemocarTelop::moveStraight()
{
    geometry_msgs::msg::Twist cmd_vel;
    rclcpp::Rate rate(0.25);

    cmd_vel.linear.x = 1.0;
    RCLCPP_INFO(this->get_logger(),"Start Go Straight...");
    pub_cmd->publish(cmd_vel);
    rate.sleep();
    stopCar();
    rate.sleep();
    RCLCPP_INFO(this->get_logger(),"Done Go Straight...");
}

//转弯
void DemocarTelop::goTurn()
{
    geometry_msgs::msg::Twist cmd_vel;
    rclcpp::Rate rate(0.25);
    cmd_vel.angular.z = 0.5;

    RCLCPP_INFO(this->get_logger(),"Start Go Turn...");
    pub_cmd->publish(cmd_vel);
    rate.sleep();
    stopCar();
    rate.sleep();
    RCLCPP_INFO(this->get_logger(),"Done Go Turn...");
}

//转弯
void DemocarTelop::moveAndTurn()
{
    geometry_msgs::msg::Twist cmd_vel;
    rclcpp::Rate rate(0.25);
    cmd_vel.linear.x = 0.5;
    cmd_vel.angular.z = 0.5;

    RCLCPP_INFO(this->get_logger(),"Start Move and Turn...");
    pub_cmd->publish(cmd_vel);
    rate.sleep();
    stopCar();
    rate.sleep();
    RCLCPP_INFO(this->get_logger(),"Done Move and Turn...");
}        