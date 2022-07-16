/*lqr controller for dynamics model*/

//导入C++自带头文件
#include <iostream>
#include <string>
#include <vector>
#include <utility>
#include <algorithm>
#include <math.h>
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"

//导入ros相关文件
#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

//自定义头文件
#include "loadPath.h"

using namespace std;
using namespace Eigen;
using std::placeholders::_1;

//lqr控制器对象
class LqrController : public rclcpp::Node
{
    public:
        //构造函数
        LqrController(string name);
    private:
        //初始化参数
        void initParameters();
        //加载路径点信息
        void loadPathInfo();
        //找到最近的路径点
        int findCLoestIndex();
        //误差计算模块
        void calStateError(const int index);
        //计算状态空间矩阵
        void calStateMatrix();
        //lqr迭代器
        void lqrSolver();
        //前馈控制
        double feedForwardControl();
        //最终控制
        double calSteering(const double steering_angle_feedforward);
        //发布全局路径
        void pubPath();
        //lqr控制函数主体
        double lqrContoll();
        //标准化转角
        //让整体转角保证在180度以内
        double normalizeAngle(const double angle);

        //里程计回调函数
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);

        //申明控制指令发布者
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd;
        //申明全局路径发布者
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path;
        //申明里程计订阅者
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;

        //小车物理属性
        //控制时间域
        double ts_;
        //左前轮质量 
        double mass_front_left_wheel;
        //右前轮质量
        double mass_front_right_wheel;
        //左后轮质量
        double mass_rear_left_wheel;
        //右后轮质量
        double mass_rear_right_wheel;
        //车身质量
        double mass_body;
        //整车质量
        double mass_;
        //车辆轴长
        double wheel_base;
        //前轮轴长
        double lf_;
        //后轮轴长
        double lr_;
        //前轮侧偏刚度
        double cf_;
        //后轮侧偏刚度
        double cr_;
        //z轴旋转惯量
        double izz_;
        //最大转向角
        double max_steer_angle;
        //期望速度
        double v_ref;

        //状态空间矩阵
        //状态大小
        const int basic_state_size = 4;
        //状态空间矩阵
        Eigen::MatrixXd a_mat_;
        //状态离散矩阵
        Eigen::MatrixXd a_mat_d_;
        //控制矩阵
        Eigen::MatrixXd b_mat_;
        //控制离散矩阵
        Eigen::MatrixXd b_mat_d_;
        //gain矩阵(LQR迭代出来的最后输出)
        Eigen::MatrixXd k_;
        //控制权重矩阵
        Eigen::MatrixXd r_;
        //状态权重矩阵
        Eigen::MatrixXd q_;
        //状态矩阵
        Eigen::MatrixXd state_mat;

        //状态误差
        double lateral_error;           //ed
        double heading_error;           //e_psi
        double lateral_error_rate;      //ed_dot
        double heading_error_rate;      //e_psi_dot

        //lqr设置
        //最大迭代次数
        int lqr_max_iteration = 200;
        //最小迭代误差
        double lqr_eps;

        //路径点容器
        vector<double> xr;
        vector<double> yr;
        vector<double> yawr;
        vector<double> kappar;

        //自身车辆位姿
        double x_car;
        double y_car;
        double yaw_car;
        double kappa_car;
        double linear_velocity;
        double heading_rate;

        //路径点数量
        int numOfWaypoints;

        //文件名称
        string pkg_name = "democar_core";
        string waypoints_name = "path2.csv";
        string waypoints_path;

        //定义全局路径信息
        nav_msgs::msg::Path base_path;
};

//构造函数
LqrController::LqrController(string name) : Node(name)
{
    RCLCPP_INFO(this->get_logger(),"Starting up lqr controller");

    //初始化各种参数
    initParameters();
    //读取路径点信息
    loadPathInfo();
    //全局路径发布者
    pub_path = this->create_publisher<nav_msgs::msg::Path>("base_path",1);
    //控制指令发布者
    pub_cmd = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",1);
    //里程计订阅者
    sub_odom = this->create_subscription<nav_msgs::msg::Odometry>("odom",1,bind(&LqrController::odom_callback,this,_1));
}

//初始化参数
void LqrController::initParameters()
{
    //申明参数变量
    this->declare_parameter<std::float_t>("car_length",wheel_base);                      //车轴长度
    this->declare_parameter<std::float_t>("front_left_wheel_mass",mass_front_left_wheel);     //左前轮质量
    this->declare_parameter<std::float_t>("front_right_wheel_mass",mass_front_right_wheel);   //右前轮质量
    this->declare_parameter<std::float_t>("rear_left_wheel_mass",mass_rear_left_wheel);       //左后轮质量
    this->declare_parameter<std::float_t>("rear_right_wheel_mass",mass_rear_right_wheel);     //右后轮质量
    this->declare_parameter<std::float_t>("body_mass",mass_body);                       //车身质量
    this->declare_parameter<std::float_t>("front_wheel_stiffiness",cf_);            //前轮侧偏刚度
    this->declare_parameter<std::float_t>("rear_wheel_stiffiness",cr_);             //后轮侧偏刚度
    this->declare_parameter<std::float_t>("control_period",ts_);                    //控制时间
    this->declare_parameter<std::float_t>("max_steer_angle",max_steer_angle);       //最大转向角
    this->declare_parameter<std::float_t>("lqr_epsilon",lqr_eps);                   //迭代误差
    this->declare_parameter<std::float_t>("reference_velocity",v_ref);              //参考速度
    
    //从yaml中获取车辆信息
    this->get_parameter("car_length",wheel_base); 
    this->get_parameter("front_left_wheel_mass",mass_front_left_wheel);     //左前轮质量
    this->get_parameter("front_right_wheel_mass",mass_front_right_wheel);   //右前轮质量
    this->get_parameter("rear_left_wheel_mass",mass_rear_left_wheel);       //左后轮质量
    this->get_parameter("rear_right_wheel_mass",mass_rear_right_wheel);     //右后轮质量
    this->get_parameter("body_mass",mass_body);                             //车身质量
    this->get_parameter("front_wheel_stiffiness",cf_);            //前轮侧偏刚度
    this->get_parameter("rear_wheel_stiffiness",cr_);             //后轮侧偏刚度
    this->get_parameter("control_period",ts_);                    //控制时间
    this->get_parameter("max_steer_angle",max_steer_angle);       //最大转向角
    this->get_parameter("lqr_epsilon",lqr_eps);                   //迭代误差
    this->get_parameter("reference_velocity",v_ref);                //参考速度

    //初始化各种矩阵
    a_mat_ = MatrixXd::Zero(basic_state_size,basic_state_size);
    a_mat_d_ = MatrixXd::Zero(basic_state_size,basic_state_size);
    b_mat_ = MatrixXd::Zero(basic_state_size,1);
    b_mat_d_ = MatrixXd::Zero(basic_state_size,1);
    k_ = MatrixXd::Zero(1,basic_state_size);
    r_ = MatrixXd::Identity(1,1);
    q_ = MatrixXd::Identity(basic_state_size,basic_state_size);
    q_(0,0) = 30;
    state_mat = MatrixXd::Zero(basic_state_size,1);

    //数据整合
    //计算整车质量
    double mass_front = mass_front_left_wheel + mass_front_right_wheel + mass_body /2.0;
    double mass_rear = mass_rear_left_wheel +mass_rear_right_wheel + mass_body / 2.0;
    mass_ = mass_front + mass_rear;
    //计算前后轮到车辆质心长度
    lf_ = wheel_base * (1.0 - mass_front / mass_);
    lr_ = wheel_base * (1.0 - mass_rear / mass_);
    //旋转惯量
    izz_ = lf_ * lf_ * mass_front + lr_ * lr_ * mass_rear;
    
    //角度换算 degree -> rad
    max_steer_angle *= M_PI/180.0;

    //获取路径文件名称
    string pkg_path = ament_index_cpp::get_package_share_directory(pkg_name);
    waypoints_path = pkg_path+"/waypoints/"+waypoints_name;

    RCLCPP_INFO(this->get_logger(),"waypoints file is loaded at directory: %s",waypoints_path.c_str());

}

//加载路径点信息
void LqrController::loadPathInfo()
{
    //创建路径加载器
    WaypointLoader wploader(waypoints_path);
    //获取读取是否成功的结果
    bool isFileLoaded = wploader.load_waypoints();
    if(!isFileLoaded)
    {
        RCLCPP_ERROR(this->get_logger(),"File is not exist or file is empty!");
        exit(1);
    }
    //读取路径点信息数据
    vector<vector<double>> wp = wploader.get_waypoints();
    for(vector<vector<double>>::const_iterator it=wp.begin();it!=wp.end();it++)
    {
        xr.push_back((*it)[0]);
        yr.push_back((*it)[1]);
        yawr.push_back((*it)[2]);
        kappar.push_back((*it)[3]);
    }
    numOfWaypoints = static_cast<int>(xr.size());
}
//找到最近的路径点
/*
    x:小车x坐标
    y:小车y坐标
    return：最近路径点下标
*/
int LqrController::findCLoestIndex()
{
    
    vector<double> dist; 
    //计算小车与路径点的距离
    for(int i=0;i<numOfWaypoints;i++)
    {
        dist.push_back(pow(xr[i]-x_car,2)+pow(yr[i]-y_car,2));
    }
    //获得最小值的位置指针
    auto smallest = min_element(dist.begin(),dist.end());
    //把位置指针转换成下标
    int index = distance(dist.begin(),smallest);
    return index;
}

//误差计算模块
/*
    index: 最近规划点下标
    x：小车x坐标
    y：小车y坐标
    yaw：小车航向角
*/
void LqrController::calStateError(const int index)
{

    double dx = x_car - xr[index];
    double dy = y_car - yr[index];

    //计算Frenet坐标下的距离误差
    //横向坐标误差
    lateral_error = dy*cos(yawr[index])-dx*sin(yawr[index]);    
    //航向误差
    heading_error = normalizeAngle(yaw_car - yawr[index]);
    //横向误差变化率
    lateral_error_rate = linear_velocity * sin(heading_error);
    //frenet坐标下的纵向速度
    double s_dot = linear_velocity*cos(heading_error)/(1-kappar[index]*lateral_error);
    //航向误差变化率
    heading_error_rate = heading_rate - kappar[index]*s_dot;
    //航行曲率
    kappa_car = kappar[index];

    //把误差放入矩阵中
    state_mat(0,0) = lateral_error;
    state_mat(1,0) = lateral_error_rate;
    state_mat(2,0) = heading_error;
    state_mat(3,0) = heading_error_rate;

}

//计算空间状态矩阵
void LqrController::calStateMatrix()
{   
    //状态矩阵
    /*
        [0,1,0,0;
        0,(-(c_f+c_r)/(m*v),(c_f+c_r)/m,-(c_f*l_f-c_r*l_r)/(m*v);
        0,0,0,1;
        0,(-(c_f*l_f-c_r*l_r))/(I_z*v),(c_f*l_f-c_r*l_r)/I_z,(-(c_f*l_f^2+c_r*l_r^2))/(I_z*v)];
    */
    a_mat_(0,1) = 1.0;
    a_mat_(1,1) = -(cf_ + cr_) / (mass_ * linear_velocity);
    a_mat_(1,2) = (cf_ + cr_) / (mass_);
    a_mat_(1,3) = -(cf_ * lf_ + cr_ * lr_) / (mass_ * linear_velocity);
    a_mat_(2,3) = 1.0;
    a_mat_(3,1) = -(cf_ * lf_ - cr_ * lr_) / (izz_ * linear_velocity);
    a_mat_(3,2) = (cf_ * lf_ - cr_ * lr_) / izz_;
    a_mat_(3,3) = -(cf_ * lf_ * lf_ + cr_ * cr_ * cr_) / (izz_ * linear_velocity);
    //离散化处理
    Eigen::MatrixXd i_mat = MatrixXd::Identity(basic_state_size,basic_state_size);
    a_mat_d_ = (i_mat - ts_ * 0.5 * a_mat_).inverse() * (i_mat + ts_ * 0.5 * a_mat_);

    //控制矩阵
    /*
        [0;
        cf/m;
        0;
        (cf*lf)/Iz]
    */
    b_mat_(1,0) = cf_ / mass_;
    b_mat_(3,0) = (cf_ * lf_) / izz_;
    //离散化处理
    b_mat_d_ = b_mat_ * ts_;

}

//lqr迭代器
void LqrController::lqrSolver()
{
    //设置初始p矩阵
    Eigen::MatrixXd p_;
    Eigen::MatrixXd p_old_ = q_;
    //设置一个初始误差值(取一个无限大的值)
    int i = 0 ;
    double error = std::numeric_limits<double>::max();
    //迭代求解p矩阵直到最小

    while (i++ < lqr_max_iteration && error>lqr_eps)
    {
        p_ = a_mat_d_.transpose() * p_old_ * a_mat_d_ 
            - a_mat_d_.transpose() * p_old_ * b_mat_d_ * (r_ + b_mat_d_.transpose() * p_old_ * b_mat_d_).inverse() * b_mat_d_.transpose() * p_old_ * a_mat_d_
            + q_;
        error = fabs((p_-p_old_).maxCoeff());
        p_old_ = p_;
    }
    k_ = (r_ + b_mat_d_.transpose() * p_ * b_mat_d_).inverse() * b_mat_d_.transpose() * p_ * a_mat_d_;

}

//前馈控制
double LqrController::feedForwardControl()
{
    double kv = lr_ * mass_/ cf_ / wheel_base - lf_ * mass_ / cr_ / wheel_base;
    double steering_angle_feedforward = wheel_base * kappa_car + kv * linear_velocity * linear_velocity * kappa_car - k_(0,2) * 
                                (lr_ * kappa_car - 
                                lf_ * mass_ * linear_velocity * linear_velocity * kappa_car  / 2 / wheel_base / cr_);
    return steering_angle_feedforward;
}

//最终控制
double LqrController::calSteering(const double steering_angle_feedforward)
{
    Eigen::MatrixXd temp_mat = k_ * state_mat;
    double steering_angle = -temp_mat(0,0) + steering_angle_feedforward; 

    //限幅
    if(steering_angle >= max_steer_angle)
    {
        steering_angle = max_steer_angle;
    }
    else if (steering_angle <= - max_steer_angle)
    {
        steering_angle = - max_steer_angle;
    }
    return steering_angle;
}

//lqr控制函数主体
double LqrController::lqrContoll()
{
    //计算最近规划点并返回其下标
    int index = findCLoestIndex();
    //计算状态误差
    calStateError(index);
    //计算状态空间矩阵
    calStateMatrix();
    //进行lqr迭代求最小代价下的control gain “k”
    lqrSolver();
    //计算前馈控制补偿
    double steering_angle_feedforward = feedForwardControl();
    //求最终的横向控制率
    double steering = calSteering(steering_angle_feedforward);

    return steering;
}

//标准化转角
//让整体转角保证在180度以内
double LqrController::normalizeAngle(const double angle)
{   
    //对角度取模
    double a = std::fmod(angle + M_PI, 2.0 * M_PI);
    if(a < 0.0)
    {
        a += (2.0 * M_PI);
    }
    return a - M_PI;
}

//发布全局路径点
void LqrController::pubPath()
{
    //申明一个poseStamped话题放置路径点
    geometry_msgs::msg::PoseStamped pose;

    //设置全局路径的时间戳
    base_path.header.stamp = this->get_clock()->now();
    //设置全局路径的id
    base_path.header.frame_id = "odom";
    //设置路径点的时间戳
    pose.header.stamp = this->get_clock()->now();
    //设置路径点的id
    pose.header.frame_id = "odom";
    //遍历读取出来的路径点
    //并放置到pose中
    for(int i=0;i<(int)xr.size();i++)
    {   
        //保存位置信息
        pose.pose.position.x = xr[i];
        pose.pose.position.y = yr[i];
        pose.pose.position.z = 0.0;

        //保存方向信息
        tf2::Quaternion q;
        q.setRPY(0,0,yawr[i]);
    
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();

        //把路径点一个个放到全局路径中
        base_path.poses.push_back(pose);
    }

    //发布全局路径
    pub_path->publish(base_path);
}

//里程计回调函数
void LqrController::odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
{

    //发布路径点
    pubPath();

    //获取当前小车的位姿
    x_car = odom_msg->pose.pose.position.x;         //x
    y_car = odom_msg->pose.pose.position.y;         //y
    tf2::Quaternion q_heading(
        odom_msg->pose.pose.orientation.x,
        odom_msg->pose.pose.orientation.y,
        odom_msg->pose.pose.orientation.z,
        odom_msg->pose.pose.orientation.w
    );
    //航向角
    //yaw_car已经在前面定义了
    double roll_car,pitch_car;
    tf2::Matrix3x3 m_heading(q_heading);
    m_heading.getRPY(roll_car,pitch_car,yaw_car);
    //速度
    double vx = odom_msg->twist.twist.linear.x;
    double vy = odom_msg->twist.twist.linear.y;
    linear_velocity = sqrt(pow(vx,2)+pow(vy,2));
    //航向率
    heading_rate = odom_msg->twist.twist.angular.z;
    //lqr控制
    //获得最终的转向角度
    double steering = lqrContoll();
    //发布动作信息
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = v_ref;
    cmd_vel.angular.z = steering;
    pub_cmd->publish(cmd_vel);

    RCLCPP_INFO(this->get_logger(),"Velocity = %f, Steering angle = %f",cmd_vel.linear.x,(cmd_vel.angular.z)*180.0/M_PI);

}

//主程序接口
int main(int argc,char **argv)
{
    //初始化ros
    rclcpp::init(argc,argv);

    //创建节点
    auto node = std::make_shared<LqrController>("lqr_controller");

    //启动节点
    rclcpp::spin(node);

    //关闭节点
    rclcpp::shutdown();

    return 0;

}