import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name = 'democar_description'
    urdf_name = "demo_car.urdf"
    rviz_name = "democar.rviz"

    # 创建启动文件描述的变量
    ld = LaunchDescription()
    # 查找模型包的路径
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    # 通过系统中的索引确定模型位置
    urdf_model_path = os.path.join(pkg_share,f'urdf/{urdf_name}')
    rviz_path = os.path.join(pkg_share,f'config/{rviz_name}')

    # 发布机器人模型
    # 将joint_states数据转换成tf信息发布
    # package:包的名称
    # executable:可执行节点名称
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[urdf_model_path]
    )

    # 发布机器人关节数据信息
    # 通过joint_states话题发布
    # 这个会跳出一个gui来操作urdf
    # 而joint_state_publisher不会
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        arguments=[urdf_model_path]
    )
    
    # 通过rviz显示机器人信息
    # rviz节点
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[rviz_path],
        output='screen',
    )
    # joint_state_publisher发布joint_states给robot_state_publisher
    # robot_state_publihser发布robot_description给rviz
    # 把各节点添加到启动描述中
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(rviz2_node)

    return ld

