import os 
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 初始化各种模型、包信息名
    robot_name_in_model = 'democar'
    package_name = 'democar_description'
    # urdf_name = "demo_car.urdf"
    urdf_name = "demo_car_v2.urdf"
    rviz_name = "democar.rviz"

    # 创建用于启动的描述对象
    ld = LaunchDescription()
    # 加载模型文件路径和包路径
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    urdf_mode_path = os.path.join(pkg_share,f'urdf/{urdf_name}')
    rviz_path = os.path.join(pkg_share,f'config/{rviz_name}')

    # 开启Gazebo服务
    start_gazebo_cmd = ExecuteProcess(
        cmd = ['gazebo','--verbose','-s','libgazebo_ros_factory.so'],
        output='screen'
    )

    # 加载小车
    spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity',robot_name_in_model,'-file',urdf_mode_path],
        output='screen'
    )

    # 加载robot_state_publisher
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[urdf_mode_path]
    )

    # 启动rviz
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[rviz_path],
        output='screen'
    )

    # 把上面的指令添加到描述对象中
    ld.add_action(start_gazebo_cmd)
    ld.add_action(spawn_entity_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_rviz_cmd)

    return ld
