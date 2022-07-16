import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    #设置文件名
    pkg_name = 'democar_core'
    yaml_name = 'democar_lqr_settings.yaml'

    #创建启动文件描述
    ld = LaunchDescription()
    #获取文件路径信息
    pkg_share = FindPackageShare(package=pkg_name).find(pkg_name)
    yaml_path = os.path.join(pkg_share,'config',yaml_name)

    #启动节点
    democar_lqr_node = Node(
        package='democar_core',
        executable='democar_lqr_kinematics_node',
        name='lqr_controller',
        parameters=[yaml_path],
        output='screen',
    )

    ld.add_action(democar_lqr_node)

    return ld