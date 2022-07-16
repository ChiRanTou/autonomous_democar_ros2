import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    #初始化路径信息
    pkg_name = 'democar_core'
    yaml_name = 'democar_purepursuit_settings.yaml'

    #创建启动文件描述对象
    ld = LaunchDescription()
    #获取yaml路径
    pkg_share = FindPackageShare(package=pkg_name).find(pkg_name)
    yaml_path = os.path.join(pkg_share,'config',yaml_name)

    pure_pursuit_node = Node(
        package='democar_core',
        executable='democar_purepursuit_node',
        name= 'pure_pursuit',
        parameters=[yaml_path],
        output='screen',
    )

    ld.add_action(pure_pursuit_node)

    return ld

