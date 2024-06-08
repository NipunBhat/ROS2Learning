from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    robot_station_1 = Node(
        package="my_cpp_pkg",
        executable="robot_news_station",
        name="robot_news_station_giskard",
        parameters=[{"robot_name": "giskard"}]
    )
    
    robot_station_2 = Node(
        package="my_cpp_pkg",
        executable="robot_news_station",
        name="robot_news_station_bb8",
        parameters=[{"robot_name": "bb8"}]
    )
    
    robot_station_3 = Node(
        package="my_cpp_pkg",
        executable="robot_news_station",
        name="robot_news_station_daneel",
        parameters=[{"robot_name": "daneel"}]
    )
    
    robot_station_4 = Node(
        package="my_cpp_pkg",
        executable="robot_news_station",
        name="robot_news_station_jander",
        parameters=[{"robot_name": "jander"}]
    )
    
    robot_station_5 = Node(
        package="my_cpp_pkg",
        executable="robot_news_station",
        name="robot_news_station_c3po",
        parameters=[{"robot_name": "c3po"}]
    )
    
    smart_phone_1 = Node(
        package="my_py_pkg",
        executable="smart_phone",
        name="smart_phone"
    )
    
    ld.add_action(robot_station_1)
    ld.add_action(robot_station_2)
    ld.add_action(robot_station_3)
    ld.add_action(robot_station_4)
    ld.add_action(robot_station_5)
    
    ld.add_action(smart_phone_1)
    
    return ld
