import os
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def get_package_source_directory():
    """Get the package source directory by finding workspace src directory."""
    # Get the absolute path of this launch file
    launch_file_path = os.path.abspath(__file__)
    current_dir = os.path.dirname(launch_file_path)
    
    # Walk up the directory tree to find workspace root (has 'src' directory)
    while current_dir != os.path.dirname(current_dir):  # Stop at filesystem root
        src_dir = os.path.join(current_dir, 'src')
        if os.path.exists(src_dir) and os.path.isdir(src_dir):
            # Found workspace root, look for package in src
            # Try to match package name from Node (walker) or use first found
            possible_names = ['walker', 'my_webots_tutorails']
            for pkg_name in possible_names:
                pkg_dir = os.path.join(src_dir, pkg_name)
                if os.path.exists(pkg_dir) and os.path.isdir(pkg_dir):
                    return pkg_dir
            
            # If neither found, use first directory in src
            src_contents = [d for d in os.listdir(src_dir) 
                           if os.path.isdir(os.path.join(src_dir, d)) and not d.startswith('.')]
            if src_contents:
                return os.path.join(src_dir, src_contents[0])
        
        current_dir = os.path.dirname(current_dir)
    
    # Fallback: use relative path from launch file
    return os.path.dirname(os.path.dirname(launch_file_path))


def generate_launch_description():
    package_dir = get_package_share_directory('walker')
    robot_description_path = os.path.join(package_dir, 'resource', 'walker_robot.urdf')

    # Launch arguments
    record_bag_arg = DeclareLaunchArgument(
        'record_bag',
        default_value='true',
        description='Enable rosbag recording (true/false)'
    )
    
    # Get the workspace source directory
    pkg_source_dir = get_package_source_directory()
    default_bag_dir = os.path.join(pkg_source_dir, "results", "walker_bag")
    
    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value=default_bag_dir,
        description='Path where rosbag files will be saved (default: results/walker_bag)'
    )

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'turtlebot3_burger_example.wbt')
    )

    walker_robot = WebotsController(
        robot_name='walker_robot',
        parameters=[
            {'robot_description': robot_description_path},
        ]
    )
    
    obstacle_avoider = Node(
        package='walker',
        executable='obstacle_avoider',
    )

    # Rosbag recording - records all topics except /camera/* topics
    # Note: ros2 bag record --exclude uses regex patterns
    rosbag_record = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('record_bag')),
        cmd=[
            'ros2', 'bag', 'record',
            '-a',  # Record all topics
            '--exclude', '/camera/.*',  # Exclude all camera topics (regex pattern)
            '-o', LaunchConfiguration('bag_path')  # Output directory (default: src/walker/results/)
        ],
        output='screen',
        shell=False
    )

    return LaunchDescription([
        record_bag_arg,
        bag_path_arg,
        webots,
        walker_robot,
        obstacle_avoider,
        rosbag_record,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
