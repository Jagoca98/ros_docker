from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PythonExpression, EnvironmentVariable

#https://roboticscasual.com/tutorial-ros2-launch-files-all-you-need-to-know/

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='test_package_py',
            executable='talker',
            name='my_publisher',
            output='screen',
            emulate_tty=True,
            namespace='my_ns1',
            parameters=[
                {'my_parameter': 'earth'}
            ],
            arguments=[
                'turtlesim_ns', 'turtlesim2',
                'use_provided_red', PythonExpression(["'", EnvironmentVariable('USER'),"' == 'alex'"]),
            ]
        ),
        Node(
            package='test_package_py',
            executable='listener',
            name='my_sub',
            output='screen',
            emulate_tty=True,
            namespace='my_ns2',
        ),
    ])