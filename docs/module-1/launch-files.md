---
sidebar_position: 7
---

# 🚀 Launch Files - Orchestrating Your ROS 2 System

**Launch files** in ROS 2 provide a powerful way to start multiple nodes simultaneously with specific configurations, parameters, and dependencies. They eliminate the need to manually launch each node and allow you to define complete robotic systems in a single, organized file. Mastering launch files is essential for deploying and managing complex ROS 2 applications.

## 🎯 Learning Objectives

By the end of this chapter, you will:
- Understand the structure and components of ROS 2 launch files
- Create launch files for single and multi-node systems
- Use launch arguments and conditional logic in launch files
- Configure nodes with parameters, remappings, and environment variables
- Integrate launch files with system services for autonomous startup

## 🏗️ Understanding Launch File Structure

Launch files in ROS 2 use Python as the configuration language, providing powerful scripting capabilities while maintaining readability. The core components include:

- **LaunchDescription**: The main container for launch actions
- **Node**: Defines a ROS 2 node to launch
- **Actions**: Various actions like setting parameters, remapping topics, etc.
- **LaunchArguments**: Configurable parameters for the launch file

:::tip Pro Tip
Launch files can include other launch files, enabling modular system composition and reuse of common configurations.
:::

### Basic Launch File Structure

```python
# basic_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    # Create a node
    my_node = Node(
        package='my_package',
        executable='my_node',
        name='my_node_name',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'robot_name': 'turtlebot4'}
        ],
        remappings=[
            ('original_topic', 'remapped_topic')
        ]
    )

    # Return the launch description
    return LaunchDescription([
        use_sim_time,
        my_node
    ])
```

## 🔧 Creating Complex Launch Files

### Multi-Node Launch with Dependencies

For complex robotic systems, you often need to launch multiple nodes with specific startup order and dependencies:

```python
# robot_system_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    # Robot description parameter
    robot_description = DeclareLaunchArgument(
        'robot_description',
        default_value=PathJoinSubstitution([
            FindPackageShare('my_robot_description'),
            'urdf',
            'robot.urdf.xacro'
        ]),
        description='Full path to robot description file to load'
    )

    # Robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'robot_description': LaunchConfiguration('robot_description')}
        ]
    )

    # Joint state publisher (starts after robot state publisher)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # Navigation node
    navigation_node = Node(
        package='my_navigation_package',
        executable='navigation_node',
        name='navigation_node',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            PathJoinSubstitution([
                FindPackageShare('my_navigation_package'),
                'config',
                'nav2_params.yaml'
            ])
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    # Camera node
    camera_node = Node(
        package='my_camera_package',
        executable='camera_node',
        name='camera_node',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'camera_fps': 30},
            {'camera_resolution': [640, 480]}
        ]
    )

    # Launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(use_sim_time)
    ld.add_action(robot_description)

    # Add nodes
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    ld.add_action(navigation_node)
    ld.add_action(camera_node)

    return ld
```

### Conditional Launch with Logic

Launch files support conditional logic for flexible system configurations:

```python
# conditional_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
import os

def launch_setup(context, *args, **kwargs):
    # Get launch arguments
    use_rviz = LaunchConfiguration('use_rviz')
    use_gazebo = LaunchConfiguration('use_gazebo')
    robot_model = LaunchConfiguration('robot_model')

    # List of actions to return
    actions = []

    # Add nodes based on conditions
    if LaunchConfiguration('use_rviz').perform(context) == 'true':
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(
                get_package_share_directory('my_robot_description'),
                'rviz',
                'robot_view.rviz'
            )],
            condition=IfCondition(use_rviz)
        )
        actions.append(rviz_node)

    # Conditional gazebo launch
    if LaunchConfiguration('use_gazebo').perform(context) == 'true':
        gazebo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('gazebo_ros'),
                    'launch',
                    'gazebo.launch.py'
                )
            ),
            condition=IfCondition(use_gazebo)
        )
        actions.append(gazebo_launch)

    # Robot model specific nodes
    if LaunchConfiguration('robot_model').perform(context) == 'turtlebot4':
        robot_node = Node(
            package='turtlebot4_node',
            executable='turtlebot4_node',
            name='turtlebot4_node'
        )
        actions.append(robot_node)
    elif LaunchConfiguration('robot_model').perform(context) == 'custom_robot':
        robot_node = Node(
            package='my_robot_driver',
            executable='robot_driver',
            name='robot_driver'
        )
        actions.append(robot_node)

    return actions

def generate_launch_description():
    # Declare launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz'
    )

    use_gazebo_arg = DeclareLaunchArgument(
        'use_gazebo',
        default_value='false',
        description='Whether to launch Gazebo simulation'
    )

    robot_model_arg = DeclareLaunchArgument(
        'robot_model',
        default_value='turtlebot4',
        choices=['turtlebot4', 'custom_robot'],
        description='Robot model to use'
    )

    # Create launch description with OpaqueFunction for conditional logic
    ld = LaunchDescription([
        use_rviz_arg,
        use_gazebo_arg,
        robot_model_arg
    ])

    # Add the function that returns additional actions
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
```

## 🎯 Advanced Launch Features

### Launch Substitutions and Expressions

Launch files support various substitutions for dynamic configuration:

```python
# advanced_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, TextSubstitution, EnvironmentVariable
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Various launch arguments
    robot_namespace = DeclareLaunchArgument(
        'robot_namespace',
        default_value='robot1',
        description='Robot namespace for multi-robot systems'
    )

    log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        choices=['debug', 'info', 'warn', 'error', 'fatal'],
        description='Logging level'
    )

    # Set environment variables
    set_ros_domain_id = SetEnvironmentVariable(
        name='ROS_DOMAIN_ID',
        value='1'  # Multi-robot domain separation
    )

    # Node with namespace and dynamic configuration
    robot_controller = Node(
        package='my_robot_control',
        executable='robot_controller',
        name='robot_controller',
        namespace=LaunchConfiguration('robot_namespace'),
        parameters=[
            {
                'robot_namespace': LaunchConfiguration('robot_namespace'),
                'log_level': LaunchConfiguration('log_level')
            }
        ],
        # Environment variables for this node
        additional_env={'RCUTILS_LOGGING_SEVERITY_THRESHOLD': LaunchConfiguration('log_level')},
        # Remap topics with namespace
        remappings=[
            ('cmd_vel', [LaunchConfiguration('robot_namespace'), '/cmd_vel']),
            ('odom', [LaunchConfiguration('robot_namespace'), '/odom'])
        ]
    )

    return LaunchDescription([
        set_ros_domain_id,
        robot_namespace,
        log_level,
        robot_controller
    ])
```

### Launch Files with YAML Configuration

For complex parameter configurations, you can load parameters from YAML files:

```python
# yaml_config_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch argument for config file
    config_file = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('my_robot_config'),
            'config',
            'robot_config.yaml'
        ]),
        description='Full path to config file'
    )

    # Create a group with namespace
    robot_group = GroupAction(
        actions=[
            # Push namespace for all nodes in this group
            PushRosNamespace(LaunchConfiguration('namespace')),

            # Robot controller with YAML config
            Node(
                package='my_robot_control',
                executable='robot_controller',
                name='robot_controller',
                parameters=[
                    LaunchConfiguration('config_file'),
                    {'use_sim_time': False}
                ],
                # Additional parameters that override YAML
                remappings=[
                    ('sensor_input', 'lidar_scan'),
                    ('control_output', 'cmd_vel')
                ]
            ),

            # Sensor processing node
            Node(
                package='my_sensor_package',
                executable='sensor_processor',
                name='sensor_processor',
                parameters=[LaunchConfiguration('config_file')]
            )
        ]
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='robot1',
        description='Robot namespace'
    )

    return LaunchDescription([
        config_file,
        namespace_arg,
        robot_group
    ])
```

## 🔍 Launch File Best Practices

### Modular Launch Files

Organize complex systems into modular launch files:

```python
# main_robot_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Main launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    # Include robot-specific launch file
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('my_robot_bringup'),
                'launch',
                'robot_drivers.launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    # Include navigation launch file
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('my_navigation_bringup'),
                'launch',
                'navigation.launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    # Include perception launch file
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('my_perception_bringup'),
                'launch',
                'perception.launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    return LaunchDescription([
        use_sim_time,
        robot_launch,
        navigation_launch,
        perception_launch
    ])
```

### Error Handling and Validation

Implement robust error handling in launch files:

```python
# robust_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit, OnProcessIO
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Launch arguments
    robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value='robot1',
        description='Name of the robot'
    )

    # Validate robot name
    validation_node = Node(
        package='my_robot_bringup',
        executable='config_validator',
        name='config_validator',
        parameters=[
            {'robot_name': LaunchConfiguration('robot_name')}
        ]
    )

    # Main robot node
    robot_node = Node(
        package='my_robot_package',
        executable='robot_node',
        name=[LaunchConfiguration('robot_name'), '_node'],
        parameters=[
            {'robot_name': LaunchConfiguration('robot_name')}
        ]
    )

    # Event handlers for monitoring node status
    start_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=robot_node,
            on_start=[
                LogInfo(msg=['Robot node started for: ', LaunchConfiguration('robot_name')])
            ]
        )
    )

    exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=robot_node,
            on_exit=[
                LogInfo(msg=['Robot node exited for: ', LaunchConfiguration('robot_name')])
            ]
        )
    )

    return LaunchDescription([
        robot_name,
        validation_node,
        robot_node,
        start_handler,
        exit_handler
    ])
```

## 🧪 Hands-On Exercise: Complete Robot System Launch

Create a launch file for a complete robot system with the following requirements:

**Expected Time:** 45 minutes

**Requirements:**
- Robot state publisher with URDF
- Navigation stack (Nav2)
- Perception pipeline (camera and lidar)
- RViz for visualization
- Simulation support (Gazebo)
- Parameter configuration for different robot models

**Instructions:**
1. Create a main launch file that includes subsystem launch files
2. Implement conditional logic for simulation vs real robot
3. Add parameter validation and error handling
4. Include RViz configuration with robot-specific settings
5. Test the launch file with different configurations

**Solution Hints:**
- Use modular launch files for each subsystem
- Implement namespace support for multi-robot systems
- Use YAML configuration files for complex parameters
- Add launch arguments for different operational modes

## 💡 Key Takeaways

- **Launch files** orchestrate complex ROS 2 systems with multiple nodes
- **Launch arguments** provide runtime configuration flexibility
- **Conditional logic** enables different system configurations
- **Modular design** promotes reusability and maintainability
- **Event handling** provides monitoring and error recovery capabilities

## 📚 Further Reading

- [ROS 2 Launch System](https://docs.ros.org/en/rolling/How-To-Guides/Launch-system.html)
- [Launch Files Tutorial](https://docs.ros.org/en/rolling/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)
- [Launch Arguments and Substitutions](https://docs.ros.org/en/rolling/How-To-Guides/Using-Substitutions.html)

---

**Next Chapter:** [Complete ROS 2 Package Project](/module-1/project-ros2-package)
