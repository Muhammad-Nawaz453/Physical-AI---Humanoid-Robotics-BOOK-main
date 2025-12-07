---
sidebar_position: 8
---

# 🤖 Project: Build and Simulate a Complete Robot

This capstone project for Module 2 integrates all the concepts you've learned about simulation environments, physics, sensors, and robot description formats. You'll build a complete simulated robot from scratch, including its physical model, sensor suite, and a testing environment. This project demonstrates the full pipeline from robot design to simulation testing.

## 🎯 Learning Objectives

By the end of this project, you will:
- Design a complete robot model with proper kinematics and dynamics
- Integrate multiple sensor types into a single robot platform
- Create a realistic testing environment for robot validation
- Validate robot behavior through simulation testing
- Implement basic control algorithms for navigation and interaction
- Document your robot design and testing methodology

## Project Overview: Autonomous Warehouse Robot

For this project, you'll create an autonomous warehouse robot capable of:
- Navigating through warehouse environments
- Detecting and avoiding obstacles
- Identifying and approaching target objects
- Operating safely in dynamic environments

The robot will include:
- Differential drive base for mobility
- RGB-D camera for visual perception
- 2D LiDAR for navigation and mapping
- IMU for orientation and stability
- Proper physical properties for realistic simulation

## Phase 1: Robot Design and Modeling

### Robot Specification

Let's define our warehouse robot specifications:

- **Base**: Differential drive with two main wheels and one caster wheel
- **Dimensions**: 0.8m length × 0.6m width × 0.5m height
- **Weight**: 25kg (including payload)
- **Sensors**: RGB-D camera, 2D LiDAR, IMU
- **Payload**: 10kg maximum
- **Speed**: 1.0 m/s maximum linear velocity

### Complete Robot URDF

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="warehouse_robot">

  <!-- Properties -->
  <xacro:property name="base_width" value="0.6"/>
  <xacro:property name="base_length" value="0.8"/>
  <xacro:property name="base_height" value="0.3"/>
  <xacro:property name="base_mass" value="20"/>
  <xacro:property name="wheel_radius" value="0.15"/>
  <xacro:property name="wheel_width" value="0.05"/>
  <xacro:property name="wheel_mass" value="1.0"/>
  <xacro:property name="caster_radius" value="0.05"/>
  <xacro:property name="caster_mass" value="0.2"/>

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="${base_length} ${base_width} ${base_height}"/>
      </geometry>
      <material name="blue">
        <color rgba="0.1 0.1 0.8 1"/>
      </material>
    </visual>

    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="${base_length} ${base_width} ${base_height}"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="${base_mass}"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia
        ixx="${base_mass * (base_width*base_width + base_height*base_height) / 12}"
        ixy="0" ixz="0"
        iyy="${base_mass * (base_length*base_length + base_height*base_height) / 12}"
        iyz="0"
        izz="${base_mass * (base_length*base_length + base_width*base_width) / 12}"/>
    </inertial>
  </link>

  <!-- Wheel macro -->
  <xacro:macro name="wheel" params="prefix parent xyz rpy joint_xyz">
    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="${parent}"/>
      <child link="${prefix}_wheel"/>
      <origin xyz="${joint_xyz}" rpy="${rpy}"/>
      <axis xyz="0 1 0"/>
      <dynamics damping="0.1" friction="0.01"/>
    </joint>

    <link name="${prefix}_wheel">
      <visual>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
        <material name="black">
          <color rgba="0 0 0 1"/>
        </material>
      </visual>

      <collision>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </collision>

      <inertial>
        <mass value="${wheel_mass}"/>
        <inertia
          ixx="${wheel_mass * (3*wheel_radius*wheel_radius + wheel_width*wheel_width) / 12}"
          ixy="0" ixz="0"
          iyy="${wheel_mass * (3*wheel_radius*wheel_radius + wheel_width*wheel_width) / 12}"
          iyz="0"
          izz="${wheel_mass * wheel_radius * wheel_radius / 2}"/>
      </inertial>
    </link>

    <!-- Transmission for ROS control -->
    <transmission name="${prefix}_wheel_trans">
      <type>transmission_interface/SimpleTransmission</type>
      <joint name="${prefix}_wheel_joint">
        <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
      </joint>
      <actuator name="${prefix}_wheel_motor">
        <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
        <mechanicalReduction>1</mechanicalReduction>
      </actuator>
    </transmission>
  </xacro:macro>

  <!-- Instantiate wheels -->
  <xacro:wheel prefix="left" parent="base_link"
               xyz="${base_length/2} ${base_width/2} 0"
               rpy="0 0 0"
               joint_xyz="${-wheel_radius} ${base_width/2} 0"/>
  <xacro:wheel prefix="right" parent="base_link"
               xyz="${base_length/2} ${-base_width/2} 0"
               rpy="0 0 0"
               joint_xyz="${-wheel_radius} ${-base_width/2} 0"/>

  <!-- Caster wheel -->
  <joint name="caster_joint" type="fixed">
    <parent link="base_link"/>
    <child link="caster_wheel"/>
    <origin xyz="${-base_length/2} 0 ${-caster_radius}" rpy="0 0 0"/>
  </joint>

  <link name="caster_wheel">
    <visual>
      <geometry>
        <sphere radius="${caster_radius}"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <sphere radius="${caster_radius}"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="${caster_mass}"/>
      <inertia
        ixx="${2*caster_mass*caster_radius*caster_radius/5}"
        ixy="0" ixz="0"
        iyy="${2*caster_mass*caster_radius*caster_radius/5}"
        iyz="0"
        izz="${2*caster_mass*caster_radius*caster_radius/5}"/>
    </inertial>
  </link>

  <!-- Camera sensor -->
  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="${base_length/2 - 0.05} 0 ${base_height/2 + 0.1}" rpy="0 0 0"/>
  </joint>

  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0 0 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- LiDAR sensor -->
  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar_link"/>
    <origin xyz="0 0 ${base_height/2 + 0.15}" rpy="0 0 0"/>
  </joint>

  <link name="lidar_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.05"/>
      </geometry>
      <material name="green">
        <color rgba="0 0.8 0 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.05"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.0002" ixy="0" ixz="0" iyy="0.0002" iyz="0" izz="0.0004"/>
    </inertial>
  </link>

  <!-- IMU sensor -->
  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <origin xyz="0 0 ${base_height/2 - 0.05}" rpy="0 0 0"/>
  </joint>

  <link name="imu_link">
    <inertial>
      <mass value="0.01"/>
      <inertia ixx="0.000001" ixy="0" ixz="0" iyy="0.000001" iyz="0" izz="0.000001"/>
    </inertial>
  </link>

</robot>
```

## Phase 2: Sensor Integration

### Sensor Configuration in SDF for Gazebo

Now let's create an SDF version with proper sensor plugins for Gazebo simulation:

```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <model name="warehouse_robot">
    <link name="chassis">
      <pose>0 0 0.3 0 0 0</pose>
      <inertial>
        <mass>20.0</mass>
        <inertia>
          <ixx>0.8</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>1.2</iyy>
          <iyz>0</iyz>
          <izz>1.6</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.8 0.6 0.3</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.8 0.6 0.3</size>
          </box>
        </geometry>
        <material>
          <ambient>0.1 0.1 0.8 1</ambient>
          <diffuse>0.1 0.1 0.8 1</diffuse>
        </material>
      </visual>
    </link>

    <!-- Left wheel -->
    <link name="left_wheel">
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.005</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.005</iyy>
          <iyz>0</iyz>
          <izz>0.01</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>0.15</radius>
            <length>0.05</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>0.15</radius>
            <length>0.05</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0 0 0 1</ambient>
          <diffuse>0 0 0 1</diffuse>
        </material>
      </visual>
    </link>

    <!-- Right wheel -->
    <link name="right_wheel">
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.005</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.005</iyy>
          <iyz>0</iyz>
          <izz>0.01</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>0.15</radius>
            <length>0.05</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>0.15</radius>
            <length>0.05</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0 0 0 1</ambient>
          <diffuse>0 0 0 1</diffuse>
        </material>
      </visual>
    </link>

    <!-- Joints -->
    <joint name="left_wheel_joint" type="revolute">
      <parent>chassis</parent>
      <child>left_wheel</child>
      <pose>-0.15 0.3 0 0 0 0</pose>
      <axis>
        <xyz>0 1 0</xyz>
        <limit>
          <effort>100</effort>
          <velocity>10</velocity>
        </limit>
      </axis>
    </joint>

    <joint name="right_wheel_joint" type="revolute">
      <parent>chassis</parent>
      <child>right_wheel</child>
      <pose>-0.15 -0.3 0 0 0 0</pose>
      <axis>
        <xyz>0 1 0</xyz>
        <limit>
          <effort>100</effort>
          <velocity>10</velocity>
        </limit>
      </axis>
    </joint>

    <!-- Camera sensor -->
    <link name="camera_link">
      <pose>0.35 0 0.25 0 0 0</pose>
      <inertial>
        <mass>0.1</mass>
        <inertia>
          <ixx>0.0001</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.0001</iyy>
          <iyz>0</iyz>
          <izz>0.0001</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.05 0.05 0.05</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.05 0.05 0.05</size>
          </box>
        </geometry>
        <material>
          <ambient>0.8 0 0 1</ambient>
          <diffuse>0.8 0 0 1</diffuse>
        </material>
      </visual>
    </link>

    <joint name="camera_joint" type="fixed">
      <parent>chassis</parent>
      <child>camera_link</child>
      <pose>0.35 0 0.25 0 0 0</pose>
    </joint>

    <!-- LiDAR sensor -->
    <link name="lidar_link">
      <pose>0 0 0.3 0 0 0</pose>
      <inertial>
        <mass>0.2</mass>
        <inertia>
          <ixx>0.0002</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.0002</iyy>
          <iyz>0</iyz>
          <izz>0.0004</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>0.05</radius>
            <length>0.05</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>0.05</radius>
            <length>0.05</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0 0.8 0 1</ambient>
          <diffuse>0 0.8 0 1</diffuse>
        </material>
      </visual>
    </link>

    <joint name="lidar_joint" type="fixed">
      <parent>chassis</parent>
      <child>lidar_link</child>
      <pose>0 0 0.3 0 0 0</pose>
    </joint>

    <!-- Camera plugin -->
    <sensor name="camera" type="camera">
      <pose>0.35 0 0.25 0 0 0</pose>
      <camera>
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>10</far>
        </clip>
      </camera>
      <always_on>1</always_on>
      <update_rate>30</update_rate>
      <visualize>true</visualize>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <ros>
          <namespace>camera</namespace>
          <remapping>image_raw:=image</remapping>
        </ros>
        <camera_name>rgb_camera</camera_name>
        <image_topic_name>image_raw</image_topic_name>
        <camera_info_topic_name>camera_info</camera_info_topic_name>
        <frame_name>camera_link</frame_name>
      </plugin>
    </sensor>

    <!-- LiDAR plugin -->
    <sensor name="lidar" type="ray">
      <pose>0 0 0.3 0 0 0</pose>
      <ray>
        <scan>
          <horizontal>
            <samples>720</samples>
            <resolution>1</resolution>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.1</min>
          <max>30.0</max>
        </range>
      </ray>
      <always_on>1</always_on>
      <update_rate>10</update_rate>
      <visualize>false</visualize>
      <plugin name="lidar_controller" filename="libgazebo_ros_laser.so">
        <topicName>scan</topicName>
        <frameName>lidar_link</frameName>
      </plugin>
    </sensor>

    <!-- IMU plugin -->
    <sensor name="imu_sensor" type="imu">
      <pose>0 0 0.1 0 0 0</pose>
      <always_on>1</always_on>
      <update_rate>100</update_rate>
      <plugin name="imu_controller" filename="libgazebo_ros_imu.so">
        <ros>
          <namespace>imu</namespace>
          <remapping>~/out:=imu/data</remapping>
        </ros>
        <topicName>imu/data</topicName>
        <serviceName>imu/service</serviceName>
        <gaussianNoise>0.0017</gaussianNoise>
        <updateRate>100.0</updateRate>
      </plugin>
    </sensor>

    <!-- Differential drive plugin -->
    <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
      <ros>
        <namespace>robot</namespace>
        <remapping>cmd_vel:=cmd_vel</remapping>
        <remapping>odom:=odom</remapping>
      </ros>
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>0.6</wheel_separation>
      <wheel_diameter>0.3</wheel_diameter>
      <command_topic>cmd_vel</command_topic>
      <odometry_topic>odom</odometry_topic>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>chassis</robot_base_frame>
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
      <publish_wheel_tf>false</publish_wheel_tf>
    </plugin>
  </model>
</sdf>
```

## Phase 3: Environment Creation

### Warehouse Testing Environment

Create a comprehensive warehouse environment for testing:

```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <world name="warehouse_world">
    <!-- Physics -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- Lighting -->
    <light name="warehouse_light_1" type="point">
      <pose>0 0 5 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <attenuation>
        <range>15</range>
        <constant>0.09</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
    </light>

    <light name="warehouse_light_2" type="point">
      <pose>5 0 5 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <attenuation>
        <range>15</range>
        <constant>0.09</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
    </light>

    <!-- Ground -->
    <model name="floor">
      <static>true</static>
      <link name="floor_link">
        <collision name="collision">
          <geometry>
            <box>
              <size>20 15 0.1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>20 15 0.1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.6 0.6 0.6 1</ambient>
            <diffuse>0.6 0.6 0.6 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Walls -->
    <model name="warehouse_walls">
      <static>true</static>
      <link name="walls">
        <collision name="front_wall">
          <pose>0 7.5 1.5 0 0 0</pose>
          <geometry>
            <box>
              <size>20 0.1 3</size>
            </box>
          </geometry>
        </collision>
        <collision name="back_wall">
          <pose>0 -7.5 1.5 0 0 0</pose>
          <geometry>
            <box>
              <size>20 0.1 3</size>
            </box>
          </geometry>
        </collision>
        <collision name="left_wall">
          <pose>-10 0 1.5 0 0 0</pose>
          <geometry>
            <box>
              <size>0.1 15 3</size>
            </box>
          </geometry>
        </collision>
        <collision name="right_wall">
          <pose>10 0 1.5 0 0 0</pose>
          <geometry>
            <box>
              <size>0.1 15 3</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>

    <!-- Shelves -->
    <model name="shelf_1">
      <pose>-8 5 0.5 0 0 0</pose>
      <link name="shelf_link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.5 3 1.5</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.5 3 1.5</size>
            </box>
          </geometry>
          <material>
            <ambient>0.4 0.2 0 1</ambient>
            <diffuse>0.4 0.2 0 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>50</mass>
          <inertia>
            <ixx>5</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>5</iyy>
            <iyz>0</iyz>
            <izz>5</izz>
          </inertial>
        </inertial>
      </link>
    </model>

    <model name="shelf_2">
      <pose>-8 -5 0.5 0 0 0</pose>
      <link name="shelf_link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.5 3 1.5</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.5 3 1.5</size>
            </box>
          </geometry>
          <material>
            <ambient>0.4 0.2 0 1</ambient>
            <diffuse>0.4 0.2 0 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>50</mass>
          <inertia>
            <ixx>5</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>5</iyy>
            <iyz>0</iyz>
            <izz>5</izz>
          </inertial>
        </inertial>
      </link>
    </model>

    <!-- Moving obstacle -->
    <model name="moving_obstacle">
      <link name="obstacle_link">
        <pose>0 0 0.5 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>0.5 0.5 1.0</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.5 0.5 1.0</size>
            </box>
          </geometry>
          <material>
            <ambient>1 0.5 0 1</ambient>
            <diffuse>1 0.5 0 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>10</mass>
          <inertia>
            <ixx>1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>1</iyy>
            <iyz>0</iyz>
            <izz>1</izz>
          </inertia>
        </inertial>
      </link>
      <plugin name="sine_motion" filename="libgazebo_ros_p3d.so">
        <always_on>true</always_on>
        <update_rate>100</update_rate>
        <body_name>obstacle_link</body_name>
        <topic_name>obstacle_pose</topic_name>
        <gaussian_noise>0.0</gaussian_noise>
        <frame_name>world</frame_name>
      </plugin>
    </model>

    <!-- Target object -->
    <model name="target_object">
      <pose>8 3 0.1 0 0 0</pose>
      <link name="target_link">
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.1</radius>
              <length>0.3</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.1</radius>
              <length>0.3</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>1 0 1 1</ambient>
            <diffuse>1 0 1 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0.01</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.01</iyy>
            <iyz>0</iyz>
            <izz>0.005</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- Robot spawn point -->
    <model name="robot_spawn">
      <pose>-9 0 0.1 0 0 0</pose>
      <link name="spawn_link">
        <visual name="visual">
          <geometry>
            <sphere>
              <radius>0.05</radius>
            </sphere>
          </geometry>
          <material>
            <ambient>0 1 0 1</ambient>
            <diffuse>0 1 0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

## Phase 4: Control and Navigation

### Basic Navigation Controller

Create a Python script for basic navigation and obstacle avoidance:

```python
#!/usr/bin/env python3
"""
Basic navigation controller for warehouse robot
"""
import rospy
import tf
from geometry_msgs.msg import Twist, Point, Pose
from sensor_msgs.msg import LaserScan, Image, Imu
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np
from math import pow, atan2, sqrt

class WarehouseRobotController:
    def __init__(self):
        rospy.init_node('warehouse_robot_controller', anonymous=True)

        # Robot state
        self.position = Point()
        self.pose = Pose()
        self.twist = Twist()
        self.laser_data = None
        self.image_data = None
        self.imu_data = None

        # Navigation parameters
        self.target_x = 8.0  # Target x coordinate
        self.target_y = 3.0  # Target y coordinate
        self.obstacle_threshold = 0.8  # Minimum distance to obstacles
        self.linear_speed = 0.5
        self.angular_speed = 0.5

        # Publishers and subscribers
        self.velocity_publisher = rospy.Publisher('/robot/cmd_vel', Twist, queue_size=10)
        self.odom_subscriber = rospy.Subscriber('/robot/odom', Odometry, self.odom_callback)
        self.laser_subscriber = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.image_subscriber = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        self.imu_subscriber = rospy.Subscriber('/imu/data', Imu, self.imu_callback)

        self.bridge = CvBridge()

        # Rate
        self.rate = rospy.Rate(10)  # 10 Hz

    def odom_callback(self, data):
        """Update robot position from odometry"""
        self.position = data.pose.pose.position
        self.pose = data.pose.pose

        # Convert quaternion to euler angles for orientation
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)
        self.yaw = yaw

    def laser_callback(self, data):
        """Process laser scan data"""
        self.laser_data = data

    def image_callback(self, data):
        """Process camera image data"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.image_data = cv_image
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge error: {e}")

    def imu_callback(self, data):
        """Process IMU data"""
        self.imu_data = data

    def euclidean_distance(self, goal_x, goal_y):
        """Calculate Euclidean distance to goal"""
        return sqrt(pow((goal_x - self.position.x), 2) + pow((goal_y - self.position.y), 2))

    def linear_vel(self, goal_x, goal_y, constant=1.5):
        """Calculate linear velocity to goal"""
        distance = self.euclidean_distance(goal_x, goal_y)
        return constant * distance if distance > 0.1 else 0

    def steering_angle(self, goal_x, goal_y):
        """Calculate desired steering angle"""
        return atan2(goal_y - self.position.y, goal_x - self.position.x)

    def angular_vel(self, goal_x, goal_y, constant=6):
        """Calculate angular velocity to goal"""
        angle_to_goal = self.steering_angle(goal_x, goal_y)
        return constant * (angle_to_goal - self.yaw)

    def obstacle_detected(self):
        """Check if obstacles are detected in front of robot"""
        if self.laser_data is None:
            return False

        # Check the front 30 degrees of the laser scan
        front_scan = self.laser_data.ranges[len(self.laser_data.ranges)//2-15:len(self.laser_data.ranges)//2+15]

        # Filter out invalid readings (inf, nan)
        valid_distances = [d for d in front_scan if d > 0 and d < float('inf')]

        if not valid_distances:
            return False

        min_distance = min(valid_distances)
        return min_distance < self.obstacle_threshold

    def avoid_obstacle(self):
        """Simple obstacle avoidance behavior"""
        twist = Twist()

        # Turn left to avoid obstacle
        twist.linear.x = 0.0
        twist.angular.z = self.angular_speed
        return twist

    def navigate_to_goal(self):
        """Navigate to target position with obstacle avoidance"""
        twist = Twist()

        if self.obstacle_detected():
            rospy.loginfo("Obstacle detected, avoiding...")
            return self.avoid_obstacle()

        # Calculate velocities to reach goal
        distance_to_goal = self.euclidean_distance(self.target_x, self.target_y)

        if distance_to_goal > 0.1:  # If not close to goal
            twist.linear.x = min(self.linear_vel(self.target_x, self.target_y), self.linear_speed)
            twist.angular.z = self.angular_vel(self.target_x, self.target_y)
        else:
            # Reached goal
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            rospy.loginfo("Goal reached!")

        return twist

    def run(self):
        """Main control loop"""
        rospy.loginfo("Starting warehouse robot navigation...")

        while not rospy.is_shutdown():
            # Get control command
            cmd_vel = self.navigate_to_goal()

            # Publish command
            self.velocity_publisher.publish(cmd_vel)

            # Log status
            distance_to_goal = self.euclidean_distance(self.target_x, self.target_y)
            rospy.loginfo(f"Position: ({self.position.x:.2f}, {self.position.y:.2f}), "
                         f"Distance to goal: {distance_to_goal:.2f}, "
                         f"Obstacle detected: {self.obstacle_detected()}")

            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = WarehouseRobotController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
```

## Phase 5: Testing and Validation

### Simulation Testing Script

Create a comprehensive test script to validate robot performance:

```python
#!/usr/bin/env python3
"""
Simulation testing script for warehouse robot
"""
import rospy
import unittest
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
import time
import numpy as np

class RobotTestSuite(unittest.TestCase):
    def __init__(self):
        rospy.init_node('robot_test_suite', anonymous=True)

        # Test variables
        self.odom_data = None
        self.laser_data = None
        self.imu_data = None
        self.test_results = {}

        # Subscribers
        self.odom_sub = rospy.Subscriber('/robot/odom', Odometry, self.odom_callback)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.imu_sub = rospy.Subscriber('/imu/data', Imu, self.imu_callback)

        self.cmd_pub = rospy.Publisher('/robot/cmd_vel', Twist, queue_size=10)

        self.rate = rospy.Rate(10)

    def odom_callback(self, data):
        self.odom_data = data

    def laser_callback(self, data):
        self.laser_data = data

    def imu_callback(self, data):
        self.imu_data = data

    def test_sensor_functionality(self):
        """Test that all sensors are publishing data"""
        rospy.loginfo("Testing sensor functionality...")

        # Wait for sensor data
        timeout = time.time() + 60*2  # 2 minutes timeout
        while (self.laser_data is None or self.imu_data is None) and time.time() < timeout:
            self.rate.sleep()

        # Check if we received data
        self.assertIsNotNone(self.laser_data, "Laser scanner not publishing")
        self.assertIsNotNone(self.imu_data, "IMU not publishing")

        # Check laser data validity
        self.assertGreater(len(self.laser_data.ranges), 0, "Laser scanner has no ranges")
        self.assertGreater(self.laser_data.range_max, 0, "Laser scanner max range is invalid")

        rospy.loginfo("✓ Sensor functionality test passed")

    def test_robot_mobility(self):
        """Test basic robot mobility"""
        rospy.loginfo("Testing robot mobility...")

        # Store initial position
        initial_odom = self.odom_data
        if initial_odom is None:
            rospy.sleep(1.0)
            initial_odom = self.odom_data

        self.assertIsNotNone(initial_odom, "Odometry not available")

        initial_pos = initial_odom.pose.pose.position

        # Send forward command
        cmd = Twist()
        cmd.linear.x = 0.5  # Move forward
        self.cmd_pub.publish(cmd)

        # Wait for movement
        rospy.sleep(2.0)

        # Check new position
        final_odom = self.odom_data
        if final_odom is not None:
            final_pos = final_odom.pose.pose.position
            distance_moved = np.sqrt((final_pos.x - initial_pos.x)**2 + (final_pos.y - initial_pos.y)**2)

            self.assertGreater(distance_moved, 0.1, "Robot did not move significantly")
            rospy.loginfo(f"✓ Robot moved {distance_moved:.2f} meters")
        else:
            rospy.logwarn("Could not verify movement, odometry not updated")

        # Stop robot
        cmd.linear.x = 0.0
        self.cmd_pub.publish(cmd)

        rospy.loginfo("✓ Mobility test completed")

    def test_obstacle_detection(self):
        """Test obstacle detection capabilities"""
        rospy.loginfo("Testing obstacle detection...")

        if self.laser_data is None:
            rospy.sleep(1.0)

        self.assertIsNotNone(self.laser_data, "Laser data not available for obstacle detection test")

        if self.laser_data:
            # Check that we have valid range readings
            valid_ranges = [r for r in self.laser_data.ranges if r > 0 and r < float('inf')]
            self.assertGreater(len(valid_ranges), 0, "No valid range readings")

            # Check that ranges are within expected bounds
            for r in valid_ranges:
                self.assertLess(r, self.laser_data.range_max, f"Range {r} exceeds max range")
                self.assertGreater(r, self.laser_data.range_min, f"Range {r} below min range")

        rospy.loginfo("✓ Obstacle detection test passed")

    def run_all_tests(self):
        """Run all tests in sequence"""
        rospy.loginfo("Starting robot simulation tests...")

        try:
            self.test_sensor_functionality()
            self.test_robot_mobility()
            self.test_obstacle_detection()

            rospy.loginfo("🎉 All tests completed successfully!")
            return True

        except AssertionError as e:
            rospy.logerr(f"❌ Test failed: {str(e)}")
            return False
        except Exception as e:
            rospy.logerr(f"❌ Unexpected error during testing: {str(e)}")
            return False

def main():
    """Main test execution"""
    test_suite = RobotTestSuite()

    # Wait a bit for systems to initialize
    rospy.sleep(3.0)

    # Run tests
    success = test_suite.run_all_tests()

    if success:
        rospy.loginfo("All tests passed! Robot is ready for operation.")
    else:
        rospy.logerr("Some tests failed. Please check robot configuration.")

if __name__ == '__main__':
    main()
```

## Phase 6: Performance Analysis

### Robot Performance Dashboard

Create a simple dashboard to monitor robot performance:

```python
#!/usr/bin/env python3
"""
Robot performance monitoring dashboard
"""
import rospy
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
import numpy as np
import threading
import time

class RobotDashboard:
    def __init__(self):
        rospy.init_node('robot_dashboard', anonymous=True)

        # Data storage
        self.odom_history = []
        self.laser_history = []
        self.imu_history = []
        self.time_stamps = []

        # Subscribers
        rospy.Subscriber('/robot/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        rospy.Subscriber('/imu/data', Imu, self.imu_callback)

        # Setup plot
        self.fig, ((self.ax1, self.ax2), (self.ax3, self.ax4)) = plt.subplots(2, 2, figsize=(12, 8))
        self.fig.suptitle('Robot Performance Dashboard')

        # Position plot
        self.ax1.set_title('Robot Position (X-Y)')
        self.ax1.set_xlabel('X Position (m)')
        self.ax1.set_ylabel('Y Position (m)')
        self.pos_line, = self.ax1.plot([], [], 'b-', label='Path')
        self.ax1.legend()
        self.ax1.grid(True)

        # Laser scan plot
        self.ax2.set_title('Laser Scan')
        self.ax2.set_xlabel('Distance (m)')
        self.ax2.set_ylabel('Distance (m)')
        self.laser_line, = self.ax2.plot([], [], 'r.', markersize=1)
        self.ax2.grid(True)

        # IMU data plot
        self.ax3.set_title('IMU Orientation')
        self.ax3.set_xlabel('Time')
        self.ax3.set_ylabel('Angle (rad)')
        self.imu_roll_line, = self.ax3.plot([], [], 'r-', label='Roll')
        self.imu_pitch_line, = self.ax3.plot([], [], 'g-', label='Pitch')
        self.imu_yaw_line, = self.ax3.plot([], [], 'b-', label='Yaw')
        self.ax3.legend()
        self.ax3.grid(True)

        # Velocity plot
        self.ax4.set_title('Robot Velocity')
        self.ax4.set_xlabel('Time')
        self.ax4.set_ylabel('Velocity (m/s)')
        self.vel_linear_line, = self.ax4.plot([], [], 'r-', label='Linear')
        self.vel_angular_line, = self.ax4.plot([], [], 'b-', label='Angular')
        self.ax4.legend()
        self.ax4.grid(True)

        # Animation
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=100, blit=False)

        # Start plotting thread
        self.plot_thread = threading.Thread(target=self.run_plot)
        self.plot_thread.daemon = True
        self.plot_thread.start()

    def odom_callback(self, data):
        """Store odometry data"""
        current_time = time.time()
        self.odom_history.append({
            'x': data.pose.pose.position.x,
            'y': data.pose.pose.position.y,
            'linear_vel': np.sqrt(data.twist.twist.linear.x**2 + data.twist.twist.linear.y**2),
            'angular_vel': data.twist.twist.angular.z
        })
        self.time_stamps.append(current_time)

        # Keep only recent data (last 100 points)
        if len(self.odom_history) > 100:
            self.odom_history.pop(0)
            self.time_stamps.pop(0)

    def laser_callback(self, data):
        """Store laser data"""
        self.laser_history.append(data.ranges)

        # Keep only recent data
        if len(self.laser_history) > 10:
            self.laser_history.pop(0)

    def imu_callback(self, data):
        """Store IMU data"""
        # Convert quaternion to euler angles
        import tf
        orientation_q = data.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)

        self.imu_history.append({
            'roll': roll,
            'pitch': pitch,
            'yaw': yaw
        })

        # Keep only recent data
        if len(self.imu_history) > 100:
            self.imu_history.pop(0)

    def update_plot(self, frame):
        """Update all plots"""
        if self.odom_history:
            # Update position plot
            x_vals = [odom['x'] for odom in self.odom_history]
            y_vals = [odom['y'] for odom in self.odom_history]
            self.pos_line.set_data(x_vals, y_vals)

            # Update velocity plot
            times = list(range(len(self.odom_history)))
            linear_vels = [odom['linear_vel'] for odom in self.odom_history]
            angular_vels = [odom['angular_vel'] for odom in self.odom_history]
            self.vel_linear_line.set_data(times, linear_vels)
            self.vel_angular_line.set_data(times, angular_vels)

            # Adjust velocity plot limits
            if times:
                self.ax4.set_xlim(0, len(times)-1)

        if self.laser_history:
            # Update laser scan plot (most recent scan)
            ranges = self.laser_history[-1]
            angles = np.linspace(-np.pi, np.pi, len(ranges))
            x_scan = [r * np.cos(theta) for r, theta in zip(ranges, angles) if r > 0 and r < float('inf')]
            y_scan = [r * np.sin(theta) for r, theta in zip(ranges, angles) if r > 0 and r < float('inf')]
            self.laser_line.set_data(x_scan, y_scan)

            # Adjust laser plot limits
            if x_scan and y_scan:
                margin = 1.0
                self.ax2.set_xlim(min(x_scan) - margin, max(x_scan) + margin)
                self.ax2.set_ylim(min(y_scan) - margin, max(y_scan) + margin)

        if self.imu_history:
            # Update IMU plot
            times = list(range(len(self.imu_history)))
            rolls = [imu['roll'] for imu in self.imu_history]
            pitches = [imu['pitch'] for imu in self.imu_history]
            yaws = [imu['yaw'] for imu in self.imu_history]

            self.imu_roll_line.set_data(times, rolls)
            self.imu_pitch_line.set_data(times, pitches)
            self.imu_yaw_line.set_data(times, yaws)

            # Adjust IMU plot limits
            if times:
                self.ax3.set_xlim(0, len(times)-1)

        return [self.pos_line, self.laser_line, self.imu_roll_line,
                self.imu_pitch_line, self.imu_yaw_line,
                self.vel_linear_line, self.vel_angular_line]

    def run_plot(self):
        """Run the plot in a separate thread"""
        plt.show()

    def run(self):
        """Main run method"""
        rospy.loginfo("Robot Dashboard started. Close the plot window to exit.")
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("Dashboard stopped by user.")

if __name__ == '__main__':
    dashboard = RobotDashboard()
    dashboard.run()
```

## Project Execution Guide

### Setting Up the Project

1. **Create the robot model directory**:
```bash
mkdir -p ~/.gazebo/models/warehouse_robot
mkdir -p ~/.gazebo/models/warehouse_robot/meshes
mkdir -p ~/.gazebo/models/warehouse_robot/materials
```

2. **Save the robot model files**:
- Save the URDF as `~/.gazebo/models/warehouse_robot/model.urdf.xacro`
- Create a `model.config` file in the directory with model information

3. **Launch the simulation**:
```bash
# Launch Gazebo with the warehouse world
gazebo ~/.gazebo/models/warehouse_world.world

# In another terminal, launch the robot controller
rosrun warehouse_robot controller.py
```

### Testing Procedures

1. **Basic Functionality Test**:
   - Verify all sensors are publishing data
   - Test robot mobility in open space
   - Check odometry accuracy

2. **Navigation Test**:
   - Test path planning to target location
   - Verify obstacle avoidance behavior
   - Test operation in complex environments

3. **Performance Test**:
   - Monitor CPU and memory usage
   - Check simulation real-time factor
   - Validate sensor data quality

## 💡 Key Takeaways

- Integration of multiple systems (robot model, sensors, environment) requires careful planning
- Proper inertial properties are crucial for realistic simulation
- Sensor fusion enables robust robot perception and navigation
- Testing in simulation reduces risk and cost compared to physical testing
- Performance monitoring helps optimize robot design and control algorithms
- Documentation of the complete system enables reproducible research

## 🏋️ Hands-On Exercise

Complete the full robot simulation project by:
1. Building the complete robot model with all sensors
2. Creating a testing environment with obstacles and targets
3. Implementing navigation and obstacle avoidance algorithms
4. Validating robot performance through systematic testing
5. Documenting your design decisions and test results

**Expected Time:** 8 hours

**Requirements:**
- Gazebo and ROS/ROS2 installed
- Basic Python programming knowledge
- Understanding of robotics concepts

**Instructions:**
1. Implement the robot model following the specifications
2. Create the warehouse environment with realistic obstacles
3. Develop navigation algorithms for goal-seeking behavior
4. Test the complete system with various scenarios
5. Document performance metrics and improvement opportunities

**Solution Hints:**
- Start with a simple robot model and gradually add complexity
- Test each component individually before integration
- Use the provided code examples as starting points
- Validate sensor data quality before implementing algorithms
- Monitor simulation performance and optimize as needed

## 📚 Further Reading

- "Robotics, Vision and Control" by Peter Corke: Comprehensive robotics textbook
- [Gazebo Simulation Tutorials](http://gazebosim.org/tutorials): Official Gazebo documentation
- "Programming Robots with ROS" by Quigley et al.: ROS programming guide
- "Probabilistic Robotics" by Thrun et al.: Advanced robotics algorithms

---

**Congratulations!** You've completed Module 2: Gazebo & Unity. You now have the skills to create complete robotic simulation environments with realistic physics, sensors, and control systems.
