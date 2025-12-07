---
sidebar_position: 5
---

# 🤖 URDF Robot Description - Defining Your Robot's Physical Structure

**Unified Robot Description Format (URDF)** is the standard way to describe robots in ROS. It defines a robot's physical structure, including links (rigid bodies), joints (connections between links), and visual/inertial properties. Understanding URDF is crucial for simulating robots, planning movements, and integrating with tools like Gazebo and RViz.

## 🎯 Learning Objectives

By the end of this chapter, you will:
- Understand the fundamental components of URDF files
- Create complete robot models with multiple links and joints
- Define visual, collision, and inertial properties for robot parts
- Use Xacro to create parameterized and reusable robot descriptions
- Validate and visualize your URDF models

## 🏗️ Understanding URDF Structure

URDF (Unified Robot Description Format) is an XML-based format that describes robot structure. A URDF file contains:

- **Links**: Rigid bodies that make up the robot structure
- **Joints**: Connections between links with specific degrees of freedom
- **Visual**: How the robot looks in visualization tools
- **Collision**: How the robot interacts with the environment in simulation
- **Inertial**: Physical properties for dynamics simulation

:::tip Pro Tip
URDF is hierarchical - each robot has a single base link (root) with all other links connected through joints.
:::

### Basic URDF Components

Here's a simple URDF structure:

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Additional links and joints would go here -->
</robot>
```

## 🔧 Creating Your First URDF Robot

Let's create a simple wheeled robot with a base and four wheels:

```xml
<?xml version="1.0"?>
<robot name="simple_wheeled_robot">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
      <material name="light_grey">
        <color rgba="0.7 0.7 0.7 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Front right wheel -->
  <link name="front_right_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Joint connecting front right wheel to base -->
  <joint name="front_right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="front_right_wheel"/>
    <origin xyz="0.15 -0.15 -0.05" rpy="1.57075 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- Front left wheel -->
  <link name="front_left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="front_left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="front_left_wheel"/>
    <origin xyz="0.15 0.15 -0.05" rpy="1.57075 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- Rear wheels (similar to front wheels) -->
  <link name="rear_right_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="rear_right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="rear_right_wheel"/>
    <origin xyz="-0.15 -0.15 -0.05" rpy="1.57075 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <link name="rear_left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="rear_left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="rear_left_wheel"/>
    <origin xyz="-0.15 0.15 -0.05" rpy="1.57075 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- Optional: Add a sensor link like a camera -->
  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.2 0 0.1"/>
  </joint>
</robot>
```

## 🧱 Understanding Joint Types

URDF supports several joint types, each with different degrees of freedom:

- **Fixed**: No movement (0 DOF)
- **Continuous**: Rotation around one axis (1 DOF, unlimited)
- **Revolute**: Rotation around one axis with limits (1 DOF, limited)
- **Prismatic**: Linear motion along one axis (1 DOF)
- **Planar**: Motion on a plane (2 DOF)
- **Floating**: Free motion in 3D space (6 DOF)

### Joint Limits and Properties

```xml
<!-- Revolute joint with limits -->
<joint name="arm_joint" type="revolute">
  <parent link="base_link"/>
  <child link="arm_link"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
  <safety_controller k_position="20" k_velocity="400"
                    soft_lower_limit="-1.5" soft_upper_limit="1.5"/>
</joint>

<!-- Prismatic joint -->
<joint name="slider_joint" type="prismatic">
  <parent link="base_link"/>
  <child link="slider_link"/>
  <origin xyz="0.1 0 0" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>
  <limit lower="0" upper="0.5" effort="1000" velocity="2.0"/>
</joint>
```

## 🎨 Visual and Collision Properties

Visual and collision elements define how your robot appears and interacts with the environment:

```xml
<link name="complex_link">
  <!-- Visual properties (for display) -->
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <!-- Multiple geometry types supported -->
      <box size="0.1 0.1 0.1"/>
      <!-- Other options: <sphere radius="0.1"/>, <cylinder radius="0.1" length="0.2"/> -->
    </geometry>
    <material name="green_material">
      <color rgba="0 1 0 1"/>  <!-- Red, Green, Blue, Alpha -->
    </material>
  </visual>

  <!-- Collision properties (for physics simulation) -->
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.1 0.1 0.1"/>
    </geometry>
  </collision>

  <!-- Inertial properties (for dynamics) -->
  <inertial>
    <mass value="1.0"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <inertia ixx="0.01" ixy="0.0" ixz="0.0"
             iyy="0.01" iyz="0.0" izz="0.01"/>
  </inertial>
</link>
```

## 🔁 Using Xacro for Complex Robots

Xacro (XML Macros) allows you to create parameterized URDF files with variables, macros, and includes:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="xacro_robot">

  <!-- Define properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="wheel_radius" value="0.1" />
  <xacro:property name="wheel_width" value="0.05" />
  <xacro:property name="base_length" value="0.5" />
  <xacro:property name="base_width" value="0.3" />
  <xacro:property name="base_height" value="0.1" />

  <!-- Macro for creating wheels -->
  <xacro:macro name="wheel" params="prefix parent xyz rpy">
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
        <mass value="0.5"/>
        <inertia ixx="0.01" ixy="0.0" ixz="0.0"
                 iyy="0.01" iyz="0.0" izz="0.01"/>
      </inertial>
    </link>

    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="${parent}"/>
      <child link="${prefix}_wheel"/>
      <origin xyz="${xyz}" rpy="${rpy}"/>
      <axis xyz="0 0 1"/>
    </joint>
  </xacro:macro>

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="${base_length} ${base_width} ${base_height}"/>
      </geometry>
      <material name="light_grey">
        <color rgba="0.7 0.7 0.7 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="${base_length} ${base_width} ${base_height}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0"
               iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Use the wheel macro to create all wheels -->
  <xacro:wheel prefix="front_right" parent="base_link"
               xyz="0.15 -0.15 -0.05" rpy="${M_PI/2} 0 0"/>
  <xacro:wheel prefix="front_left" parent="base_link"
               xyz="0.15 0.15 -0.05" rpy="${M_PI/2} 0 0"/>
  <xacro:wheel prefix="rear_right" parent="base_link"
               xyz="-0.15 -0.15 -0.05" rpy="${M_PI/2} 0 0"/>
  <xacro:wheel prefix="rear_left" parent="base_link"
               xyz="-0.15 0.15 -0.05" rpy="${M_PI/2} 0 0"/>

  <!-- Include other components -->
  <xacro:include filename="$(find my_robot_description)/urdf/sensors.urdf.xacro"/>
</robot>
```

## 🔍 Validating and Visualizing URDF

To check if your URDF is valid and visualize it, you can use ROS tools:

```bash
# Check URDF syntax
check_urdf /path/to/your/robot.urdf

# Visualize in RViz
roslaunch urdf_tutorial display.launch model:=$(rospack find my_robot_description)/urdf/robot.urdf

# Or in ROS 2
ros2 run rviz2 rviz2
# Then add RobotModel display and set the URDF file
```

### Python Script for URDF Validation

```python
#!/usr/bin/env python3
# urdf_validator.py
import xml.etree.ElementTree as ET
from urdf_parser_py.urdf import URDF
import sys

def validate_urdf(urdf_file):
    """Validate URDF file and print robot information"""
    try:
        # Parse XML to check syntax
        tree = ET.parse(urdf_file)
        root = tree.getroot()

        if root.tag != 'robot':
            raise ValueError("Root element must be 'robot'")

        # Load URDF to validate structure
        robot = URDF.from_xml_file(urdf_file)

        print(f"Robot name: {robot.name}")
        print(f"Number of links: {len(robot.links)}")
        print(f"Number of joints: {len(robot.joints)}")
        print(f"Base link: {robot.get_root()}")

        # Print joint information
        for joint in robot.joints:
            print(f"Joint: {joint.name} ({joint.type}) - {joint.parent} -> {joint.child}")

        print("URDF validation successful!")
        return True

    except ET.ParseError as e:
        print(f"XML Parse Error: {e}")
        return False
    except Exception as e:
        print(f"Validation Error: {e}")
        return False

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python urdf_validator.py <urdf_file>")
        sys.exit(1)

    urdf_file = sys.argv[1]
    validate_urdf(urdf_file)
```

## 🧪 Hands-On Exercise: Create a Simple Manipulator Arm

Create a 3-DOF manipulator arm with the following specifications:

**Expected Time:** 35 minutes

**Requirements:**
- Base link with fixed joint to world
- 3 revolute joints with appropriate limits
- Links representing arm segments
- End-effector link
- Use Xacro for parameterization

**Instructions:**
1. Create a base link as the root
2. Add 3 arm segments connected by revolute joints
3. Add an end-effector link
4. Define appropriate joint limits and visual properties
5. Use Xacro to parameterize link lengths and joint limits

**Solution Hints:**
- Use revolute joints with reasonable angle limits (e.g., ±90 degrees)
- Make each arm segment progressively smaller
- Consider the physical meaning of joint axes (which direction they rotate)

## 💡 Key Takeaways

- **URDF** defines robot structure using links, joints, and properties
- **Links** are rigid bodies, **joints** connect them with specific DOF
- **Visual** properties control appearance, **collision** properties enable physics
- **Xacro** allows parameterization and reuse of URDF components
- Proper inertial properties are essential for accurate simulation

## 📚 Further Reading

- [URDF Documentation](http://wiki.ros.org/urdf)
- [Xacro Documentation](http://wiki.ros.org/xacro)
- [URDF Tutorials](http://wiki.ros.org/urdf/Tutorials)

---

**Next Chapter:** [rclpy Python Integration](/module-1/rclpy-python-integration)
