---
sidebar_position: 6
---

# 🤖 URDF vs SDF: Robot Description Formats

Robot description formats are fundamental to robotics simulation and development, providing standardized ways to define robot geometry, kinematics, dynamics, and sensors. The two primary formats in the robotics ecosystem are URDF (Unified Robot Description Format) and SDF (Simulation Description Format). Understanding both formats and their appropriate use cases is essential for effective robotics development across different platforms and simulation environments.

## 🎯 Learning Objectives

By the end of this chapter, you will:
- Understand the differences between URDF and SDF formats
- Create complete robot descriptions using both formats
- Implement kinematic and dynamic properties in robot models
- Integrate sensors and actuators into robot descriptions
- Choose the appropriate format for different use cases
- Troubleshoot common issues in robot description files

## Understanding Robot Description Formats

### What Are Robot Description Formats?

Robot description formats are XML-based languages that define the physical and kinematic properties of robots. They specify:
- **Kinematic structure**: Joint connections and degrees of freedom
- **Geometric properties**: Link shapes, sizes, and visual appearance
- **Dynamic properties**: Mass, inertia, and center of mass
- **Sensor placement**: Locations and types of sensors
- **Collision properties**: Shapes used for collision detection

These formats enable:
- **Simulation**: Creating virtual robot models in Gazebo and other simulators
- **Visualization**: Displaying robots in RViz and other visualization tools
- **Control**: Understanding robot kinematics for motion planning
- **Analysis**: Computing forward/inverse kinematics and dynamics

## URDF: The ROS Standard

### URDF Overview

URDF (Unified Robot Description Format) is the standard robot description format for ROS and ROS2. It's primarily designed for describing robot kinematics and is tightly integrated with ROS tooling.

### Basic URDF Structure

A minimal URDF file includes:

```xml
<?xml version="1.0"?>
<robot name="simple_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
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
      <mass value="10.0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>
</robot>
```

### Links in URDF

Links represent rigid bodies in the robot:

```xml
<link name="wheel_link">
  <!-- Visual properties for rendering -->
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <mesh filename="package://robot_description/meshes/wheel.dae"/>
    </geometry>
    <material name="wheel_material">
      <color rgba="0.5 0.5 0.5 1.0"/>
    </material>
  </visual>

  <!-- Collision properties for physics simulation -->
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <cylinder length="0.1" radius="0.15"/>
    </geometry>
  </collision>

  <!-- Inertial properties for dynamics -->
  <inertial>
    <mass value="2.0"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.02"/>
  </inertial>
</link>
```

### Joints in URDF

Joints define the connections between links:

```xml
<joint name="wheel_joint" type="continuous">
  <parent link="base_link"/>
  <child link="wheel_link"/>
  <origin xyz="0.2 0.3 0" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <dynamics damping="0.1" friction="0.0"/>
</joint>

<!-- Different joint types -->
<joint name="revolute_joint" type="revolute">
  <parent link="link1"/>
  <child link="link2"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  <dynamics damping="0.1" friction="0.01"/>
</joint>

<joint name="prismatic_joint" type="prismatic">
  <parent link="base_link"/>
  <child link="slider_link"/>
  <origin xyz="0 0 0.2" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>
  <limit lower="0" upper="0.5" effort="50" velocity="0.5"/>
</joint>
```

### Materials and Colors

Materials define the visual appearance:

```xml
<material name="red">
  <color rgba="1 0 0 1"/>
</material>

<material name="green">
  <color rgba="0 1 0 1"/>
</material>

<material name="blue">
  <color rgba="0 0 1 1"/>
</material>

<material name="white">
  <color rgba="1 1 1 1"/>
</material>

<!-- Using texture files -->
<material name="textured_material">
  <color rgba="1 1 1 1"/>
  <texture filename="package://robot_description/materials/textures/wood.png"/>
</material>
```

## SDF: The Gazebo Standard

### SDF Overview

SDF (Simulation Description Format) is the native format for Gazebo simulation. It's more comprehensive than URDF, supporting advanced features like plugins, lighting, and complex environments.

### Basic SDF Structure

A minimal SDF file:

```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <model name="simple_model">
    <link name="chassis">
      <pose>0 0 0.1 0 0 0</pose>
      <inertial>
        <mass>10.0</mass>
        <inertia>
          <ixx>0.4</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.4</iyy>
          <iyz>0</iyz>
          <izz>0.4</izz>
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <box>
            <size>1 0.5 0.2</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>1 0.5 0.2</size>
          </box>
        </geometry>
        <material>
          <ambient>0.8 0.2 0.2 1</ambient>
          <diffuse>0.8 0.2 0.2 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
```

### SDF Model Features

SDF supports more advanced features than URDF:

```xml
<sdf version="1.7">
  <model name="advanced_robot">
    <!-- Model properties -->
    <static>false</static>
    <self_collide>false</self_collide>
    <enable_wind>false</enable_wind>

    <!-- Pose and inertial -->
    <pose>0 0 0.5 0 0 0</pose>

    <link name="base_link">
      <inertial>
        <mass>20.0</mass>
        <inertia>
          <ixx>1.0</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>1.0</iyy>
          <iyz>0</iyz>
          <izz>1.0</izz>
        </inertia>
      </inertial>

      <!-- Multiple collision geometries -->
      <collision name="collision_base">
        <geometry>
          <mesh>
            <uri>model://advanced_robot/meshes/base_collision.dae</uri>
          </mesh>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>0.5</mu>
              <mu2>0.5</mu2>
            </ode>
          </friction>
        </surface>
      </collision>

      <!-- Multiple visual geometries -->
      <visual name="visual_base">
        <geometry>
          <mesh>
            <uri>model://advanced_robot/meshes/base_visual.dae</uri>
          </mesh>
        </geometry>
        <material>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/Orange</name>
          </script>
        </material>
      </visual>
    </link>

    <!-- Joint definition in SDF -->
    <joint name="wheel_joint" type="revolute">
      <parent>base_link</parent>
      <child>wheel_link</child>
      <pose>0.3 0.2 0 0 0 0</pose>
      <axis>
        <xyz>0 1 0</xyz>
        <limit>
          <lower>-1.57</lower>
          <upper>1.57</upper>
          <effort>10</effort>
          <velocity>1</velocity>
        </limit>
        <dynamics>
          <damping>0.1</damping>
          <friction>0.01</friction>
        </dynamics>
      </axis>
    </joint>
  </model>
</sdf>
```

## Converting Between URDF and SDF

### URDF to SDF Conversion

Gazebo can automatically convert URDF to SDF:

```bash
# Convert URDF to SDF
gz sdf -p robot.urdf > robot.sdf

# Or using ROS tools
rosrun xacro xacro robot.xacro > robot.urdf
gz sdf -p robot.urdf > robot.sdf
```

### Programmatic Conversion

```python
#!/usr/bin/env python3
"""
Script to convert URDF to SDF programmatically
"""
import subprocess
import os

def urdf_to_sdf(urdf_file, sdf_file):
    """Convert URDF file to SDF using gz sdf command"""
    try:
        result = subprocess.run(['gz', 'sdf', '-p', urdf_file],
                               capture_output=True, text=True, check=True)

        with open(sdf_file, 'w') as f:
            f.write(result.stdout)

        print(f"Successfully converted {urdf_file} to {sdf_file}")
        return True
    except subprocess.CalledProcessError as e:
        print(f"Error converting URDF to SDF: {e}")
        return False
    except FileNotFoundError:
        print("gz command not found. Make sure Ignition Gazebo is installed.")
        return False

# Example usage
if __name__ == "__main__":
    urdf_to_sdf("my_robot.urdf", "my_robot.sdf")
```

## Advanced URDF Features

### Xacro Macros

Xacro (XML Macros) allows parameterization and reusability:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="xacro_robot">

  <!-- Properties -->
  <xacro:property name="wheel_radius" value="0.1"/>
  <xacro:property name="wheel_width" value="0.05"/>
  <xacro:property name="base_mass" value="10.0"/>

  <!-- Wheel macro -->
  <xacro:macro name="wheel" params="prefix parent xyz rpy">
    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="${parent}"/>
      <child link="${prefix}_wheel"/>
      <origin xyz="${xyz}" rpy="${rpy}"/>
      <axis xyz="0 1 0"/>
    </joint>

    <link name="${prefix}_wheel">
      <visual>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </visual>
      <collision>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="1.0"/>
        <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.02"/>
      </inertial>
    </link>
  </xacro:macro>

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="1 0.5 0.2"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="1 0.5 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="${base_mass}"/>
      <inertia ixx="0.4" ixy="0" ixz="0" iyy="0.4" iyz="0" izz="0.4"/>
    </inertial>
  </link>

  <!-- Use the wheel macro -->
  <xacro:wheel prefix="front_left" parent="base_link"
               xyz="0.3 0.2 0" rpy="0 0 0"/>
  <xacro:wheel prefix="front_right" parent="base_link"
               xyz="0.3 -0.2 0" rpy="0 0 0"/>
  <xacro:wheel prefix="rear_left" parent="base_link"
               xyz="-0.3 0.2 0" rpy="0 0 0"/>
  <xacro:wheel prefix="rear_right" parent="base_link"
               xyz="-0.3 -0.2 0" rpy="0 0 0"/>
</robot>
```

### Transmission Elements

URDF can include transmission definitions for actuator control:

```xml
<transmission name="wheel_transmission">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="wheel_joint">
    <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
  </joint>
  <actuator name="wheel_motor">
    <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

## Advanced SDF Features

### Plugins in SDF

SDF supports plugins for extending functionality:

```xml
<model name="sensor_robot">
  <!-- Differential drive plugin -->
  <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
    <ros>
      <namespace>robot</namespace>
      <remapping>cmd_vel:=cmd_vel</remapping>
      <remapping>odom:=odom</remapping>
    </ros>
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
    <wheel_separation>0.4</wheel_separation>
    <wheel_diameter>0.2</wheel_diameter>
    <command_topic>cmd_vel</command_topic>
    <odometry_topic>odom</odometry_topic>
    <odometry_frame>odom</odometry_frame>
    <robot_base_frame>base_link</robot_base_frame>
  </plugin>

  <!-- Camera plugin -->
  <sensor name="camera" type="camera">
    <pose>0.2 0 0.1 0 0 0</pose>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>camera</namespace>
        <remapping>image_raw:=image</remapping>
      </ros>
      <camera_name>rgb_camera</camera_name>
      <image_topic_name>image_raw</image_topic_name>
      <frame_name>camera_link</frame_name>
    </plugin>
  </sensor>
</model>
```

### World Definition in SDF

SDF can also define entire simulation worlds:

```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <world name="simple_world">
    <!-- Physics engine configuration -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- Include default ground plane and lighting -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Custom models -->
    <model name="robot1" static="false">
      <include>
        <uri>model://turtlebot3_waffle</uri>
      </include>
      <pose>0 0 0 0 0 0</pose>
    </model>

    <!-- Static objects -->
    <model name="table">
      <static>true</static>
      <link name="table_link">
        <pose>2 0 0.4 0 0 0</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>1 0.8 0.8</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 0.8 0.8</size>
            </box>
          </geometry>
          <material>
            <ambient>0.6 0.4 0.2 1</ambient>
            <diffuse>0.6 0.4 0.2 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

## Practical Example: Complete Robot Description

Let's create a complete robot description that works in both formats:

### URDF Version (robot.urdf.xacro)

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="mobile_robot">

  <!-- Properties -->
  <xacro:property name="base_width" value="0.5"/>
  <xacro:property name="base_length" value="0.8"/>
  <xacro:property name="base_height" value="0.2"/>
  <xacro:property name="wheel_radius" value="0.1"/>
  <xacro:property name="wheel_width" value="0.05"/>
  <xacro:property name="wheel_mass" value="0.5"/>
  <xacro:property name="base_mass" value="10.0"/>

  <!-- Base link -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="${base_length} ${base_width} ${base_height}"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
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
      <!-- Approximate inertia for a box -->
      <inertia ixx="${base_mass * (base_width*base_width + base_height*base_height) / 12}"
               ixy="0" ixz="0"
               iyy="${base_mass * (base_length*base_length + base_height*base_height) / 12}"
               iyz="0"
               izz="${base_mass * (base_length*base_length + base_width*base_width) / 12}"/>
    </inertial>
  </link>

  <!-- Wheel macro -->
  <xacro:macro name="wheel" params="prefix parent xyz rpy">
    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="${parent}"/>
      <child link="${prefix}_wheel"/>
      <origin xyz="${xyz}" rpy="${rpy}"/>
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
        <inertia ixx="${wheel_mass * (3*wheel_radius*wheel_radius + wheel_width*wheel_width) / 12}"
                 ixy="0" ixz="0"
                 iyy="${wheel_mass * (3*wheel_radius*wheel_radius + wheel_width*wheel_width) / 12}"
                 iyz="0"
                 izz="${wheel_mass * wheel_radius * wheel_radius / 2}"/>
      </inertial>
    </link>
  </xacro:macro>

  <!-- Instantiate wheels -->
  <xacro:wheel prefix="front_left" parent="base_link"
               xyz="${base_length/2} ${base_width/2} 0" rpy="0 0 0"/>
  <xacro:wheel prefix="front_right" parent="base_link"
               xyz="${base_length/2} -${base_width/2} 0" rpy="0 0 0"/>
  <xacro:wheel prefix="rear_left" parent="base_link"
               xyz="-${base_length/2} ${base_width/2} 0" rpy="0 0 0"/>
  <xacro:wheel prefix="rear_right" parent="base_link"
               xyz="-${base_length/2} -${base_width/2} 0" rpy="0 0 0"/>

  <!-- Camera -->
  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="${base_length/2} 0 ${base_height/2}" rpy="0 0 0"/>
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

</robot>
```

## Tools for Working with URDF/SDF

### Visualization Tools

```bash
# View URDF in RViz
rosrun rviz rviz -d `rospack find urdf_tutorial`/rviz/urdf.rviz

# Check URDF for errors
check_urdf robot.urdf

# Convert and view in Gazebo
gz sdf -p robot.urdf
```

### Validation Scripts

```python
#!/usr/bin/env python3
"""
URDF validation script
"""
import xml.etree.ElementTree as ET
import subprocess
import sys

def validate_urdf(urdf_file):
    """Validate URDF file using check_urdf command"""
    try:
        result = subprocess.run(['check_urdf', urdf_file],
                               capture_output=True, text=True)

        if result.returncode == 0:
            print(f"URDF validation passed for {urdf_file}")
            print(result.stdout)
            return True
        else:
            print(f"URDF validation failed for {urdf_file}")
            print(result.stderr)
            return False
    except FileNotFoundError:
        print("check_urdf command not found. Make sure ROS is installed.")
        return False

def parse_urdf(urdf_file):
    """Parse URDF file and extract basic information"""
    try:
        tree = ET.parse(urdf_file)
        root = tree.getroot()

        robot_name = root.get('name')
        links = root.findall('.//link')
        joints = root.findall('.//joint')

        print(f"Robot name: {robot_name}")
        print(f"Number of links: {len(links)}")
        print(f"Number of joints: {len(joints)}")

        # Print link names
        print("Links:")
        for link in links:
            print(f"  - {link.get('name')}")

        # Print joint names and types
        print("Joints:")
        for joint in joints:
            print(f"  - {joint.get('name')} ({joint.get('type')})")

        return True
    except ET.ParseError as e:
        print(f"Error parsing URDF file: {e}")
        return False
    except Exception as e:
        print(f"Unexpected error: {e}")
        return False

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 urdf_validator.py <urdf_file>")
        sys.exit(1)

    urdf_file = sys.argv[1]

    # Parse the URDF
    if parse_urdf(urdf_file):
        print("\n" + "="*50)
        # Validate the URDF
        validate_urdf(urdf_file)
```

## Best Practices for URDF/SDF

### URDF Best Practices

1. **Use consistent naming**: Follow ROS naming conventions
2. **Parameterize with Xacro**: Use properties and macros for reusability
3. **Validate inertia**: Ensure inertial properties are physically realistic
4. **Separate visual and collision**: Use different geometries when appropriate
5. **Use proper units**: All measurements in meters and kilograms

### SDF Best Practices

1. **Leverage plugins**: Use appropriate plugins for functionality
2. **Optimize for simulation**: Use simpler collision geometries when possible
3. **Define proper physics**: Set appropriate friction, damping, and restitution
4. **Organize hierarchically**: Structure models with clear parent-child relationships

## Common Issues and Troubleshooting

### URDF Issues

**Floating Point Errors**:
- Use consistent precision in poses and dimensions
- Avoid very small or very large values

**Inertia Issues**:
- Ensure inertia matrix is positive definite
- Use realistic mass and inertia values

**Joint Limit Issues**:
- Verify joint limits are within physical constraints
- Check for proper joint types

### SDF Issues

**Plugin Loading Errors**:
- Verify plugin filenames and paths
- Check for missing dependencies

**Physics Instability**:
- Adjust physics parameters (time step, solver iterations)
- Review mass and inertia values

## URDF vs SDF Comparison

| Feature | URDF | SDF |
|---------|------|-----|
| **Primary Use** | ROS Robot Descriptions | Gazebo Simulation |
| **Scope** | Robot kinematics/structure | Complete simulation environments |
| **Plugins** | Limited (through ROS) | Native support |
| **World Definition** | No | Yes |
| **Lighting** | No | Yes |
| **Complex Sensors** | Through ROS plugins | Native support |
| **Learning Curve** | Moderate | Steeper |
| **ROS Integration** | Native | Through bridges |

## 💡 Key Takeaways

- URDF is the standard for ROS robot descriptions, focusing on kinematic structure
- SDF is Gazebo's native format, supporting complete simulation environments
- Xacro macros provide parameterization and reusability in URDF
- SDF plugins enable advanced simulation features
- Choose URDF for ROS-based development, SDF for comprehensive simulation
- Proper inertial properties are crucial for realistic simulation
- Validation tools help ensure correctness of robot descriptions

## 🏋️ Hands-On Exercise

Create a complete robot description that includes:
- A mobile base with proper kinematic structure
- Four wheels with appropriate joints and dynamics
- A camera sensor with realistic mounting
- Proper inertial properties for all links
- Xacro macros for reusability

**Expected Time:** 90 minutes

**Requirements:**
- ROS/ROS2 installed
- Text editor
- Basic understanding of robot kinematics

**Instructions:**
1. Create a URDF file with a differential drive robot
2. Add proper visual, collision, and inertial properties
3. Include a camera sensor with mounting joint
4. Use Xacro macros to reduce code duplication
5. Validate your URDF using ROS tools
6. Convert to SDF and test in Gazebo

**Solution Hints:**
- Calculate realistic inertia values using proper formulas
- Use fixed joints for sensor mounting
- Validate joint limits and types
- Test in both RViz and Gazebo

## 📚 Further Reading

- [URDF/XML Format](http://wiki.ros.org/urdf/XML): Official URDF specification
- [SDF Format Documentation](http://sdformat.org/spec): Complete SDF specification
- [ROS URDF Tutorials](http://wiki.ros.org/urdf/Tutorials): Step-by-step URDF guides
- [Xacro Documentation](http://wiki.ros.org/xacro): XML macro language for URDF
- "Robotics, Vision and Control" by Peter Corke: Robotics fundamentals including kinematics

---

**Next Chapter:** [Environment Building](/module-2/environment-building) - Learn how to create simulation worlds and environments
