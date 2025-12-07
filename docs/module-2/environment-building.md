---
sidebar_position: 7
---

# 🏗️ Creating Simulation Worlds and Environments

Creating realistic and diverse simulation environments is crucial for developing robust robotic systems. Well-designed environments enable comprehensive testing of robot capabilities, from navigation and manipulation to perception and interaction. This chapter explores the principles and techniques for building effective simulation environments that bridge the gap between laboratory testing and real-world deployment.

## 🎯 Learning Objectives

By the end of this chapter, you will:
- Design and implement diverse simulation environments for robotics testing
- Create realistic indoor and outdoor environments with appropriate physics properties
- Build custom models and objects for specialized testing scenarios
- Understand the impact of environmental complexity on robot performance
- Apply best practices for environment optimization and validation
- Implement dynamic and interactive elements in simulation worlds

## Fundamentals of Environment Design

### Environmental Complexity Spectrum

Simulation environments exist on a complexity spectrum, from simple geometric shapes to photorealistic scenes:

**Simple Environments**: Basic shapes and minimal features, ideal for algorithm validation and debugging.

**Medium Complexity**: Structured environments like rooms, corridors, and basic outdoor spaces, suitable for navigation and path planning.

**High Complexity**: Detailed environments with realistic textures, lighting, and interactive elements, perfect for perception and interaction testing.

**Photorealistic**: Environments that closely match real-world conditions, essential for sim-to-real transfer.

### Key Design Principles

**Relevance**: Environments should match the intended application domain.

**Progressive Complexity**: Start simple and gradually increase complexity as robot capabilities improve.

**Repeatability**: Environments should provide consistent conditions for algorithm testing.

**Variety**: Multiple environments help ensure algorithm robustness.

## Gazebo World Building

### Basic World Structure

A Gazebo world file defines the complete simulation environment:

```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <world name="simple_indoor">
    <!-- Physics engine configuration -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- Environment lighting -->
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.3 0.3 -1</direction>
    </light>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Default camera view -->
    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>-5 -5 5 0 0.5 1.5708</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
      </camera>
    </gui>

    <!-- Custom models will go here -->
  </world>
</sdf>
```

### Creating Walls and Obstacles

Walls and obstacles form the basic structure of indoor environments:

```xml
<!-- Simple wall -->
<model name="wall_1">
  <pose>5 0 0.5 0 0 0</pose>
  <link name="link">
    <collision name="collision">
      <geometry>
        <box>
          <size>0.1 4 2</size>
        </box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box>
          <size>0.1 4 2</size>
        </box>
      </geometry>
      <material>
        <ambient>0.7 0.7 0.7 1</ambient>
        <diffuse>0.7 0.7 0.7 1</diffuse>
      </material>
    </visual>
    <inertial>
      <mass>100</mass>
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
</model>

<!-- Doorway in wall -->
<model name="wall_with_door">
  <pose>0 3 1 0 0 0</pose>
  <link name="link">
    <collision name="left_wall_collision">
      <pose>-1.5 0 0 0 0 0</pose>
      <geometry>
        <box>
          <size>2 0.2 2</size>
        </box>
      </geometry>
    </collision>
    <collision name="right_wall_collision">
      <pose>1.5 0 0 0 0 0</pose>
      <geometry>
        <box>
          <size>2 0.2 2</size>
        </box>
      </geometry>
    </collision>
    <visual name="left_wall_visual">
      <pose>-1.5 0 0 0 0 0</pose>
      <geometry>
        <box>
          <size>2 0.2 2</size>
        </box>
      </geometry>
      <material>
        <ambient>0.8 0.8 0.8 1</ambient>
        <diffuse>0.8 0.8 0.8 1</diffuse>
      </material>
    </visual>
    <visual name="right_wall_visual">
      <pose>1.5 0 0 0 0 0</pose>
      <geometry>
        <box>
          <size>2 0.2 2</size>
        </box>
      </geometry>
      <material>
        <ambient>0.8 0.8 0.8 1</ambient>
        <diffuse>0.8 0.8 0.8 1</diffuse>
      </material>
    </visual>
  </link>
</model>
```

### Advanced Environment Features

#### Textured Surfaces

Realistic textures improve visual fidelity and perception testing:

```xml
<model name="textured_floor">
  <link name="floor_link">
    <collision name="collision">
      <geometry>
        <box>
          <size>10 10 0.1</size>
        </box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box>
          <size>10 10 0.1</size>
        </box>
      </geometry>
      <material>
        <script>
          <uri>file://media/materials/scripts/gazebo.material</uri>
          <name>Gazebo/Concrete</name>
        </script>
      </material>
    </visual>
  </link>
</model>
```

#### Interactive Elements

Creating objects that robots can interact with:

```xml
<!-- Movable box -->
<model name="movable_box">
  <pose>2 2 0.1 0 0 0</pose>
  <link name="box_link">
    <collision name="collision">
      <geometry>
        <box>
          <size>0.3 0.3 0.3</size>
        </box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box>
          <size>0.3 0.3 0.3</size>
        </box>
      </geometry>
      <material>
        <ambient>0.2 0.8 0.2 1</ambient>
        <diffuse>0.2 0.8 0.2 1</diffuse>
      </material>
    </visual>
    <inertial>
      <mass>2.0</mass>
      <inertia>
        <ixx>0.01</ixx>
        <ixy>0</ixy>
        <ixz>0</ixz>
        <iyy>0.01</iyy>
        <iyz>0</iyz>
        <izz>0.01</izz>
      </inertia>
    </inertial>
  </link>
</model>

<!-- Rotating door (hinged object) -->
<model name="rotating_door">
  <link name="door_frame">
    <pose>0 0 1 0 0 0</pose>
    <collision name="collision">
      <geometry>
        <box>
          <size>2 0.1 2</size>
        </box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box>
          <size>2 0.1 2</size>
        </box>
      </geometry>
    </visual>
    <inertial>
      <mass>100</mass>
      <inertia>
        <ixx>10</ixx>
        <ixy>0</ixy>
        <ixz>0</ixz>
        <iyy>10</iyy>
        <iyz>0</iyz>
        <izz>10</izz>
      </inertia>
    </inertial>
  </link>

  <link name="door_panel">
    <pose>0.5 0 1 0 0 0</pose>
    <collision name="collision">
      <geometry>
        <box>
          <size>1 0.05 2</size>
        </box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box>
          <size>1 0.05 2</size>
        </box>
      </geometry>
      <material>
        <ambient>0.6 0.4 0.2 1</ambient>
        <diffuse>0.6 0.4 0.2 1</diffuse>
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

  <joint name="door_hinge" type="revolute">
    <parent>door_frame</parent>
    <child>door_panel</child>
    <pose>-0.5 0 0 0 0 0</pose>
    <axis>
      <xyz>0 1 0</xyz>
      <limit>
        <lower>-1.57</lower>
        <upper>1.57</upper>
        <effort>100</effort>
        <velocity>1</velocity>
      </limit>
      <dynamics>
        <damping>1</damping>
        <friction>0.1</friction>
      </dynamics>
    </axis>
  </joint>
</model>
```

## Indoor Environment Design

### Room Layouts

Creating structured indoor environments:

```xml
<!-- Living room environment -->
<sdf version="1.7">
  <world name="living_room">
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Room walls -->
    <model name="room_walls">
      <static>true</static>
      <link name="walls">
        <!-- Front wall -->
        <collision name="front_wall">
          <pose>0 5 1 0 0 0</pose>
          <geometry>
            <box>
              <size>10 0.2 2</size>
            </box>
          </geometry>
        </collision>
        <!-- Back wall -->
        <collision name="back_wall">
          <pose>0 -5 1 0 0 0</pose>
          <geometry>
            <box>
              <size>10 0.2 2</size>
            </box>
          </geometry>
        </collision>
        <!-- Left wall -->
        <collision name="left_wall">
          <pose>-5 0 1 0 0 0</pose>
          <geometry>
            <box>
              <size>0.2 10 2</size>
            </box>
          </geometry>
        </collision>
        <!-- Right wall -->
        <collision name="right_wall">
          <pose>5 0 1 0 0 0</pose>
          <geometry>
            <box>
              <size>0.2 10 2</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>

    <!-- Furniture -->
    <model name="sofa">
      <pose>-2 2 0.2 0 0 0</pose>
      <link name="sofa_link">
        <collision name="collision">
          <geometry>
            <box>
              <size>2 1 0.4</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>2 1 0.4</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.3 0.1 1</ambient>
            <diffuse>0.5 0.3 0.1 1</diffuse>
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
          </inertia>
        </inertial>
      </link>
    </model>

    <model name="coffee_table">
      <pose>0 0 0.3 0 0 0</pose>
      <link name="table_link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 0.6 0.6</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 0.6 0.6</size>
            </box>
          </geometry>
          <material>
            <ambient>0.3 0.2 0.1 1</ambient>
            <diffuse>0.3 0.2 0.1 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>10</mass>
          <inertia>
            <ixx>0.5</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.5</iyy>
            <iyz>0</iyz>
            <izz>0.5</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- Lighting -->
    <light name="room_light" type="point">
      <pose>0 0 3 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>10</range>
        <constant>0.2</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
    </light>
  </world>
</sdf>
```

### Navigation Testing Environments

Creating environments specifically for navigation challenges:

```xml
<!-- Maze environment -->
<model name="maze_walls">
  <static>true</static>
  <link name="maze_structure">
    <!-- Outer walls -->
    <collision name="outer_wall_1">
      <pose>0 5 0.5 0 0 0</pose>
      <geometry>
        <box>
          <size>10 0.1 1</size>
        </box>
      </geometry>
    </collision>
    <collision name="outer_wall_2">
      <pose>0 -5 0.5 0 0 0</pose>
      <geometry>
        <box>
          <size>10 0.1 1</size>
        </box>
      </geometry>
    </collision>
    <collision name="outer_wall_3">
      <pose>5 0 0.5 0 0 0</pose>
      <geometry>
        <box>
          <size>0.1 10 1</size>
        </box>
      </geometry>
    </collision>
    <collision name="outer_wall_4">
      <pose>-5 0 0.5 0 0 0</pose>
      <geometry>
        <box>
          <size>0.1 10 1</size>
        </box>
      </geometry>
    </collision>

    <!-- Inner maze walls -->
    <collision name="inner_wall_1">
      <pose>-3 3 0.5 0 0 0</pose>
      <geometry>
        <box>
          <size>2 0.1 1</size>
        </box>
      </geometry>
    </collision>
    <collision name="inner_wall_2">
      <pose>2 -2 0.5 0 0 0</pose>
      <geometry>
        <box>
          <size>0.1 4 1</size>
        </box>
      </geometry>
    </collision>
    <!-- Additional maze walls... -->
  </link>
</model>
```

## Outdoor Environment Design

### Terrain Modeling

Creating realistic outdoor environments with varied terrain:

```xml
<!-- Outdoor environment with terrain -->
<sdf version="1.7">
  <world name="outdoor_world">
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <gravity>0 0 -9.8</gravity>
    </physics>

    <!-- Ground plane with texture -->
    <model name="terrain">
      <static>true</static>
      <link name="terrain_link">
        <collision name="collision">
          <geometry>
            <heightmap>
              <uri>model://my_terrain/images/heightmap.png</uri>
              <size>20 20 3</size>
              <pos>0 0 0</pos>
            </heightmap>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <heightmap>
              <uri>model://my_terrain/images/heightmap.png</uri>
              <size>20 20 3</size>
              <pos>0 0 0</pos>
            </heightmap>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Dirt</name>
            </script>
          </material>
        </visual>
      </link>
    </model>

    <!-- Trees -->
    <model name="tree_1">
      <include>
        <uri>model://tree</uri>
        <pose>5 5 0 0 0 0</pose>
      </include>
    </model>

    <model name="tree_2">
      <include>
        <uri>model://tree</uri>
        <pose>-5 -5 0 0 0 0</pose>
      </include>
    </model>

    <!-- Rocks -->
    <model name="rock_1">
      <pose>2 3 0.2 0 0 0</pose>
      <link name="rock_link">
        <collision name="collision">
          <geometry>
            <mesh>
              <uri>model://my_terrain/meshes/rock.dae</uri>
            </mesh>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <mesh>
              <uri>model://my_terrain/meshes/rock.dae</uri>
            </mesh>
          </geometry>
        </visual>
        <inertial>
          <mass>5</mass>
          <inertia>
            <ixx>0.1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.1</iyy>
            <iyz>0</iyz>
            <izz>0.1</izz>
          </inertia>
        </inertial>
      </link>
    </model>
  </world>
</sdf>
```

### Weather and Environmental Effects

Simulating environmental conditions:

```xml
<!-- Environment with weather effects -->
<world name="weather_world">
  <!-- Physics -->
  <physics type="ode">
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1</real_time_factor>
    <gravity>0 0 -9.8</gravity>
  </physics>

  <!-- Wind effects -->
  <model name="wind_generator">
    <static>true</static>
    <link name="wind_link">
      <inertial>
        <mass>1</mass>
        <inertia>
          <ixx>1</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>1</iyy>
          <iyz>0</iyz>
          <izz>1</izz>
        </inertia>
      </inertial>
      <velocity_decay>
        <linear>0.01</linear>
        <angular>0.01</angular>
      </velocity_decay>
    </link>
    <plugin name="wind_plugin" filename="libgazebo_ros_wind.so">
      <always_on>true</always_on>
      <update_rate>10</update_rate>
      <wind_direction>1 0 0</wind_direction>
      <wind_force>0.5 0 0</wind_force>
      <wind_gust_direction>0.7 0.3 0</wind_gust_direction>
      <wind_gust_duration>2</wind_gust_duration>
      <wind_gust_start>10</wind_gust_start>
    </plugin>
  </model>

  <!-- Lighting for different times of day -->
  <light name="sun" type="directional">
    <pose>0 0 10 0 0 0</pose>
    <diffuse>0.8 0.8 0.7 1</diffuse>
    <specular>0.2 0.2 0.2 1</specular>
    <attenuation>
      <range>1000</range>
      <constant>0.9</constant>
      <linear>0.01</linear>
      <quadratic>0.001</quadratic>
    </attenuation>
    <direction>-0.3 -0.3 -0.8</direction>
  </light>
</world>
```

## Dynamic and Interactive Environments

### Moving Obstacles

Creating environments with dynamic elements:

```xml
<!-- Environment with moving obstacles -->
<model name="moving_obstacle">
  <link name="obstacle_link">
    <pose>0 0 0.5 0 0 0</pose>
    <collision name="collision">
      <geometry>
        <sphere>
          <radius>0.3</radius>
        </sphere>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <sphere>
          <radius>0.3</radius>
        </sphere>
      </geometry>
      <material>
        <ambient>1 0 0 1</ambient>
        <diffuse>1 0 0 1</diffuse>
      </material>
    </visual>
    <inertial>
      <mass>1</mass>
      <inertia>
        <ixx>0.1</ixx>
        <ixy>0</ixy>
        <ixz>0</ixz>
        <iyy>0.1</iyy>
        <iyz>0</iyz>
        <izz>0.1</izz>
      </inertia>
    </inertial>
  </link>

  <!-- Plugin to make the obstacle move in a circle -->
  <plugin name="model_move_plugin" filename="libgazebo_ros_p3d.so">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <body_name>obstacle_link</body_name>
    <topic_name>ground_truth_odom</topic_name>
    <gaussian_noise>0.0</gaussian_noise>
    <frame_name>world</frame_name>
  </plugin>
</model>
```

### Interactive Objects with Sensors

Creating objects that respond to robot interactions:

```xml
<!-- Interactive object with contact sensor -->
<model name="interactive_box">
  <pose>3 0 0.2 0 0 0</pose>
  <link name="box_link">
    <collision name="collision">
      <geometry>
        <box>
          <size>0.4 0.4 0.4</size>
        </box>
      </geometry>
      <surface>
        <contact>
          <ode>
            <soft_cfm>0</soft_cfm>
            <soft_erp>0.2</soft_erp>
            <kp>1e+6</kp>
            <kd>100</kd>
            <max_vel>100</max_vel>
            <min_depth>0.001</min_depth>
          </ode>
        </contact>
      </surface>
    </collision>
    <visual name="visual">
      <geometry>
        <box>
          <size>0.4 0.4 0.4</size>
        </box>
      </geometry>
      <material>
        <ambient>0 0 1 1</ambient>
        <diffuse>0 0 1 1</diffuse>
      </material>
    </visual>
    <inertial>
      <mass>5</mass>
      <inertia>
        <ixx>0.1</ixx>
        <ixy>0</ixy>
        <ixz>0</ixz>
        <iyy>0.1</iyy>
        <iyz>0</iyz>
        <izz>0.1</izz>
      </inertia>
    </inertial>
  </link>

  <!-- Contact sensor to detect interactions -->
  <sensor name="contact_sensor" type="contact">
    <always_on>1</always_on>
    <update_rate>30</update_rate>
    <contact>
      <collision>collision</collision>
    </contact>
    <plugin name="contact_plugin" filename="libgazebo_ros_bumper.so">
      <alwaysOn>true</alwaysOn>
      <frameName>world</frameName>
      <bumperTopicName>contact_info</bumperTopicName>
    </plugin>
  </sensor>
</model>
```

## Environment Optimization

### Performance Considerations

Optimizing environments for better simulation performance:

```xml
<!-- Optimized world with performance settings -->
<world name="optimized_world">
  <!-- Physics optimization -->
  <physics type="ode">
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1</real_time_factor>
    <real_time_update_rate>1000</real_time_update_rate>
    <!-- Solver optimization -->
    <ode>
      <solver>
        <type>quick</type>
        <iters>100</iters>
        <sor>1.3</sor>
      </solver>
      <constraints>
        <cfm>0.000001</cfm>
        <erp>0.2</erp>
        <contact_max_correcting_vel>100</contact_max_correcting_vel>
        <contact_surface_layer>0.001</contact_surface_layer>
      </constraints>
    </ode>
  </physics>

  <!-- Use simple collision geometries where possible -->
  <model name="optimized_model">
    <link name="link">
      <!-- Use box instead of complex mesh for collision -->
      <collision name="collision">
        <geometry>
          <box>
            <size>1 1 1</size>
          </box>
        </geometry>
      </collision>
      <!-- Use detailed mesh for visual only -->
      <visual name="visual">
        <geometry>
          <mesh>
            <uri>model://detailed_model/meshes/complex_shape.dae</uri>
          </mesh>
        </geometry>
      </visual>
    </link>
  </model>
</world>
```

### Level of Detail (LOD) Systems

Implementing LOD for complex environments:

```xml
<!-- LOD model for complex object -->
<model name="lod_model">
  <link name="main_link">
    <visual name="high_detail">
      <geometry>
        <mesh>
          <uri>model://lod_model/meshes/high_detail.dae</uri>
        </geometry>
      </visual>
      <!-- Only show at close range -->
      <transparency>0</transparency>
    </visual>
    <visual name="low_detail">
      <geometry>
        <mesh>
          <uri>model://lod_model/meshes/low_detail.dae</uri>
        </geometry>
      </visual>
      <!-- Show at far range -->
      <transparency>0</transparency>
    </visual>
  </link>
</model>
```

## Practical Example: Complete Warehouse Environment

Let's create a comprehensive warehouse environment for mobile robot testing:

```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <world name="warehouse">
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
          </inertia>
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
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- Loading dock area -->
    <model name="loading_dock">
      <pose>8 0 0.1 0 0 0</pose>
      <link name="dock_link">
        <collision name="collision">
          <geometry>
            <box>
              <size>2 4 0.2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>2 4 0.2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.3 0.3 0.3 1</ambient>
            <diffuse>0.3 0.3 0.3 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Pallets -->
    <model name="pallet_1">
      <pose>6 3 0.1 0 0 0</pose>
      <link name="pallet_link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1.2 0.8 0.15</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1.2 0.8 0.15</size>
            </box>
          </geometry>
          <material>
            <ambient>0.6 0.4 0.2 1</ambient>
            <diffuse>0.6 0.4 0.2 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>20</mass>
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
    </model>

    <model name="pallet_2">
      <pose>6 -3 0.1 0 0 0</pose>
      <link name="pallet_link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1.2 0.8 0.15</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1.2 0.8 0.15</size>
            </box>
          </geometry>
          <material>
            <ambient>0.6 0.4 0.2 1</ambient>
            <diffuse>0.6 0.4 0.2 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>20</mass>
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
    </model>

    <!-- Start position for robot -->
    <model name="robot_spawn_point">
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

## Environment Validation and Testing

### Automated Environment Testing

Creating scripts to validate environment properties:

```python
#!/usr/bin/env python3
"""
Environment validation script
"""
import xml.etree.ElementTree as ET
import os
from pathlib import Path

def validate_environment(world_file):
    """Validate a Gazebo world file for common issues"""
    try:
        tree = ET.parse(world_file)
        root = tree.getroot()

        # Check for physics definition
        physics = root.find('.//physics')
        if physics is None:
            print("Warning: No physics engine defined in world file")
        else:
            print("✓ Physics engine defined")

        # Check for ground plane or floor
        has_ground = False
        for include in root.findall('.//include'):
            uri = include.find('uri')
            if uri is not None and 'ground_plane' in uri.text:
                has_ground = True
                break

        if not has_ground:
            print("Warning: No ground plane found")
        else:
            print("✓ Ground plane found")

        # Check for lighting
        lights = root.findall('.//light')
        if len(lights) == 0:
            print("Warning: No lights defined in environment")
        else:
            print(f"✓ Found {len(lights)} light(s)")

        # Check for static models (obstacles)
        static_models = root.findall('.//model[@static="true"]')
        if len(static_models) == 0:
            print("Warning: No static obstacles in environment")
        else:
            print(f"✓ Found {len(static_models)} static model(s)")

        return True
    except ET.ParseError as e:
        print(f"Error parsing world file: {e}")
        return False
    except Exception as e:
        print(f"Unexpected error: {e}")
        return False

def check_model_uris(world_file, model_path="~/.gazebo/models"):
    """Check if referenced models exist"""
    try:
        tree = ET.parse(world_file)
        root = tree.getroot()

        # Find all URIs referencing models
        uris = root.findall('.//uri')

        model_dir = Path(os.path.expanduser(model_path))

        missing_models = []
        for uri in uris:
            uri_text = uri.text
            if uri_text.startswith('model://'):
                model_name = uri_text.split('://')[1].split('/')[0]
                model_folder = model_dir / model_name

                if not model_folder.exists():
                    missing_models.append(model_name)

        if missing_models:
            print(f"Warning: Missing models: {missing_models}")
        else:
            print("✓ All referenced models found")

        return len(missing_models) == 0
    except Exception as e:
        print(f"Error checking model URIs: {e}")
        return False

if __name__ == "__main__":
    import sys
    if len(sys.argv) != 2:
        print("Usage: python3 env_validator.py <world_file>")
        sys.exit(1)

    world_file = sys.argv[1]
    print(f"Validating environment: {world_file}")
    print("-" * 50)

    # Validate basic structure
    basic_valid = validate_environment(world_file)
    print()

    # Check model availability
    models_valid = check_model_uris(world_file)
    print()

    if basic_valid and models_valid:
        print("✓ Environment validation passed")
        sys.exit(0)
    else:
        print("✗ Environment validation failed")
        sys.exit(1)
```

## Best Practices for Environment Building

### Design Guidelines

1. **Start Simple**: Begin with basic geometric shapes before adding complexity
2. **Progressive Complexity**: Gradually add features as needed
3. **Realistic Physics**: Ensure environmental objects have appropriate physical properties
4. **Performance Balance**: Optimize for both visual quality and simulation performance
5. **Repeatability**: Create consistent conditions for algorithm testing
6. **Documentation**: Include comments explaining environment features

### Performance Optimization

- Use simple collision geometries (boxes, spheres, cylinders) when possible
- Limit the number of complex meshes
- Use appropriate lighting without excessive computational overhead
- Consider Level of Detail (LOD) for distant objects
- Optimize physics parameters for your specific use case

### Validation Strategies

- Test environments with basic robot navigation
- Verify sensor data quality in the environment
- Check for physics stability with moving objects
- Validate that the environment behaves as expected under different conditions

## 💡 Key Takeaways

- Environments should match the intended application domain for effective testing
- Balance visual complexity with simulation performance
- Use appropriate physics properties for realistic interactions
- Include varied elements to test robot robustness
- Validate environments with automated tools before deployment
- Document environment features for reproducible research

## 🏋️ Hands-On Exercise

Create a complete indoor environment that includes:
- A structured room layout with walls and doors
- Furniture and obstacles of varying sizes and shapes
- Appropriate lighting for the environment
- A navigation path for robot testing
- At least one interactive element (movable object or sensor)

**Expected Time:** 120 minutes

**Requirements:**
- Gazebo installed
- Text editor
- Understanding of SDF format

**Instructions:**
1. Create a new world file with basic physics and lighting
2. Design a room layout with walls and openings
3. Add furniture and obstacles with appropriate physics properties
4. Include at least one interactive element
5. Test your environment in Gazebo to verify functionality
6. Document the environment features and intended use cases

**Solution Hints:**
- Start with simple geometric shapes before adding complexity
- Use realistic sizes for furniture and obstacles
- Ensure navigation paths are clear of static obstacles
- Test with a simple robot model to verify navigability
- Consider the intended robot size when designing spaces

## 📚 Further Reading

- [Gazebo World Tutorial](http://gazebosim.org/tutorials?tut=build_world): Official Gazebo world building guide
- [SDF World Specification](http://sdformat.org/spec): Complete SDF world format documentation
- "Simulation-Based Development of Autonomous Systems" by Althoff et al.: Advanced simulation techniques
- [ROS Navigation Tutorials](http://wiki.ros.org/navigation/Tutorials): Robot navigation in various environments

---

**Next Chapter:** [Project: Simulated Robot](/module-2/project-simulated-robot) - Build and simulate a complete robot in your environments
