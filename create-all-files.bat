@echo off
REM Windows batch script to create all required markdown files

cd docs

echo Creating folder structure...

REM Create directories
mkdir getting-started 2>nul
mkdir module-1 2>nul
mkdir module-2 2>nul
mkdir module-3 2>nul
mkdir module-4 2>nul
mkdir humanoid 2>nul
mkdir capstone 2>nul
mkdir appendix 2>nul

echo Creating Getting Started files...
(
echo ---
echo sidebar_position: 1
echo ---
echo.
echo # Overview
echo.
echo Content coming soon...
) > getting-started\overview.md

(
echo ---
echo sidebar_position: 2
echo ---
echo.
echo # Prerequisites
echo.
echo Content coming soon...
) > getting-started\prerequisites.md

(
echo ---
echo sidebar_position: 3
echo ---
echo.
echo # Hardware Requirements
echo.
echo Content coming soon...
) > getting-started\hardware-requirements.md

(
echo ---
echo sidebar_position: 4
echo ---
echo.
echo # Software Setup
echo.
echo Content coming soon...
) > getting-started\software-setup.md

echo Creating Module 1 files...
(
echo ---
echo sidebar_position: 2
echo ---
echo.
echo # ROS 2 Architecture
echo.
echo Content coming soon...
) > module-1\ros2-architecture.md

(
echo ---
echo sidebar_position: 3
echo ---
echo.
echo # Nodes and Topics
echo.
echo Content coming soon...
) > module-1\nodes-and-topics.md

(
echo ---
echo sidebar_position: 4
echo ---
echo.
echo # Services and Actions
echo.
echo Content coming soon...
) > module-1\services-and-actions.md

(
echo ---
echo sidebar_position: 5
echo ---
echo.
echo # URDF Robot Description
echo.
echo Content coming soon...
) > module-1\urdf-robot-description.md

(
echo ---
echo sidebar_position: 6
echo ---
echo.
echo # RCLPy Python Integration
echo.
echo Content coming soon...
) > module-1\rclpy-python-integration.md

(
echo ---
echo sidebar_position: 7
echo ---
echo.
echo # Launch Files
echo.
echo Content coming soon...
) > module-1\launch-files.md

(
echo ---
echo sidebar_position: 8
echo ---
echo.
echo # Project: ROS 2 Package
echo.
echo Content coming soon...
) > module-1\project-ros2-package.md

echo Creating Module 2 files...
(
echo ---
echo sidebar_position: 1
echo ---
echo.
echo # Introduction
echo.
echo Content coming soon...
) > module-2\introduction.md

(
echo ---
echo sidebar_position: 2
echo ---
echo.
echo # Gazebo Fundamentals
echo.
echo Content coming soon...
) > module-2\gazebo-fundamentals.md

(
echo ---
echo sidebar_position: 3
echo ---
echo.
echo # Physics Simulation
echo.
echo Content coming soon...
) > module-2\physics-simulation.md

(
echo ---
echo sidebar_position: 4
echo ---
echo.
echo # Sensor Simulation
echo.
echo Content coming soon...
) > module-2\sensor-simulation.md

(
echo ---
echo sidebar_position: 5
echo ---
echo.
echo # Unity Integration
echo.
echo Content coming soon...
) > module-2\unity-integration.md

(
echo ---
echo sidebar_position: 6
echo ---
echo.
echo # URDF SDF Formats
echo.
echo Content coming soon...
) > module-2\urdf-sdf-formats.md

(
echo ---
echo sidebar_position: 7
echo ---
echo.
echo # Environment Building
echo.
echo Content coming soon...
) > module-2\environment-building.md

(
echo ---
echo sidebar_position: 8
echo ---
echo.
echo # Project: Simulated Robot
echo.
echo Content coming soon...
) > module-2\project-simulated-robot.md

echo Creating Module 3 files...
(
echo ---
echo sidebar_position: 1
echo ---
echo.
echo # Introduction
echo.
echo Content coming soon...
) > module-3\introduction.md

(
echo ---
echo sidebar_position: 2
echo ---
echo.
echo # Isaac Sim Overview
echo.
echo Content coming soon...
) > module-3\isaac-sim-overview.md

(
echo ---
echo sidebar_position: 3
echo ---
echo.
echo # Synthetic Data Generation
echo.
echo Content coming soon...
) > module-3\synthetic-data-generation.md

(
echo ---
echo sidebar_position: 4
echo ---
echo.
echo # Isaac ROS Hardware Acceleration
echo.
echo Content coming soon...
) > module-3\isaac-ros-hardware-acceleration.md

(
echo ---
echo sidebar_position: 5
echo ---
echo.
echo # VSLAM Navigation
echo.
echo Content coming soon...
) > module-3\vslam-navigation.md

(
echo ---
echo sidebar_position: 6
echo ---
echo.
echo # Nav2 Path Planning
echo.
echo Content coming soon...
) > module-3\nav2-path-planning.md

(
echo ---
echo sidebar_position: 7
echo ---
echo.
echo # Perception Pipeline
echo.
echo Content coming soon...
) > module-3\perception-pipeline.md

(
echo ---
echo sidebar_position: 8
echo ---
echo.
echo # Reinforcement Learning
echo.
echo Content coming soon...
) > module-3\reinforcement-learning.md

(
echo ---
echo sidebar_position: 9
echo ---
echo.
echo # Sim to Real Transfer
echo.
echo Content coming soon...
) > module-3\sim-to-real-transfer.md

(
echo ---
echo sidebar_position: 10
echo ---
echo.
echo # Project: Perception System
echo.
echo Content coming soon...
) > module-3\project-perception-system.md

echo Creating Module 4 files...
(
echo ---
echo sidebar_position: 1
echo ---
echo.
echo # Introduction
echo.
echo Content coming soon...
) > module-4\introduction.md

(
echo ---
echo sidebar_position: 2
echo ---
echo.
echo # Voice to Action with Whisper
echo.
echo Content coming soon...
) > module-4\voice-to-action-whisper.md

(
echo ---
echo sidebar_position: 3
echo ---
echo.
echo # LLM Cognitive Planning
echo.
echo Content coming soon...
) > module-4\llm-cognitive-planning.md

(
echo ---
echo sidebar_position: 4
echo ---
echo.
echo # Natural Language Commands
echo.
echo Content coming soon...
) > module-4\natural-language-commands.md

(
echo ---
echo sidebar_position: 5
echo ---
echo.
echo # Multimodal Interaction
echo.
echo Content coming soon...
) > module-4\multimodal-interaction.md

(
echo ---
echo sidebar_position: 6
echo ---
echo.
echo # GPT Integration
echo.
echo Content coming soon...
) > module-4\gpt-integration.md

(
echo ---
echo sidebar_position: 7
echo ---
echo.
echo # Conversational Robotics
echo.
echo Content coming soon...
) > module-4\conversational-robotics.md

(
echo ---
echo sidebar_position: 8
echo ---
echo.
echo # Project: Autonomous Humanoid
echo.
echo Content coming soon...
) > module-4\project-autonomous-humanoid.md

echo Creating Humanoid files...
(
echo ---
echo sidebar_position: 1
echo ---
echo.
echo # Kinematics and Dynamics
echo.
echo Content coming soon...
) > humanoid\kinematics-dynamics.md

(
echo ---
echo sidebar_position: 2
echo ---
echo.
echo # Bipedal Locomotion
echo.
echo Content coming soon...
) > humanoid\bipedal-locomotion.md

(
echo ---
echo sidebar_position: 3
echo ---
echo.
echo # Balance Control
echo.
echo Content coming soon...
) > humanoid\balance-control.md

(
echo ---
echo sidebar_position: 4
echo ---
echo.
echo # Manipulation and Grasping
echo.
echo Content coming soon...
) > humanoid\manipulation-grasping.md

(
echo ---
echo sidebar_position: 5
echo ---
echo.
echo # Human Robot Interaction
echo.
echo Content coming soon...
) > humanoid\human-robot-interaction.md

echo Creating Capstone files...
(
echo ---
echo sidebar_position: 1
echo ---
echo.
echo # Project Overview
echo.
echo Content coming soon...
) > capstone\project-overview.md

(
echo ---
echo sidebar_position: 2
echo ---
echo.
echo # Requirements
echo.
echo Content coming soon...
) > capstone\requirements.md

(
echo ---
echo sidebar_position: 3
echo ---
echo.
echo # Architecture Design
echo.
echo Content coming soon...
) > capstone\architecture-design.md

(
echo ---
echo sidebar_position: 4
echo ---
echo.
echo # Implementation Guide
echo.
echo Content coming soon...
) > capstone\implementation-guide.md

(
echo ---
echo sidebar_position: 5
echo ---
echo.
echo # Testing and Validation
echo.
echo Content coming soon...
) > capstone\testing-validation.md

(
echo ---
echo sidebar_position: 6
echo ---
echo.
echo # Presentation Tips
echo.
echo Content coming soon...
) > capstone\presentation-tips.md

echo Creating Appendix files...
(
echo ---
echo sidebar_position: 1
echo ---
echo.
echo # Glossary
echo.
echo Content coming soon...
) > appendix\glossary.md

(
echo ---
echo sidebar_position: 2
echo ---
echo.
echo # Resources
echo.
echo Content coming soon...
) > appendix\resources.md

(
echo ---
echo sidebar_position: 3
echo ---
echo.
echo # Troubleshooting
echo.
echo Content coming soon...
) > appendix\troubleshooting.md

(
echo ---
echo sidebar_position: 4
echo ---
echo.
echo # Hardware Guide
echo.
echo Content coming soon...
) > appendix\hardware-guide.md

cd ..

echo.
echo ========================================
echo All files created successfully!
echo ========================================
echo.
echo Total files: 53
echo.
echo You can now run: npm start
echo ========================================