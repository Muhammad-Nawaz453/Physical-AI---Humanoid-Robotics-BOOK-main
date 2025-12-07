#!/bin/bash

# Script to create the complete folder structure for Physical AI Textbook

echo "Creating Physical AI Textbook folder structure..."

# Navigate to docs folder
cd docs

# Create Getting Started section
mkdir -p getting-started
touch getting-started/overview.md
touch getting-started/prerequisites.md
touch getting-started/hardware-requirements.md
touch getting-started/software-setup.md

# Create Module 1: ROS 2
mkdir -p module-1
touch module-1/introduction.md
touch module-1/ros2-architecture.md
touch module-1/nodes-and-topics.md
touch module-1/services-and-actions.md
touch module-1/urdf-robot-description.md
touch module-1/rclpy-python-integration.md
touch module-1/launch-files.md
touch module-1/project-ros2-package.md

# Create Module 2: Gazebo & Unity
mkdir -p module-2
touch module-2/introduction.md
touch module-2/gazebo-fundamentals.md
touch module-2/physics-simulation.md
touch module-2/sensor-simulation.md
touch module-2/unity-integration.md
touch module-2/urdf-sdf-formats.md
touch module-2/environment-building.md
touch module-2/project-simulated-robot.md

# Create Module 3: NVIDIA Isaac
mkdir -p module-3
touch module-3/introduction.md
touch module-3/isaac-sim-overview.md
touch module-3/synthetic-data-generation.md
touch module-3/isaac-ros-hardware-acceleration.md
touch module-3/vslam-navigation.md
touch module-3/nav2-path-planning.md
touch module-3/perception-pipeline.md
touch module-3/reinforcement-learning.md
touch module-3/sim-to-real-transfer.md
touch module-3/project-perception-system.md

# Create Module 4: VLA
mkdir -p module-4
touch module-4/introduction.md
touch module-4/voice-to-action-whisper.md
touch module-4/llm-cognitive-planning.md
touch module-4/natural-language-commands.md
touch module-4/multimodal-interaction.md
touch module-4/gpt-integration.md
touch module-4/conversational-robotics.md
touch module-4/project-autonomous-humanoid.md

# Create Humanoid section
mkdir -p humanoid
touch humanoid/kinematics-dynamics.md
touch humanoid/bipedal-locomotion.md
touch humanoid/balance-control.md
touch humanoid/manipulation-grasping.md
touch humanoid/human-robot-interaction.md

# Create Capstone section
mkdir -p capstone
touch capstone/project-overview.md
touch capstone/requirements.md
touch capstone/architecture-design.md
touch capstone/implementation-guide.md
touch capstone/testing-validation.md
touch capstone/presentation-tips.md

# Create Appendix section
mkdir -p appendix
touch appendix/glossary.md
touch appendix/resources.md
touch appendix/troubleshooting.md
touch appendix/hardware-guide.md

echo "✅ Folder structure created successfully!"
echo "Total files created: $(find . -type f -name '*.md' | wc -l)"

# Navigate back
cd ..