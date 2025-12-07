---
sidebar_position: 4
---

# Software Setup: Preparing Your Physical AI Workspace 🛠️🖥️

Welcome to the practical side of setting up your Physical AI development environment! With your hardware ready and prerequisites understood, it's time to install the essential software that will power your robotic journey. This chapter provides a comprehensive, step-by-step guide to installing Ubuntu 22.04 LTS, ROS 2 Humble Hawksbill, Gazebo, NVIDIA Isaac Sim, and all necessary Python dependencies.

Getting your software stack correctly configured is a crucial first step, and while it might seem daunting, we've broken it down into manageable segments with clear commands and troubleshooting advice. By the end of this chapter, you'll have a fully functional development environment, ready to build and simulate intelligent humanoid robots. Let's get everything installed and verify your setup!

---

## 🎯 Learning Objectives

By the end of this chapter, you will be able to:

*   **Successfully install** Ubuntu 22.04 LTS, either as a standalone OS or in a dual-boot configuration.
*   **Perform a complete installation** of ROS 2 Humble Hawksbill and its core development tools.
*   **Install and configure** the Gazebo physics simulator for basic robotic simulations.
*   **Set up NVIDIA Isaac Sim**, including Omniverse Launcher and necessary assets.
*   **Manage Python environments** and install required dependencies using `pip`.
*   **Verify the functionality** of your entire Physical AI software stack with test commands.
*   **Troubleshoot** common installation issues encountered during setup.

---

## 1. Ubuntu 22.04 LTS Installation 🐧💿

As established in the [Hardware Requirements](./hardware-requirements.md) chapter, Ubuntu 22.04 LTS is the mandatory operating system for this course. For the best performance, a native installation is recommended.

### Option A: Clean Install (Recommended for Best Performance)

1.  **Download Ubuntu 22.04 LTS:**
    *   Visit the official Ubuntu website: [ubuntu.com/download/desktop](https://ubuntu.com/download/desktop)
    *   Download the **Ubuntu 22.04 LTS** Desktop ISO image.
2.  **Create a Bootable USB Drive:**
    *   Use a tool like **Rufus** (Windows) or **BalenaEtcher** (Windows/macOS/Linux) to flash the ISO image onto a USB drive (minimum 8GB).
3.  **Boot from USB:**
    *   Insert the USB drive into your computer.
    *   Restart your computer and access your BIOS/UEFI settings (usually by pressing `F2`, `F10`, `F12`, `Del`, or `Esc` during startup).
    *   Set the USB drive as the primary boot device.
4.  **Follow Installation Prompts:**
    *   Select "Install Ubuntu."
    *   Choose your language, keyboard layout, and network connection.
    *   For installation type, select "Erase disk and install Ubuntu." **WARNING: This will delete ALL data on your selected drive.** Ensure you have backed up any important files.
    *   Follow the remaining prompts (time zone, user creation).
    *   Restart your computer when prompted.

### Option B: Dual-Boot with Windows

1.  **Shrink Windows Partition:**
    *   In Windows, open "Disk Management" (search for it in the Start menu).
    *   Right-click on your main Windows partition (usually C:) and select "Shrink Volume."
    *   Shrink it by at least 200GB (or more, depending on your needs for Ubuntu). You'll create "Unallocated space."
2.  **Download and Create Bootable USB (Steps 1-2 from Option A).**
3.  **Boot from USB (Step 3 from Option A).**
4.  **Follow Installation Prompts:**
    *   When you reach the "Installation type" screen, select "**Install Ubuntu alongside Windows Boot Manager**."
    *   You will be able to choose how much of the "Unallocated space" to dedicate to Ubuntu.
    *   Complete the installation as prompted.
    *   On restart, you should see a GRUB boot menu allowing you to choose between Ubuntu and Windows.

### Post-Installation (Ubuntu)

1.  **Update Your System:**
    ```bash
    sudo apt update
    sudo apt upgrade -y
    sudo apt autoremove -y
    ```
2.  **Install Essential Build Tools and Drivers:**
    ```bash
    sudo apt install build-essential git curl wget vim htop software-properties-common -y
    ```
3.  **Install NVIDIA GPU Drivers:**
    *   **Crucial for GPU acceleration.**
    *   Open "Software & Updates" > "Additional Drivers" tab.
    *   Select the latest proprietary, tested NVIDIA driver (e.g., `nvidia-driver-535` or `nvidia-driver-545`) and click "Apply Changes."
    *   Alternatively, via command line:
        ```bash
        sudo apt update
        sudo apt install nvidia-driver-535 -y # Use the latest stable driver version
        sudo reboot
        ```
    *   **Verify Driver Installation:**
        ```bash
        nvidia-smi
        ```
        You should see information about your NVIDIA GPU.

:::warning Troubleshooting Ubuntu Installation
*   **Black Screen/Boot Issues:** Ensure your NVIDIA drivers are correctly installed. If you encounter issues, try booting with `nomodeset` kernel parameter (search online for how to add this in GRUB).
*   **Dual Boot not showing GRUB:** If Windows boots directly, you may need to disable "Fast Startup" in Windows Power Options. You might also need to use a tool like Boot-Repair from a live Ubuntu USB.
*   **UEFI Secure Boot:** You may need to disable Secure Boot in your BIOS/UEFI settings for the NVIDIA drivers to load correctly.
:::
---

## 2. ROS 2 Humble Hawksbill Installation 🤖🌿

This course uses **ROS 2 Humble Hawksbill**, which is officially supported on Ubuntu 22.04 LTS. We'll install the "desktop" version, which includes ROS, RViz, and Gazebo.

1.  **Set Up Locales:**
    ```bash
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    ```
2.  **Add ROS 2 Repository:**
    ```bash
    sudo apt update && sudo apt install software-properties-common -y
    sudo add-apt-repository universe -y
    sudo apt update
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    ```
3.  **Install ROS 2 Humble Desktop:**
    ```bash
    sudo apt update
    sudo apt install ros-humble-desktop -y
    ```
    This command will install ROS 2, RViz, `ros-humble-ros-base`, `ros-humble-perception-pcl` and the `ros-humble-gazebo-ros-pkgs`. It might take some time.
4.  **Install `colcon` Build Tools:**
    `colcon` is the recommended build tool for ROS 2.
    ```bash
    sudo apt install python3-colcon-common-extensions -y
    ```
5.  **Source ROS 2 Environment:**
    You need to source the ROS 2 setup files in every new terminal you open. To make it permanent:
    ```bash
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```
    *   If you use `zsh`, replace `.bashrc` with `.zshrc`.
6.  **Verify ROS 2 Installation:**
    ```bash
    ros2 --version
    ```
    You should see the ROS 2 CLI version.
    ```bash
    ros2 run demo_nodes_cpp talker
    ```
    Open a new terminal (which should now automatically source ROS 2) and run:
    ```bash
    ros2 run demo_nodes_py listener
    ```
    You should see "Hello World" messages being exchanged. Press `Ctrl+C` in both terminals to stop.

:::tip Troubleshooting ROS 2 Installation
*   **`ros2: command not found`:** Ensure you've sourced your `setup.bash` (or `setup.zsh`) file.
*   **Package errors during `apt install`:** Double-check the repository setup and try `sudo apt update --fix-missing`.
*   **`colcon build` issues:** Ensure you're in a ROS 2 workspace and have sourced the `/opt/ros/humble/setup.bash` (or `/opt/ros/humble/setup.zsh`)
:::
---

## 3. Gazebo Simulator Installation and Setup 🌍⚙️

Gazebo is a powerful 3D robotics simulator. The desktop installation of ROS 2 Humble typically includes Gazebo, specifically **Gazebo Garden**.

### Verify Gazebo Installation

You can check if Gazebo is already installed and runnable:

```bash
gazebo --version
```
You should see output similar to `Gazebo Garden version: X.X.X`. If not, or if you prefer a standalone installation:

### Standalone Gazebo Garden Installation (If not included with ROS 2)

1.  **Add Gazebo Repository:**
    ```bash
    sudo apt update
    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list'
    wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/gazebo-archive-keyring.gpg
    ```
2.  **Install Gazebo Garden:**
    ```bash
    sudo apt update
    sudo apt install gazebo-garden -y
    sudo apt install libgazebo-garden-dev -y # For development libraries
    ```
3.  **Test Gazebo:**
    ```bash
gazebo
    ```
    This should launch the Gazebo GUI. You can close it once it loads.

:::info Gazebo Versions
There are multiple Gazebo versions (Classic, Fortress, Garden, Harmonic). ROS 2 Humble primarily uses **Gazebo Garden**. Ensure you're working with the correct version.
:::
---

## 4. NVIDIA Isaac Sim Setup 💡🤖

NVIDIA Isaac Sim, built on NVIDIA Omniverse, is a high-fidelity simulator crucial for this course's advanced modules. Its installation is slightly different from typical package managers.

### 4.1. NVIDIA Omniverse Launcher Installation

1.  **Download Omniverse Launcher:**
    *   Go to the NVIDIA Omniverse website: [nvidia.com/omniverse](https://www.nvidia.com/omniverse/)
    *   Click on "Download" and then "Download Launcher" for Linux.
2.  **Install Launcher:**
    *   Open a terminal and navigate to your Downloads directory.
    *   Make the installer executable and run it:
        ```bash
        chmod +x omniverse-launcher-linux.run
        ./omniverse-launcher-linux.run
        ```
    *   Follow the on-screen instructions. The launcher will install to your home directory by default.
3.  **Login:**
    *   Once installed, open the Omniverse Launcher. You will need to log in with an NVIDIA account. Create one if you don't have one.

### 4.2. Install Isaac Sim via Omniverse Launcher

1.  **Install Nucleus:**
    *   In the Omniverse Launcher, go to the "Nucleus" tab.
    *   Click "Add Local Cache" and follow instructions to install a local Nucleus server. This is essential for managing assets.
2.  **Install Isaac Sim:**
    *   In the Omniverse Launcher, go to the "Exchange" tab.
    *   Search for "Isaac Sim" and click "Install."
    *   **Crucial:** Ensure you install a version compatible with ROS 2 Humble. The documentation for Isaac Sim will specify which Python and ROS 2 versions are supported. At the time of writing, Isaac Sim often targets Python 3.8/3.9, which might require managing separate Python environments (see Python dependencies section below).
3.  **Install Dependencies for Isaac Sim:**
    *   After installation, go to the "Library" tab, find Isaac Sim, click the three dots (`...`) menu, and select "Open Terminal."
    *   In the terminal, you'll find a script to install common Python dependencies for Isaac Sim. Run it:
        ```bash
        ./python.sh -m pip install --upgrade pip
        ./python.sh -m pip install numpy scipy
        ```
        (This `python.sh` script points to Isaac Sim's bundled Python interpreter.)

### 4.3. Test Isaac Sim

1.  **Launch Isaac Sim:**
    *   In the Omniverse Launcher, click "Launch" next to Isaac Sim.
    *   Once Isaac Sim loads, you should see the main interface. You can try loading one of the example scenes (e.g., from `File > Open > omniverse://localhost/NVIDIA/Assets/Scenes/`).

:::warning Troubleshooting Isaac Sim
*   **`GPU driver not detected` / `OpenGL errors`:** Ensure your NVIDIA drivers are up-to-date and correctly installed for your Ubuntu system. Check `nvidia-smi`.
*   **`Python version mismatch`:** Isaac Sim often uses a specific Python version. You may need to create a virtual environment (`venv` or `conda`) for your ROS 2 development with a compatible Python version, or carefully manage the `python.sh` interpreter that comes with Isaac Sim.
*   **`Error loading assets from Nucleus`:** Ensure your local Nucleus server is running and accessible. Check the Omniverse Launcher for its status.
*   **High GPU/CPU usage without action:** Isaac Sim is demanding. Ensure your hardware meets the [minimum requirements](./hardware-requirements.md).
:::
---

## 5. Python Dependencies & Virtual Environments 🐍📦

Managing Python dependencies is critical, especially when different tools require different Python versions or package sets. Using virtual environments is highly recommended to isolate your project dependencies.

1.  **Install `pip` and `venv` (if not already installed):**
    ```bash
    sudo apt install python3-pip python3-venv -y
    ```
2.  **Create a Virtual Environment for ROS 2 Development:**
    *   Navigate to your ROS 2 workspace (e.g., `~/ros2_ws`).
    *   Create a virtual environment:
        ```bash
        python3 -m venv ~/ros2_ws/venv
        ```
    *   Activate the virtual environment:
        ```bash
        source ~/ros2_ws/venv/bin/activate
        ```
        Your terminal prompt should change to `(venv) user@host:...`.
3.  **Install Common Python Packages:**
    With your virtual environment activated:
    ```bash
    pip install --upgrade pip
    pip install numpy scipy matplotlib transforms3d opencv-python-headless # Add other common packages as needed
    ```
4.  **Integrate ROS 2 Sourcing:**
    To ensure your virtual environment can find ROS 2 packages, you'll need to source the ROS 2 environment *after* activating your `venv`.
    ```bash
    echo "source /opt/ros/humble/setup.bash" >> ~/ros2_ws/venv/bin/activate
    ```
    *   **Note:** This modifies the `activate` script of your venv. Now, every time you activate your venv, ROS 2 will also be sourced.

:::tip Using `conda`
For advanced users or those who prefer `conda` for environment management, you can use `conda` instead of `venv`. Create a new environment (`conda create -n ros2_env python=3.10`), activate it (`conda activate ros2_env`), and then source ROS 2 (`source /opt/ros/humble/setup.bash`).
:::
---

## 6. Testing Your Complete Physical AI Setup ✅🔬

Now that all components are installed, let's run a quick series of tests to ensure everything is communicating correctly.

### Test 1: ROS 2 Talker-Listener with Python

Ensure your ROS 2 environment is sourced (or your venv is activated and configured to source ROS 2).

1.  **Terminal 1:**
    ```bash
    ros2 run demo_nodes_cpp talker
    ```
2.  **Terminal 2:**
    ```bash
    ros2 run demo_nodes_py listener
    ```
    Verify messages are exchanged. Press `Ctrl+C` in both terminals.

### Test 2: Gazebo World Launch

Launch a simple Gazebo world to confirm it's working with ROS 2 integration.

```bash
ros2 launch gazebo_ros gazebo.launch.py
```
This should open Gazebo with an empty world. Close the Gazebo GUI to stop.

### Test 3: RViz2 Visualization

RViz2 is the primary visualization tool for ROS 2.

```bash
rviz2
```
RViz2 should launch. You can add a `RobotModel` display and try loading a URDF to visualize.

### Test 4: Isaac Sim Basic Python Script

Assuming you have Isaac Sim installed and its Python environment accessible, try running a basic script.

1.  **Open Isaac Sim's Python Terminal:**
    *   From Omniverse Launcher, go to "Library", find Isaac Sim, click the three dots (`...`), and select "Open Terminal."
2.  **Run a Simple Script:**
    *   In this terminal, type:
        ```python
        import omni.usd
        print("Isaac Sim Python environment is working!")
        ```
    *   You should see the print statement, confirming the environment.

:::tip Troubleshooting Test Failures
*   **`command not found`:** Always ensure the relevant environment (`source ~/.bashrc`, `source venv/bin/activate`) is sourced in your terminal.
*   **Segmentation fault/Crashes:** Often indicates a driver issue (NVIDIA) or a conflict between installed libraries. Review driver installation and ensure `apt update/upgrade` is recent.
*   **Errors related to `rclpy` or `rosidl_runtime_py`:** Suggests issues with your Python virtual environment's integration with ROS 2 sourcing. Double-check the `echo "source /opt/ros/humble/setup.bash" >> ~/ros2_ws/venv/bin/activate` step.
:::
---

## Conclusion: Your AI Robotics Platform is Ready! 🎉🚀

Congratulations! You've successfully navigated the complex landscape of software installation and configured your dedicated Physical AI development environment. From Ubuntu to ROS 2, Gazebo, Isaac Sim, and essential Python libraries, your workstation is now a powerful platform ready to bring your intelligent robot dreams to life.

This robust setup provides the foundation for all the exciting modules to come. You're now equipped to delve into advanced ROS 2 development, high-fidelity simulations, cutting-edge AI integration, and ultimately, building your autonomous humanoid capstone. The hard part of setting up is done; the fun of building begins!

---

## 💡 Key Takeaways

*   **Ubuntu 22.04 LTS** is the mandatory base OS, requiring a clean install or dual-boot for optimal performance.
*   **NVIDIA GPU drivers** must be correctly installed and verified for any GPU-accelerated tasks.
*   **ROS 2 Humble Hawksbill** provides the core robotic nervous system, installed with `apt` and permanently sourced in your `.bashrc`.
*   **Gazebo Garden** (often bundled with ROS 2 desktop) enables physics-based simulations.
*   **NVIDIA Isaac Sim**, installed via the Omniverse Launcher, is essential for high-fidelity, AI-driven simulations.
*   **Python virtual environments** are crucial for managing dependencies, and should be configured to source ROS 2 upon activation.
*   Thorough **testing** of all components (ROS 2 talker/listener, Gazebo, RViz2, Isaac Sim Python) is vital to confirm a functional setup.

---

## 🏋️ Hands-On Exercise: Full System Check & Workspace Creation

This exercise guides you through a final, comprehensive check of your newly installed software stack and sets up your primary ROS 2 development workspace.

**Expected Time:** 25 minutes

**Requirements:**
*   A fully installed Ubuntu 22.04 system with all software components from this chapter.

**Instructions:**
1.  **Confirm ROS 2 and `colcon`:**
    *   Open a new terminal.
    *   Verify ROS 2 sourcing: `echo $ROS_DISTRO` (should output `humble`).
    *   Verify `colcon` is available: `colcon --version`.
2.  **Create Your Main ROS 2 Workspace:**
    *   Navigate to your home directory: `cd ~`
    *   Create a new directory for your primary workspace: `mkdir -p ~/physical_ai_ws/src`
    *   Navigate into the source directory: `cd ~/physical_ai_ws/src`
    *   Initialize a ROS 2 package for testing: `ros2 pkg create --build-type ament_python my_first_robot_pkg`
    *   Navigate back to the workspace root: `cd ~/physical_ai_ws`
3.  **Build Your Workspace:**
    *   Run `colcon build`. This should build your `my_first_robot_pkg` (even if it's empty, it validates the build system).
4.  **Test ROS 2 in Workspace:**
    *   Source your workspace: `source install/setup.bash`
    *   Run the talker/listener again, but this time referencing your workspace's setup:
        *   Terminal 1: `ros2 run demo_nodes_cpp talker`
        *   Terminal 2: `ros2 run demo_nodes_py listener`
    *   Verify communication.
5.  **Gazebo and RViz2 Check:**
    *   In a new terminal (sourced with your workspace): `ros2 launch gazebo_ros gazebo.launch.py` (Verify Gazebo launches).
    *   In another new terminal: `rviz2` (Verify RViz2 launches).
6.  **Isaac Sim Python Environment Check (Optional, but recommended):**
    *   Launch Isaac Sim from the Omniverse Launcher.
    *   Once loaded, go to the "Script Editor" (usually under `Window -> Scripting -> Script Editor`).
    *   Paste and run the following code:
        ```python
        import omni.usd
        print("Isaac Sim Python environment is working!")
        ```
    *   Observe the output in the console within Isaac Sim.

**Solution Hints:**
*   If `colcon build` fails, ensure your `CMakeLists.txt` and `package.xml` are correctly formed (though `ros2 pkg create` handles this for a new package).
*   Remember to source your *workspace* setup files (`source install/setup.bash`) after building a package to make its executables available.
*   Persistent issues might require reviewing the specific installation logs for error messages.

---

## 📚 Further Reading

*   **Ubuntu Installation Guide:** [ubuntu.com/tutorials/install-ubuntu-desktop](https://ubuntu.com/tutorials/install-ubuntu-desktop)
*   **ROS 2 Humble Installation (Official Documentation):** [docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
*   **Gazebo Garden Installation (Official Documentation):** [gazebosim.org/docs/garden/install_ubuntu](https://gazebosim.org/docs/garden/install_ubuntu)
*   **NVIDIA Omniverse Launcher Download:** [nvidia.com/omniverse](https://www.nvidia.com/omniverse/)
*   **Isaac Sim Installation Guide (Official Documentation):** [docs.omniverse.nvidia.com/isaacsim/latest/install_linux.html](https://docs.omniverse.nvidia.com/isaacsim/latest/install_linux.html) (Always refer to the latest official docs for Isaac Sim, as versions can change.)
*   **Python Virtual Environments:** [docs.python.org/3/library/venv.html](https://docs.python.org/3/library/venv.html)

---

**Next Chapter:** [Module 1: ROS 2 - The Robotic Nervous System Introduction](../module-1/introduction.md)
