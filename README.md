Blue Team Robot Files
This repository contains the code and resources developed by the Blue Team for the Senior Project in UNCA's JEM Mechatronics Program. It serves as a centralized storage for all project-related files, including robot control algorithms, sensor integration, and simulation environments.

Table of Contents
Project Overview
Repository Structure
Getting Started
Dependencies
Usage
Contributing
License
Project Overview
The Blue Team's Senior Project focuses on developing a comprehensive control system for an omni-directional robot. Key features include:

Omni-Wheel Drive Control: Implementing algorithms to manage the movement of the robot in any direction.
Sensor Integration: Incorporating various sensors for environment perception and navigation.
Simulation Environment: Setting up a simulation framework to test and validate control strategies before deployment.
Repository Structure
The repository is organized as follows:

OpenCR_Code/: Contains firmware and control code for the OpenCR board.
Sensor_Test_Code/: Includes scripts and programs for testing individual sensors.
catkin_ws/: ROS workspace containing packages for robot control, sensor fusion, and simulation.
Getting Started
To get a local copy of the project up and running, follow these steps:

Clone the repository:
bash
Copy
Edit
git clone https://github.com/NimuMel/Blue-Team-Robot-Files.git
Navigate to the project directory:
bash
Copy
Edit
cd Blue-Team-Robot-Files
Dependencies
Ensure the following software and libraries are installed:

ROS (Robot Operating System): Required for the catkin_ws packages.
OpenCR Board Drivers: Necessary for uploading firmware to the OpenCR board.
Sensor Libraries: Depending on the sensors used, install the appropriate libraries.
Usage
Building the ROS Workspace:
bash
Copy
Edit
cd catkin_ws
catkin_make
Sourcing the Workspace:
bash
Copy
Edit
source devel/setup.bash
Running the Robot Control Node:
bash
Copy
Edit
roslaunch robot_control control.launch
Contributing
Contributions are welcome. Please follow these steps:

Fork the repository.
Create a new branch (git checkout -b feature/YourFeature).
Commit your changes (git commit -m 'Add some feature').
Push to the branch (git push origin feature/YourFeature).
Open a Pull Request.
License
This project is licensed under the MIT License. See the LICENSE file for details.

Note: This README is based on the current structure and objectives of the Blue Team's Senior Project. For detailed information, please refer to the specific directories and files within the repository.


Sources
