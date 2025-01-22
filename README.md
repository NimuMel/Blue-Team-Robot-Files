Here's a complete and polished version of the README file for your GitHub project:

---

# Blue Team Robot Files

This repository contains the code and resources developed by the **Blue Team** for the Senior Project in UNCA's JEM Mechatronics Program. The project focuses on designing, building, and programming an omni-directional robot capable of navigating a defined arena, collecting objects, and operating autonomously.

![Robot](https://img.shields.io/badge/Robot-Omni--Directional-blue) ![License](https://img.shields.io/badge/License-MIT-green)

---

## Table of Contents
1. [Project Overview](#project-overview)
2. [Features](#features)
3. [Repository Structure](#repository-structure)
4. [Installation](#installation)
5. [Usage](#usage)
6. [Dependencies](#dependencies)
7. [Contributing](#contributing)
8. [License](#license)

---

## Project Overview

The Blue Team's Senior Project centers around building a 4-wheel omni-directional robot. The robot uses advanced control algorithms and sensor fusion to perform tasks such as object collection and autonomous navigation.

Key goals:
- Develop a robot capable of smooth, omni-directional movement.
- Use a combination of sensors (Lidar, IMU, encoders) to detect and navigate obstacles.
- Incorporate ROS for modular and scalable software design.

---

## Features

- **Omni-Wheel Drive**: Precise, all-directional movement using inverse kinematics.
- **Sensor Fusion**: Combines data from IMU and encoders using a Kalman filter for accurate position estimation.
- **ROS Integration**: Prebuilt packages to support control, navigation, and visualization.
- **Joystick Control**: Real-time manual operation using an Xbox controller.
- **Dynamixel Servo Management**: Home and operate X-Series servos with intelligent feedback.

---

## Repository Structure

```plaintext
Blue-Team-Robot-Files/
├── OpenCR_Code/           # Firmware and control logic for the OpenCR board
├── Sensor_Test_Code/      # Scripts for testing sensors (Lidar, IMU, etc.)
├── catkin_ws/             # ROS workspace
│   ├── src/
│   │   ├── mobile_ik/     # Inverse kinematics for omni-wheels
│   │   ├── pose_est/      # Pose estimation and tracking
│   │   ├── pose_controller/  # Local pose controller for autonomous navigation
├── LICENSE                # License file
└── README.md              # Project documentation
```

---

## Installation

1. **Clone the repository**:
   ```bash
   git clone https://github.com/NimuMel/Blue-Team-Robot-Files.git
   cd Blue-Team-Robot-Files
   ```

2. **Set up the ROS workspace**:
   ```bash
   cd catkin_ws
   catkin_make
   source devel/setup.bash
   ```

3. **Install dependencies**:
   - [Install ROS](http://wiki.ros.org/ROS/Installation)
   - Follow the instructions for setting up the [Dynamixel SDK](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/).

---

## Usage

### 1. Run the ROS Nodes
Launch the robot control system:
```bash
roslaunch robot_control control.launch
```

### 2. Test Sensor Integration
Run the sensor test scripts to verify hardware functionality:
```bash
rosrun sensor_test lidar_test.py
```

### 3. Run the Joystick Controller
Start manual control using an Xbox controller:
```bash
roslaunch joy_control joy_control.launch
```

---

## Dependencies

Ensure the following are installed:
- ROS (Robot Operating System)
- OpenCR board firmware
- Dynamixel X-Series servos and SDK
- Python 3.x (for testing scripts)
- Lidar Sensor libraries
- Xbox controller drivers (for joystick integration)

---

## Contributing

We welcome contributions to improve the project! Here's how you can get involved:
1. Fork the repository.
2. Create a new branch:
   ```bash
   git checkout -b feature/YourFeature
   ```
3. Commit your changes:
   ```bash
   git commit -m "Add your descriptive message"
   ```
4. Push your changes:
   ```bash
   git push origin feature/YourFeature
   ```
5. Open a Pull Request.

---

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for more details.

---

Feel free to modify this README to suit any specific updates or additional features of your project! Let me know if you’d like help with anything else.
