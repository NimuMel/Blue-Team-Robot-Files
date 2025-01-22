Here’s a rewritten version of your README with a detailed and polished structure:

---

# Blue Team Robot Files

This repository stores all code and resources for the **Senior Project** by the **Blue Team** of UNCA's JEM Mechatronics Program. The project aims to design, build, and program an omni-directional robot capable of navigating a controlled arena, collecting objects, and operating both autonomously and manually.

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

The **Blue Team Senior Project** focuses on creating a highly versatile omni-directional robot. This project involves applying concepts from mechatronics, robotics, and programming to achieve:

- Precise and smooth robot movement in all directions.  
- Object collection and handling using specialized mechanisms.  
- Autonomous and manual operation modes.  
- Integration of sensors for environmental awareness and navigation.  

This project serves as a culmination of skills learned in the JEM Mechatronics Program, demonstrating both theoretical and practical expertise.

---

## Features

- **Omni-Wheel Drive**: Enables smooth movement in all directions using 4 omni-wheels and inverse kinematics.  
- **Joystick Integration**: Real-time manual control via an Xbox controller.  
- **Autonomous Navigation**: Utilizes sensor data for obstacle avoidance and precise movement.  
- **Sensor Fusion**: Combines IMU and encoder data through a Kalman filter for accurate position tracking.  
- **ROS Support**: Provides modular software packages for seamless integration and visualization.  
- **Dynamixel Servo Management**: Efficient control of X-Series servos with homing and position-based operation.  

---

## Repository Structure

```plaintext
Blue-Team-Robot-Files/
├── OpenCR_Code/           # Firmware and control logic for the OpenCR board
├── Sensor_Test_Code/      # Sensor testing scripts (IMU, Lidar, etc.)
├── catkin_ws/             # ROS workspace
│   ├── src/
│   │   ├── omni_wheel_ik/     # Inverse kinematics for omni-wheels
│   │   ├── pose_est/      # Pose estimation and position tracking
│   │   ├── pose_controller/  # Local pose control for autonomous navigation
├── LICENSE                # Licensing information
└── README.md              # Project documentation
```

---

## Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/NimuMel/Blue-Team-Robot-Files.git
   cd Blue-Team-Robot-Files
   ```

2. Set up the ROS workspace:
   ```bash
   cd catkin_ws
   catkin_make
   source devel/setup.bash
   ```

3. Install dependencies:
   - [ROS](http://wiki.ros.org/ROS/Installation) (tested with Ubuntu and ROS Noetic).  
   - Follow the setup instructions for [Dynamixel SDK](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/).  

## Dependencies

Ensure the following dependencies are installed:

- **ROS**: Required for running the ROS nodes and packages.  
- **OpenCR Board Firmware**: Upload firmware to the OpenCR microcontroller.  
- **Dynamixel X-Series Servos**: Includes setup and operation of Dynamixel actuators.  
- **Lidar Sensor**: Driver and integration for distance measurement.  
- **Xbox Controller Drivers**: Used for joystick integration.  

---

## Contributing

We welcome contributions from the community! To contribute:

1. Fork the repository.  
2. Create a new branch for your feature or bug fix:  
   ```bash
   git checkout -b feature/your-feature
   ```  
3. Commit your changes:  
   ```bash
   git commit -m "Add your descriptive message"
   ```  
4. Push the branch to your forked repository:  
   ```bash
   git push origin feature/your-feature
   ```  
5. Open a Pull Request, and provide a detailed description of your changes.

---

## License

This project is licensed under the MIT License. For more details, see the [LICENSE](LICENSE) file.

---

This README is designed to be comprehensive and user-friendly. Let me know if you’d like to further customize any sections or add additional details!
