# ROS-based Drone Simulation

A ROS-based simulation framework for Unmanned Aerial Vehicles (UAVs), integrating Gazebo for realistic environments and Python scripts for autonomous waypoint navigation.

## Features

* **Gazebo Integration**: Simulate UAVs in detailed environments, including maps with dynamic elements like vehicles.
* **ROS Launch Files**: Easily initialize simulation worlds with predefined configurations.
* **Autonomous Navigation**: Python scripts to command UAVs through a series of waypoints.
* **Modular Structure**: Organized directories for scripts, launch files, and simulation assets.([GitHub][1])

## Project Structure

```
UAV-Project/
├── python_scripts/
│   └── main.py
├── src/
├── deneme.rviz
├── deneme_world.sdf
├── README.md
└── .gitignore
```

* `python_scripts/main.py`: Script to initiate waypoint navigation.
* `deneme_world.sdf`: Gazebo world file defining the simulation environment.
* `deneme.rviz`: RViz configuration for visualizing the UAV's state.([GitHub][2])

## Getting Started

### Prerequisites

* **Operating System**: Ubuntu 20.04 or later
* **ROS**: Noetic Ninjemys
* **Gazebo**: Compatible version with ROS Noetic
* **Python**: 3.6 or later([GitHub][2])

### Installation

1. **Clone the Repository**

   ```bash
   git clone https://github.com/MahmutEsadErman/UAV-Project.git
   cd UAV-Project
   ```



2. **Install Dependencies**

   Ensure all necessary ROS packages and Python dependencies are installed.

   ```bash
   sudo apt-get update
   sudo apt-get install ros-noetic-<package-name>
   pip install -r requirements.txt
   ```



*Replace `<package-name>` with required ROS packages and ensure `requirements.txt` lists all Python dependencies.*

## Usage

1. **Launch the Simulation Environment**

   ```bash
   roslaunch rotors_gazebo mav_with_map.launch world_name:=basic_with_car
   ```



*This command initializes the Gazebo world defined in `deneme_world.sdf`.*

2. **Run the Waypoint Navigation Script**

   In a new terminal:

   ```bash
   cd UAV-Project
   python python_scripts/main.py
   ```



*The UAV will commence navigation through predefined waypoints.*

## 🖼️ Visualization

Use RViz to monitor the UAV's state and trajectory.([GitHub][3])

```bash
rosrun rviz rviz -d deneme.rviz
```



*Ensure RViz is configured correctly to display relevant topics and frames.*


For questions or suggestions, please open an issue on the repository or contact [Mahmut Esad Erman](https://github.com/MahmutEsadErman).

