# P.A.T.H.
A robot system capable of Porter Assistance for Transportation in Hospitals.

The repository includes four different folders: MotorController, catkin_ws, test_ws and visual_ws.

## MotorController
This is the STM32 project files. The source code written for this project is found inside:
   - Sub MotorController/Core/Src (source files)
   - Sub MotorController/Core/Inc (header files)

The files inside the project is as follows:
- **main**: Main file
- **orientation**: Handles rotation encoder for the robot top plate.
- **MPU6050**: Handles initiation of IMU communication.
- **IMU_handler**: Handles IMU communication and data processing.
- **IMU_filter**: Implements the digital IMU filter.
- **frame_comm**: Handles UART communication with the Raspberry Pi.
- **circle_queue_Vector3**: Circular queue structure of 3 dimensional vectors.
- **microsecond_counter**: Used for measuring execution time with microsecond precision.

## catkin_ws
Mangler skriv

## test_ws
Mangler skriv

## visual_ws
This ROS workspace is used for simulating bed movement using two robots.
Relevant files:
- **visual_ws/src/path_visual/launch/display_gazebo.launch**: Launch file that starts the Gazebo simulation and spawns the bed and robots.
- **visual_ws/src/path_visual/urdf/robot.xacro**: URDF files that describes the links, joints, bodies and inertias that comprise the simulated model of the bed and robots.
- **visual_ws/src/visual_control/scripts/robot_control.py**: Script that takes incomming commands for bed movement and translates it into robot movement.



## Missing files

A few ROS packages have been removed from the source code due to upload problems. The following packages have been removed:

 - catkin_ws/src/openslam_gmapping/
 - catkin_ws/src/robot_pose_ekf/
 - catkin_ws/src/rplidar_ros/
 - catkin_ws/src/slam_gmapping/
 - test_ws/src/navigation/
 - test_ws/src/robot_localization/

