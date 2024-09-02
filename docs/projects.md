---
title: Projects
author: Tamas Levendovics
---

# Projects

---


## Challenge levels and grades

---

Projects can be completed at three *Challenge levels*. The *Challenge level* determines the  **best** grade that can be received to the project! 

| Challenge level | Best grade |
| -------- | :-------: |
| Basic    |     3 |
| Advanced |     4 |
| Epic     |     5 |

!!! tip
	The projects are defined in a way that it is recommended to tart with the **Basic** level, and then gradually work towards **Epic**.

The projects are graded based on the follwoing aspects:

- Proved to be the student's own work
- Running results valid output
- Usage of versioning, usage of GitHub/GitLab/other repository
- Grading: 
    - completeness of the soultion
    - proper ROS communication
    - proper structure of the program
    - quality of implementation
    - documentation quality


---

### Grading

Personal attendance on the classes is mandatory (min 70%).

To pass the course, Tests and the Project must be passed (grade 2). One of the Test can be taken again.


!!! abstract "Grade"
	$Jegy = (Test1 + Test2 + 2 \times Project) / 4$ 
	
---

## Project topics

---

### 1. Mobil robot

#### A. Playground Robot

- [Gazebo install](https://gazebosim.org/docs/harmonic/install_ubuntu)
- [Setting up a robot simulation (Gazebo)](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html)

![playground_robot.png](img%2Fplayground_robot.png){:style="width:600px"}


#### B. TurtleBot4

- [TurtleBot4 Simulator Tutorial](https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_simulator.html)
- [TurtleBot4 GUI Docs](https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_common.html#menu-control)

![turtlebot.png](img%2Fturtlebot.png){:style="width:600px"}

#### C. PlatypOUs (ROS 1)

- [PlatypOUs GitHub](https://github.com/ABC-iRobotics/PlatypOUs-Mobile-Robot-Platform)

![](https://www.mdpi.com/sensors/sensors-22-02284/article_deploy/html/images/sensors-22-02284-g001.png){:style="width:600px"}

#### D. Any mobile robot

---



#### 1.1. Mobile robot obstacle avoidance

- **Basic:** SSimulator setup, testing SLAM. Implementation of ROS node(s) to read the sensor data and move the robot.
- **Advanced:** Implementation of a ROS system to detect obstacle. Calculation and execution of a trajectory avoiding the obstacle in the simulator, using any sensor of the robot.
- **Epic:** Implementation and testing on the real robot/impress me!

---

#### 1.2. Mobile robot path following

- **Basic:** Simulator setup. Implementation of ROS node(s) to read the sensor data and move the robot.
- **Advanced:** Implementation of a ROS system for path follwoing in the simulator, using any sensor of the robot (e.g., driving next to the wall with given distance using LIDAR).
- **Epic:** Implementation and testing on the real robot/impress me!



#### 1.3. Mobile robot object follwoing

- **Basic:** Simulator setup. Implementation of ROS node(s) to read the sensor data and move the robot.
- **Advanced:** Implementation of a ROS system to detect an object and follow it in the simulator, using any sensor of the robot(e.g., visual servoing).
- **Epic:** Implementation and testing on the real robot/impress me!

#### 1.4. Mobile robot action library

- **Basic:** Simulator setup. Implementation of ROS node(s) to read the sensor data and move the robot.
- **Advanced:** Implementation of a ROS action library containing simple actions and their execution (e.g., push object, move to object, turn around).
- **Epic:** Implementation and testing on the real robot/impress me!


---

### 2. Quadcopter

- [Gazebo install](https://gazebosim.org/docs/harmonic/install_ubuntu)
- [Setting up a robot simulation (Gazebo)](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html)



    ```bash
    ign gazebo -v 4 -r quadcopter.sdf
    ```


![quadcopter.png](img%2Fquadcopter.png){:style="width:600px"}

- **Basic:** Simulator setup. Implementation of ROS node(s) to read the sensor data and move the robot.  
- **Advanced:** ROS system implementation to control velocity/position.
- **Epic:** Impress me!

---


### 3. Any Gazebo simulaion

- [Gazebo install](https://gazebosim.org/docs/harmonic/install_ubuntu)
- [Setting up a robot simulation (Gazebo)](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html)
- [Gazebo World Examples](https://github.com/gazebosim/gz-sim/tree/gz-sim7/examples/worlds)

![pendulum.png](img%2Fpendulum.png){:style="width:600px"}


Based on discussion.

---


### 4. Gazebo simulation creation

- [Gazebo install](https://gazebosim.org/docs/harmonic/install_ubuntu)
- [Setting up a robot simulation (Gazebo)](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html)
- [Gazebo World Examples](https://github.com/gazebosim/gz-sim/tree/gz-sim7/examples/worlds)

![velocity.png](img%2Fvelocity.png){:style="width:600px"}

Based on discussion.

---

### 5. TurtleSim

- [Turtlesim Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html)
- [Koch snowflake](https://en.wikipedia.org/wiki/Koch_snowflake)

![turtle_xmas_fast.gif](img%2Fturtle_xmas_fast.gif){:style="width:600px"}

#### 5.1 Turtlesim Fraktál/Szöveg

- **Basic:** Implement a proportianl controller.
- **Advanced:** Draw fractal/text.
- **Epic:** Impress me!

---

### 6. DVRK

- [Download and compile dVRK 2](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/BuildROS2)
- [Marker examples](https://www.programcreek.com/python/example/88812/visualization_msgs.msg.Marker)

![PSM_coordinates.png](img%2FPSM_coordinates.png){:style="width:600px"}

#### 6.1 DVRK Interactive Marker

Graspable, movable marker for the DVRK simulator.

---

### 7. YouBot (Windows)

<iframe width="560" height="315" src="https://www.youtube.com/embed/qvBEQsGvC3M" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>


- [YouBot controller GitHub](https://github.com/ABC-iRobotics/YoubotDriver/tree/ROS)

---


#### 7.1. YouBot ROS integration

- **Basic:** YouBot repo build.
- **Advanced:** ROS wrapper/interface implementation, move the simulated arm in joint space from ROS.
- **Epic:** Implementation and testing on the real robot/impress me!


---

### X. Custom topic

---

Based on discussion.

---

## Links

- [Gazebo install](https://gazebosim.org/docs/harmonic/install_ubuntu)
- [Setting up a robot simulation (Gazebo)](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html)
- [Gazebo World Examples](https://github.com/gazebosim/gz-sim/tree/gz-sim7/examples/worlds)
- [YouBot controller GitHub](https://github.com/ABC-iRobotics/YoubotDriver/tree/ROS)
- [Download and compile dVRK 2](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/BuildROS2)
- [Marker examples](https://www.programcreek.com/python/example/88812/visualization_msgs.msg.Marker)
- [Turtlesim Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html)
- [Koch snowflake](https://en.wikipedia.org/wiki/Koch_snowflake)  








