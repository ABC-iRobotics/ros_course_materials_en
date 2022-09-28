---
title: Principles of robotics. Programming a da Vinci surgical robot in simulated environment. 
author: Tamas D. Nagy
---

# 05. Principles of robotics. Programming a da Vinci surgical robot in simulated environment. 

---

## Lecture

--- 

!!! warning
	**ZH1** (ROS principles, publisher, subscriber. Python principles. Principles of robotics.) **October 11.**




### Rigid body motion

---

![](img/merev_test_helyzete.png){:style="width:200px" align=right}

!!! abstract "Def. Rigid body"
    A rigid body is defined as a body on which the distance between two points remains constant in time regardless of the force applied on it.


- Shape and the volume of the rigid bodies are also constant.
- The **pose** of a rigid body can be given by the three coordinates of three of its points that do not lie on the same straight line.

![](img/merev_test_transzlacio.png){:style="width:200px" align=right}

- The **pose** of a rigid body can be described in a more expressive way by the three coordinates of one of its points chosen arbitrarily **position** and the body's **orientation**.



- The **motion of rigid bodies** is composed by two elemental motions: **translation** and **rotation**. 



-  During **translation**, all points of the body move along straight, parallel lines. 

![](img/merev_test_rotacio.png){:style="width:200px" align=right}

- During **rotation**, the position of the points of the rotational axis are constant, and the other points of the body move along circles in planes perpendicular to the axis of rotation.


- The **free motion** of rigid bodies can always be expressed  as the superposition of a translational motion and a rotation around a single axis.

---

### 3D transfomrations

---


- **Position:** 3D offset vector 
![](https://d2t1xqejof9utc.cloudfront.net/pictures/files/19711/original.png?1367580819){:style="width:250px" align=right}
- **Orientation:** 3 x 3 rotation matrix
    - further orientation representations: Euler-angles, RPY, angle axis, quaternion

- **Pose**: 4 × 4 (homogenous) transformation matrix
- **Frame**: origin, 3 axes, 3 base vectors, right hand rule
- **Homogenous transformation:** rotation and translation in one transfromation
    - e.g., for the rotation $\mathbf{R}$ and translation $\mathbf{v}$:

$$
\mathbf{T} = \left[\matrix{\mathbf{R} & \mathbf{v}\\\mathbf{0} & 1 }\right] = \left[\matrix{r_{1,1} & r_{1,2} & r_{1,3} & v_x\\r_{2,1} & r_{2,2} & r_{2,3} & v_y\\r_{3,1} & r_{3,2} & r_{3,3} & v_z\\\ 0 & 0 & 0 & 1 }\right]
$$

- **Homogenous coordinates:** 
    - **Vector:** extended with 0, $\mathbf{a_H}=\left[\matrix{\mathbf{a} \\ 0}\right]=\left[\matrix{a_x \\ a_y \\ a_z \\ 0}\right]$
    - **Point:** extended by 1, $\mathbf{p_H}=\left[\matrix{\mathbf{p} \\ 1}\right]=\left[\matrix{p_x \\ p_y \\ p_z \\ 1}\right]$
    - Applying transfomrations is much easier:

$$
\mathbf{q} = \mathbf{R}\mathbf{p} + \mathbf{v} \to \left[\matrix{\mathbf{q} \\ 1}\right] = \left[\matrix{\mathbf{R} & \mathbf{v}\\\mathbf{0} & 1 }\right]\left[\matrix{\mathbf{p} \\ 1}\right]
$$

- **Degrees of Freedom** (DoF): the number of independent parameters.

---

### Principles of robotics

---

![](img/segments.png){:style="width:400px" align=right}

- Robots are built of: **segments** (or links) és **joints**
- **Task space** (or cartesian space):
    - 3D space around us, where the task, endpoint trajectories, obstacles are defined.
    - **TCP** (Tool Center Point): Frame fixed to the end effector of the robot.
    - **Base frame**, **world frame**
- **Joint space**:
    -  Properties or values regarding the joints.
    -  Low-level controller.
    -  Joint angles, joint velocities, accelerations, torques....


---

### Python libraries

---

#### Numpy

---

- Python library
- High dimension arrays and matrices
- Mathematical functions

```python
import numpy as np

# Creating ndarrays
a = np.zeros(3)
a.shape
a.shape=(3,1)
a = np.ones(5)
a = np.empty(10)
l = np.linspace(5, 10, 6)
r = np.array([1,2])    # ndarray from python list
r = np.array([[1,2],[3,4]])
type(r)

# Indexing
l[0]
l[0:2]
l[-1]
r[:,0]

# Operations on ndarrays
r_sin = np.sin(r)
np.max(r)
np.min(r)
np.sum(r)
np.mean(r)
np.std(r)

l < 7
l[l < 7]
np.where(l < 7)

p = np.linspace(1, 5, 6)
q = np.linspace(10, 14, 6)

s = p + q
s = p * q
s = p * 10
s = p + 10
s = p @ q    # dot product
s = r.T
```
If not installed:

```bash
pip3 install numpy
```

#### Matplotlib

- Visualization in python
- Syntax similar to Matlab

```python
import numpy as np
from matplotlib import pyplot as plt

X = np.linspace(-np.pi, np.pi, 256)
C, S = np.cos(X), np.sin(X)

plt.plot(X, C)
plt.plot(X, S)

plt.show()
```

If not installed:

```bash
pip3 install matplotlib
```




---

## Practice

---


### 1. DVRK install

The da Vinci Surgical System is used to perform minimally invasive surgeries by teleoperation. The da Vinci Research Kit (DVRK) is an open-source hardware and software platform, offers, amongst others, reading and writing all the joints of the da Vinci, and also simulators for each arm. The DVRK software can be built as follows.

---

1. On Ubuntu 20.04 the following packages are required:


    ```bash
    sudo apt install libxml2-dev libraw1394-dev libncurses5-dev qtcreator swig sox espeak cmake-curses-gui cmake-qt-gui git subversion gfortran libcppunit-dev libqt5xmlpatterns5-dev python3-wstool python3-catkin-tools python3-osrf-pycommon ros-noetic-rviz
    ```
    
    ---
    
2. Download and install DVRK packages:

    ```bash
    cd ~/catkin_ws                     # go in the workspace
    wstool init src                    # we're going to use wstool to pull all the code from github
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release # all code should be compiled in release mode
    cd src                             # go in source directory to pull code
    wstool merge https://raw.githubusercontent.com/jhu-dvrk/dvrk-ros/master/dvrk_ros.rosinstall # or replace master by devel
    wstool up                          # now wstool knows which repositories to pull, let's get the code
    cd ~/catkin_ws
    catkin build --summary             # ... and finally compile everything
    ```

    ---
    
3. Restart the terminal then launch the RViz simulation of the PSM1 (Patient Side Manipulator):

    ```bash
    roslaunch dvrk_robot dvrk_arm_rviz.launch arm:=PSM1 config:=/home/$(whoami)/catkin_ws/src/cisst-saw/sawIntuitiveResearchKit/share/console/console-PSM1_KIN_SIMULATED.json
    ```

    !!! tip
        `Roslaunch` also launches the ROS master, if there is no ROS master running.

<iframe width="560" height="315" src="https://www.youtube.com/embed/QksAVT0YMEo" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

    ---

### 2. PSM subscriber

---

    
1. Create a new file named `psm_grasp.py` in the `~/catkin_ws/src/ros_course/scripts` folder. Add it to the `CMakeLists.txt`, as usually.

    ---
    
2. Check the topics and nodes of the simulator using the commands learned earlier (`rostopic list`, `rosrun rqt_graph rqt_graph`, etc.). PSM1 publishes the pose of the TCP and the angle of the jaws into the topics below. Subscribe to these topic in `psm_grasp.py` and store the current values into variables.

    ```bash
    /PSM1/measured_cp
    /PSM1/jaw/measured_js
    ```
    
    ---

3. Build and run the node:

    ```bash
    cd ~/catkin_ws
    catkin build ros_course
    rosrun ros_course psm_grasp.py 
    ```

    ---

### 3. Move the TCP along a linear trajectory

---

![](img/PSM_coordinates.png){:style="width:400px" align=right}

1. PSM1 expects commands regarding the pose of the TCP and the angle of the jaws from the topics below. Create publishers to these topic in `psm_grasp.py`.

    ```bash
    /PSM1/servo_cp
    /PSM1/jaw/servo_jp
    ```

    ---

2. Implement a method that moves the TCP to the desired position along a linear trajectory. Send the gripper to the position (0.0, 0.05, -0.12), leave the orientation as it is. Let the sampling time **dt** be 0.01s.

    ```python
    def move_tcp_to(self, target, v, dt):
    ```
    !!! tip
         Use the function `np.linspace(start, stop, num)` to create the array of **t** values (T). This function can also be used to create the linear trajectory along the axes **x, y, z** in separate arrays X, Y and Z.

    ---
    
3.  Write a method that can open and close the gripper jaws, also along a linear trajectory.


    ```python
    def move_jaw_to(self, target, omega, dt):
    ```
    
    ![](img/lin.png){:style="width:700px" align=right}
    
    ---
    
### 4. Dummy marker

---

1. Write a node that creates a virtual marker that can be grasped publishing`visualization_msgs/Marker` messages. Create a new file named `dummy_marker.py` in the `~/catkin_ws/src/ros_course/scripts` folder. Add it to the `CMakeLists.txt`, as usually. Copy the following code into the file `dummy_marker.py`:

     ```python
    import rospy
    from visualization_msgs.msg import Marker

    def marker(position):
        rospy.init_node('dummy_target_publisher', anonymous=True)
        pub = rospy.Publisher('dummy_target_marker', Marker, queue_size=10)
        rate = rospy.Rate(10) # 10hz
        i = 0
        while not rospy.is_shutdown():
            marker = Marker()
            marker.header.frame_id = 'PSM1_psm_base_link'
            marker.header.stamp = rospy.Time()
            marker.ns = "dvrk_viz"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.MODIFY
            marker.pose.position.x = position[0]
            marker.pose.position.y = position[1]
            marker.pose.position.z = position[2]
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.008
            marker.scale.y = 0.008
            marker.scale.z = 0.008
            marker.color.a = 1.0 # Don't forget to set the alpha!
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0;

            #rospy.loginfo(marker)
            pub.publish(marker)
            i = i + 1
            rate.sleep()

    if __name__ == '__main__':
        try:
            marker([-0.05, 0.08, -0.12])
        except rospy.ROSInterruptException:
            pass
    ```
    ---
    
2.  Build and run the node. Visualize the marker in RViz.

    ---


### 5. Grasp the marker

---

1. Subscribe to the topic with the marker position `dummy_target_publisher` the file `psm_grasp.py`.

    ---

2. Implement a method in `psm_grasp.py` to grasp the generated marker with PSM1.

    !!! note
        Some values tends to stuck in the simulator. Thus, at the beginning of the program, it is a good idea to reset the arm:
        ```python
        #Reset the arm
        psm.move_tcp_to([0.0, 0.0, -0.12], 0.01, 0.01)
        psm.move_jaw_to(0.0, 0.1, 0.01)
        ```


---

## Links

- [Download and compile dVRK](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/CatkinBuild)
- [Marker examples](https://www.programcreek.com/python/example/88812/visualization_msgs.msg.Marker)
- [Numpy vector magnitude](https://numpy.org/doc/stable/reference/generated/numpy.linalg.norm.html)
- [Numpy linspace](https://numpy.org/doc/stable/reference/generated/numpy.linspace.html)





















