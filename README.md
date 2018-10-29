# ropod-low-level-control

In this repository we use the branches also as versions for different robots. See different "master" branches. Then we create develop and feature branches per version

# Ropod Low-level-control

## 1. Overview
In this repository the low-level-control atchitecture of the ropod is presented. The architecture is has two main loops: a wheel velocity control loop to add damping locally at the wheels, and a platform velocity control loop with force distribution. 

The wheel velocity loop is purposely designed with a low gain and no integral action to reduce pertubation among the different wheel controllers. Since these are now low gain controllers, if used alone, they will not be able to track properly the reference velocities. For the outer loop, there are three controlers, x and y local platform velocities and rotation velocity. The torque distribution block takes care that wheel torque saturation is avoided whenever possible. Constrained quadratic programing is used to compute the optimal torque distribution. When the total required wrench cannot be achieving without wheel torque saturation, the wrench is scale down accordingly.

## 2. ROS node API
### 2.1 Topics
#### 2.1.1 Published topics
* **&#x223C;<name\>/odom(nav_msgs/Odometry)**\
Robot's odometry

#### 2.1.2 Subscribed topics
* **&#x223C;<name\>/cmd_vel(geometry_msgs/Twist)**\
Velocity commands to be executed by the robot

### 2.2 Parameters
#### 2.2.1 Robot kinematics
* **&#x223C;<name\>/base_kin_model/r_wL_SW1 (double, default: 0.05)**\
Radius of left wheel of Smart-Wheel 1. Use similar parameter for other smart-wheels up to 4.
* **&#x223C;<name\>/base_kin_model/r_wR_SW1 (double, default: 0.05)**\
Radius of right wheel of Smart-Wheel 1. Use similar parameter for other smart-wheels up to 4.
* **&#x223C;<name\>/base_kin_model/s_w (double, default: 0.001)**\
Swivel caster off-set of smartwheels. 
* **&#x223C;<name\>/base_kin_model/d_w (double, default: 0.08)**\
Separation between wheels of a smartwheel.
* **&#x223C;<name\>/base_kin_model/xpos_SW1 (double, default: 0.21)**\
x position of smart wheel 1 pivot with respect to the platform center of rotation. Use similar parameter for other smart-wheels up to 4.
* **&#x223C;<name\>/base_kin_model/ypos_SW1 (double, default: 0.21)**\
y position of smart wheel 1 pivot with respect to the platform center of rotation. Use similar parameter for other smart-wheels up to 4.
