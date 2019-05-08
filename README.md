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
#### 2.2.1 Robot and smart-wheel kinematics
* **&#x223C;<name\>/base_kin_model/r_wL_SW1 (double, default: 0.05)**\
Radius in meters of left wheel of Smart-Wheel 1. Use similar parameter for other smart-wheels up to 4.
* **&#x223C;<name\>/base_kin_model/r_wR_SW1 (double, default: 0.05)**\
Radius in meters of right wheel of Smart-Wheel 1. Use similar parameter for other smart-wheels up to 4.
* **&#x223C;<name\>/base_kin_model/s_w (double, default: 0.001)**\
Swivel caster off-set in meters of smartwheels. 
* **&#x223C;<name\>/base_kin_model/d_w (double, default: 0.08)**\
Separation in meters between wheels of a smartwheel.
* **&#x223C;<name\>/base_kin_model/xpos_SW1 (double, default: 0.21)**\
x position in meters of smart wheel 1 pivot with respect to the platform center of rotation. Use similar parameter for other smart-wheels up to 4.
* **&#x223C;<name\>/base_kin_model/ypos_SW1 (double, default: 0.21)**\
y position in meters of smart wheel 1 pivot with respect to the platform center of rotation. Use similar parameter for other smart-wheels up to 4.

#### 2.2.2 Smart-wheels configuration
* **&#x223C;<name\>/smart_wheels/enable (double, default: 1.0)**\
Enable smart-wheels. 
* **&#x223C;<name\>/smart_wheels/max_current (double, default: 10.0)**\
Maximum current in amperes per smart-wheel. Used by the outer control loop saturation.
* **&#x223C;<name\>/smart_wheels/pivot_offs_sw1 (double, default: 0.0)**\
Pivot offset in radians to calibrate zero pivot angular position of Smart-wheel 1. Use similar parameter for other smart-wheels up to 4.

#### 2.2.2 Platform configuration
* **&#x223C;<name\>/base_conf/max_vel_xy (double, default: 2.0)**\
Maximum platform translational velocity in meters per second.
* **&#x223C;<name\>/base_conf/max_acc_xy (double, default: 2.0)**\
Maximum platform translational acceleration in meters per second square.
* **&#x223C;<name\>/base_conf/max_vel_theta (double, default: 1.6)**\
Maximum platform rotational velocity in radians per second.
* **&#x223C;<name\>/base_conf/max_acc_theta (double, default: 1.6)**\
Maximum platform rotational acceleration in radians per second square.

#### 2.2.2 Controller configuration
#### 2.2.2.1 Outer loop translational platform velocity controllers
* **&#x223C;<name\>/platform_dxdy_cntr/Kgain (double, default: 200)**\
Proportional gain for translational velocity controller.
* **&#x223C;<name\>/platform_dxdy_cntr/I_fhz (double, default: 0.25)**\
Weak integrator frequency for translational velocity controller.
* **&#x223C;<name\>/platform_dxdy_cntr/LL_wz_fhz (double, default: 20)**\
Frequency of the zero for a lead-lag control action of the translational velocity controller.
* **&#x223C;<name\>/platform_dxdy_cntr/LL_wp_fhz (double, default: 20)**\
Frequency of the pole for a lead-lag control action of the translational velocity controller.
* **&#x223C;<name\>/platform_dxdy_cntr/LPF_fhz (double, default: 100)**\
Frequency for a first order low-pass-filter of the translational velocity controller.

#### 2.2.2.2 Outer loop rotational platform angular velocity controllers
* **&#x223C;<name\>/platform_dtheta_cntr/Kgain (double, default: 150)**\
Proportional gain for the platform angular velocity controller.
* **&#x223C;<name\>/platform_dtheta_cntr/I_fhz (double, default: 0.3)**\
Weak integrator frequency for the platform angular angular velocity controller.
* **&#x223C;<name\>/platform_dtheta_cntr/LL_wz_fhz (double, default: 20)**\
Frequency of the zero for a lead-lag control action of the platform angular velocity controller.
* **&#x223C;<name\>/platform_dtheta_cntr/LL_wp_fhz (double, default: 20)**\
Frequency of the pole for a lead-lag control action of the platform angular velocity controller.
* **&#x223C;<name\>/platform_dtheta_cntr/LPF_fhz (double, default: 100)**\
Frequency for a first order low-pass-filter of the platform angular velocity controller.

#### 2.2.2.3 Inner loop wheel angular velocity controllers
* **&#x223C;<name\>/platform_dvarphi_cntr/Kgain (double, default: 0.05)**\
Proportional gain for wheel angular velocity controller.
* **&#x223C;<name\>/platform_dvarphi_cntr/I_fhz (double, default: 0)**\
Weak integrator frequency for wheel angular velocity controller. It is recmmended not to use integral control action.
* **&#x223C;<name\>/platform_dvarphi_cntr/LL_wz_fhz (double, default: 2)**\
Frequency of the zero for a lead-lag control action of the wheel angular velocity controller.
* **&#x223C;<name\>/platform_dvarphi_cntr/LL_wp_fhz (double, default: 10)**\
Frequency of the pole for a lead-lag control action of the wheel angular velocity controller.
* **&#x223C;<name\>/platform_dvarphi_cntr/LPF_fhz (double, default: 50)**\
Frequency for a first order low-pass-filter of the angular velocity controller.
