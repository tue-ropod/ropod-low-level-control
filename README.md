# ropod-low-level-control

In this repository we use the branches also as versions for different robots. See different "master" branches. Then we create develop and feature branches per version

# Ropod Low-level-control

## 1. Overview
In this repository the low-level-control atchitecture of the ropod is presented. The architecture is has two main loops: a wheel velocity control loop to add damping locally at the wheels, and a platform velocity control loop with force distribution. 

The wheel velocity loop is purposely designed with a low gain and no integral action to reduce pertubation among the different wheel controllers. Since these are now low gain controllers, if used alone, they will not be able to track properly the reference velocities. For the outer loop, there are three controlers, x and y local platform velocities and rotation velocity. The torque distribution block takes care that wheel torque saturation is avoided whenever possible. Constrained quadratic programing is used to compute the optimal torque distribution. When the total required wrench cannot be achieving without wheel torque saturation, the wrench is scale down accordingly.
