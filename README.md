# ROS2 node for PS5 Dual Sense motion sensor

* Publisher:
```
ros2 run dual_sense_motion dual_sense_motion
```
Outputs attitude, angular velocity, and acceleration to the /imu topic.  
Since there is no gravity feedback in the Y-axis direction,
with the current Kalman filter gain, Y-axis direction will 
return to zero with First-order lag response with a time constant
of 1 second.
Gain and magnifications should be adjusted further.

* Test subscriber:
```
ros2 run dual_sense_motion simple_sub_imu
```
Made a little easier to read than `ros2 topic echo --csv /imu`
