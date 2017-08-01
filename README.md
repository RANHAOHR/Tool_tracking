# Tool tracking package

This package is for the da Vinci surgical robot tool tracking in simulatin environment, Gazebo.

#Dependences:
pkgs:

cwru_vision: https://github.com/cwru-robotics/cwru_vision.git

glm library: sudo apt-get install libglm-dev

vision_opencv: https://github.com/ros-perception/vision_opencv.git

xform_utils: private

cwru_davinci: private

The state vector is stored as (g_CT, join_angle 4, 5, 6).

-To run particle filter tracking algorithm:

`rosrun tool_tracking tracking_particle`

-To run uscented Kalman filter tracking algorithm:

`rosrun tool_tracking tracking_kalman`



 






