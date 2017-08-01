#Davinci Tool Tracking 
This package is for da Vinci surgical robot tool tracking, this branch is for testing the results from tool images captured under the endoscpoes of the da Vinci robot. This is a post processing pkg.

#Dependences:
cwru_vision: https://github.com/cwru-robotics/cwru_vision.git

glm library: sudo apt-get install libglm-dev

vision_opencv: https://github.com/ros-perception/vision_opencv.git

cwru_davinci: https://github.com/cwru-robotics/cwru_davinci 

- tool model package: load the surgical tool model, construct tool geometry and acheive  virtual rendering.

To check the tool geometry and virtual rendering performance, run:

`rosrun tool_model tool_model_main`

- tool tracking package: integrate Particle Filter (PF) algorithm, Unscented Kalman Filter (UKF) algorithm

To run tracking algorithm:

`rosrun tool_tracking tracking_particle`

To run UKF tracking algorithm:

`rosrun tool_tracking tracking_kalman`











