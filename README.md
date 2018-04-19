## Davinci Surgical Tool Tracking 
This package is for da Vinci surgical robot tool tracking

#### Dependences:
cwru_vision: https://github.com/cwru-robotics/cwru_vision.git

glm library: sudo apt-get install libglm-dev

vision_opencv: https://github.com/ros-perception/vision_opencv.git

cwru_davinci: https://github.com/cwru-robotics/cwru_davinci 

### Tool model package: load the surgical tool model, construct tool geometry and perform virtual rendering.

To check the tool geometry and virtual rendering performance, run:

`rosrun tool_model tool_model_main`

- tool tracking package: integrate Particle Filter (PF) algorithm, Unscented Kalman Filter (UKF) algorithm

### To run PF tracking algorithm:

`rosrun tool_tracking tracking_particle`

### To run UKF tracking algorithm:

`rosrun tool_tracking tracking_kalman`

### load model package: load CAD model in obj files

This package is to test the object loading via OpenGl glm library.

To test the vertices loading performance, cmake the package and ./load_main












