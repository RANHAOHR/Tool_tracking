cmake_minimum_required(VERSION 2.8.3)
project(tool_model)

#uncomment next line to use OpenCV library
#find_package(catkin_simple REQUIRED)
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS roscpp message_generation std_msgs sensor_msgs cwru_opencv_common) 

#uncomment the next line to use the point-cloud library
#find_package(PCL 1.7 REQUIRED)

#uncomment the following 4 lines to use the Eigen library
#find_package(cmake_modules REQUIRED)
#find_package(Eigen3 REQUIRED)
#include_directories(${EIGEN3_INCLUDE_DIR})
#add_definitions(${EIGEN_DEFINITIONS})

#catkin_simple()

include_directories(include ${catkin_INCLUDE_DIRS} )
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS} )

catkin_package(CATKIN_DEPENDS roscpp message_runtime std_msgs sensor_msgs cwru_opencv_common)
catkin_package(
	DEPENDS EIGEN_DEP
	LIBRARIES tool_model_lib
	# CATKIN_DEPENDS
	INCLUDE_DIRS include
)


# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# use carefully;  can interfere with point-cloud library
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -pg -Q" )
# SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pg -Q")


# Libraries: uncomment the following and edit arguments to create a new library
# cs_add_library(my_lib src/my_lib.cpp)   
add_library(tool_model_lib src/tool_model.cpp)
# Executables: uncomment the following and edit arguments to compile new nodes
# may add more of these lines for more nodes from the same package
add_executable(showing_image src/showing_image.cpp)
add_executable(test_seg src/test_seg.cpp)
add_executable(tool_model_main src/tool_model_main.cpp)


#the following is required, if desire to link a node in this package with a library created in this same package
# edit the arguments to reference the named node and named library within this package
# target_link_library(example my_lib)

target_link_libraries(tool_model_lib
      ${OpenCV_LIBRARIES}
      ${catkin_LIBRARIES}
)
target_link_libraries(test_seg ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(tool_model_main tool_model_lib ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} )
target_link_libraries(showing_image ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
