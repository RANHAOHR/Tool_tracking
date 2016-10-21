cmake_minimum_required(VERSION 2.8)
project(test_opengl)


#uncomment next line to use OpenCV library
find_package(OpenGL REQUIRED)

#uncomment the next line to use the point-cloud library
#find_package(PCL 1.7 REQUIRED)

#uncomment the following 4 lines to use the Eigen library
#find_package(cmake_modules REQUIRED)
#find_package(Eigen3 REQUIRED)
#include_directories(${EIGEN3_INCLUDE_DIR})
#add_definitions(${EIGEN_DEFINITIONS})

#include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})
find_package(GLEW REQUIRED)
find_package(GLUT REQUIRED)
find_package(GLU REQUIRED)


include_directories(SYSTEM ${OPENGL_INCLUDE_DIRS}  ${GLEW_INCLUDE_DIRS} ${GLUT_INCLUDE_DIRS} ${GLU_INCLUDE_DIRS} )

# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# use carefully;  can interfere with point-cloud library
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries: uncomment the following and edit arguments to create a new library
# cs_add_library(my_lib src/my_lib.cpp)   

# Executables: uncomment the following and edit arguments to compile new nodes
# may add more of these lines for more nodes from the same package
add_executable(load_main load_main.cpp)


#the following is required, if desire to link a node in this package with a library created in this same package
# edit the arguments to reference the named node and named library within this package
# target_link_library(example my_lib), put GLEW IN FRONT OF OTHERS
target_link_libraries(load_main ${OpenGL_LIBRARIES} ${GLUT_LIBRARY} ${GLU_LIBRARY} )
#target_link_libraries(loadfeature ${OpenGL_LIBRARIES} ${GLEW_LIBRARY} ${GLUT_LIBRARY} ${GLU_LIBRARY})

    
