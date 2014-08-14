# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(khepera3_controller)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

catkin_package(
    CATKIN_DEPENDS roscpp geometry_msgs khepera3_driver khepera3_odometry optitrack_driver vicon_driver
    INCLUDE_DIRS include
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(unicycle src/UnicycleController.cpp ${catkin_LIBRARIES})
add_executable(pid src/PIDRegulator.cpp ${catkin_LIBRARIES})

target_link_libraries(unicycle ${catkin_LIBRARIES})
target_link_libraries(pid ${catkin_LIBRARIES})

add_dependencies(unicycle khepera3_controller_generate_messages_cpp khepera3_driver_generate_messages_cpp)
add_dependencies(pid khepera3_controller_generate_messages_cpp khepera3_driver_generate_messages_cpp)
