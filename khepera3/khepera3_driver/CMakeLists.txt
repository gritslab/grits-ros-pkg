# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(khepera3_driver)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS roscpp message_generation std_msgs)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_message_files(FILES
	UnicycleControlMsg.msg
)

add_service_files(FILES
    SensorData.srv
    UnicycleControl.srv
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
    CATKIN_DEPENDS roscpp message_runtime std_msgs
    INCLUDE_DIRS include
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(driver src/Khepera3Driver.cpp)
target_link_libraries(driver ${catkin_LIBRARIES})
add_dependencies(driver khepera3_driver_generate_messages_cpp)

add_executable(driverNode src/Khepera3DriverAngularOrientedNode.cpp)
target_link_libraries(driverNode ${catkin_LIBRARIES})
add_dependencies(driverNode khepera3_driver_generate_messages_cpp)
