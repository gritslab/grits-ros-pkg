# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(arduino_serial_port)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS roscpp message_generation)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_service_files(FILES
    SerialPacketRead.srv
    SerialPacketReadEOL.srv
    SerialPacketWrite.srv
)

generate_messages(DEPENDENCIES )

catkin_package(
    CATKIN_DEPENDS roscpp message_runtime
    INCLUDE_DIRS include
)

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(arduino_serial_port src/ArduinoSerialPort.cpp)
target_link_libraries(arduino_serial_port ${catkin_LIBRARIES})
add_dependencies(arduino_serial_port arduino_serial_port_generate_messages_cpp)
