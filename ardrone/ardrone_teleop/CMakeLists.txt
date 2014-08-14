# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(ardrone_teleop)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS roscpp)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

catkin_package(
    DEPENDS ncurses
    CATKIN_DEPENDS roscpp ardrone_driver
    INCLUDE_DIRS include
    LIBRARIES ncurses
)

#SET(CURSES_NEED_NCURSES TRUE)
#FIND_PACKAGE(Curses)
#if(CURSES_FOUND)
#	message("found Curses: " ${CURSES_INCLUDE_DIR})
#	include_directories(${CURSES_INCLUDE_DIR})
#endif(CURSES_FOUND)

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(keymote src/CursesKeymote.cpp ${CURSES_LIBRARIES})
target_link_libraries(keymote ncurses ${catkin_LIBRARIES})
add_dependencies(keymote ardrone_teleop_generate_messages_cpp ardrone_driver_generate_message_cpp)
