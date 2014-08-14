cmake_minimum_required(VERSION 2.8.3)
project(dyndensity_controller)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  khepera3_driver
  message_generation
  optitrack_driver
  roscpp
  sensor_msgs
  vicon_driver
  visualization_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

add_message_files(FILES
	displayerData.msg
)

generate_messages(DEPENDENCIES geometry_msgs)

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS geometry_msgs khepera3_driver message_runtime optitrack_driver roscpp sensor_msgs vicon_driver visualization_msgs
	DEPENDS eigen
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${BOOST_INCLUDE_DIRS})

add_executable(points_and_lines src/points_and_lines.cpp)
add_executable(tracker src/tracker.cpp)
add_executable(displayer src/displayer.cpp)
add_executable(displayVoronoi src/displayVoronoi.cpp src/VoronoiDiagramGenerator.cpp src/graham/ConvexHull.cpp src/graham/GrahamScanConvexHull.cpp)
add_executable(CVTControl12 src/CVTControl12.cpp src/VoronoiDiagramGenerator.cpp src/graham/ConvexHull.cpp src/graham/GrahamScanConvexHull.cpp src/dunavant/dunavant.cpp src/CommunicationProtocol.cpp)

add_dependencies(points_and_lines dyndensity_controller_generate_messages_cpp khepera3_driver_generate_messages_cpp vicon_driver_generate_messages_cpp)
add_dependencies(displayer dyndensity_controller_generate_messages_cpp khepera3_driver_generate_messages_cpp vicon_driver_generate_messages_cpp)
add_dependencies(tracker dyndensity_controller_generate_messages_cpp khepera3_driver_generate_messages_cpp vicon_driver_generate_messages_cpp)
add_dependencies(displayVoronoi dyndensity_controller_generate_messages_cpp khepera3_driver_generate_messages_cpp vicon_driver_generate_messages_cpp)
add_dependencies(CVTControl12 dyndensity_controller_generate_messages_cpp khepera3_driver_generate_messages_cpp vicon_driver_generate_messages_cpp)

target_link_libraries(points_and_lines ${catkin_LIBRARIES} ${BOOST_LIBRARIES})
target_link_libraries(displayer ${catkin_LIBRARIES} ${BOOST_LIBRARIES})
target_link_libraries(tracker ${catkin_LIBRARIES} ${BOOST_LIBRARIES})
target_link_libraries(displayVoronoi ${catkin_LIBRARIES} ${BOOST_LIBRARIES})
target_link_libraries(CVTControl12 ${catkin_LIBRARIES} ${BOOST_LIBRARIES})
