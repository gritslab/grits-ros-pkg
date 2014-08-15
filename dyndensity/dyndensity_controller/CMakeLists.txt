cmake_minimum_required(VERSION 2.8.3)
project(dyndensity_controller)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  khepera3_driver
  message_generation
  roscpp
  sensor_msgs
  optitrack_driver
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
	CATKIN_DEPENDS geometry_msgs khepera3_driver message_runtime roscpp sensor_msgs optitrack_driver visualization_msgs
	DEPENDS Eigen3 Boost
)

include_directories(include ext ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${Boost_INCLUDE_DIR})

#add_executable(points_and_lines src/points_and_lines.cpp)
#add_executable(tracker src/tracker.cpp)
#add_executable(displayer src/displayer.cpp)
#add_executable(displayVoronoi src/displayVoronoi.cpp src/VoronoiDiagramGenerator.cpp src/graham/ConvexHull.cpp src/graham/GrahamScanConvexHull.cpp)
add_executable(CVTControl12 src/DynamicDensityControl.cpp ext/voronoi/VoronoiDiagramGenerator.cpp ext/graham/ConvexHull.cpp ext/graham/GrahamScanConvexHull.cpp ext/dunavant/dunavant.cpp src/CommunicationProtocol.cpp)

#add_dependencies(points_and_lines dyndensity_controller_generate_messages_cpp khepera3_driver_generate_messages_cpp optitrack_driver_generate_messages_cpp)
#add_dependencies(displayer dyndensity_controller_generate_messages_cpp khepera3_driver_generate_messages_cpp optitrack_driver_generate_messages_cpp)
#add_dependencies(tracker dyndensity_controller_generate_messages_cpp khepera3_driver_generate_messages_cpp optitrack_driver_generate_messages_cpp)
#add_dependencies(displayVoronoi dyndensity_controller_generate_messages_cpp khepera3_driver_generate_messages_cpp optitrack_driver_generate_messages_cpp)
add_dependencies(CVTControl12 dyndensity_controller_generate_messages_cpp khepera3_driver_generate_messages_cpp optitrack_driver_generate_messages_cpp geometry_msgs_generate_messages_cpp)

#target_link_libraries(points_and_lines ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_LIBRARIES})
#target_link_libraries(displayer ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_LIBRARIES})
#target_link_libraries(tracker ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_LIBRARIES})
#target_link_libraries(displayVoronoi ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_LIBRARIES})
target_link_libraries(CVTControl12 ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_LIBRARIES})
