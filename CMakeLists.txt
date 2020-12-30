cmake_minimum_required(VERSION 2.8.3)
project(actor_legs_extraction)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  obstacle_detector
  people_msgs
)

catkin_package(
	CATKIN_DEPENDS 
	gazebo_ros
	roscpp 
    obstacle_detector
    people_msgs
)

set(CMAKE_CXX_FLAGS   "-Wall -Wpedantic --std=c++14")
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(${PROJECT_NAME} 
	src/LegsExtraction.hpp 
	src/LegsExtraction.cpp
)

add_library(people_extraction 
	src/PeopleExtraction.hpp 
	src/PeopleExtraction.cpp
)

add_dependencies(${PROJECT_NAME} 
	${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME} 
	${catkin_LIBRARIES} 
	${GAZEBO_LIBRARIES} 
	${roscpp_LIBRARIES}
)

target_link_libraries(people_extraction
	${catkin_LIBRARIES} 
	${GAZEBO_LIBRARIES} 
	${roscpp_LIBRARIES}
)

add_executable(actor_legs_extraction_node src/actor_legs_extraction_node.cpp)
target_link_libraries(actor_legs_extraction_node 
	${PROJECT_NAME} 
	${catkin_LIBRARIES}
)

add_executable(people_extraction_node src/people_extraction_node.cpp)
target_link_libraries(people_extraction_node 
	people_extraction
	${catkin_LIBRARIES}
)