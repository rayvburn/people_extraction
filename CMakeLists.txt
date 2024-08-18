cmake_minimum_required(VERSION 3.5)
project(people_extraction)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  people_msgs
  people_msgs_utils
)

catkin_package(
	CATKIN_DEPENDS 
	gazebo_ros
	roscpp 
    people_msgs
	people_msgs_utils
)

set(CMAKE_CXX_FLAGS   "-Wall -Wpedantic --std=c++17")
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(people_extraction 
	src/PeopleExtraction.hpp 
	src/PeopleExtraction.cpp
	src/GazeboExtractor.hpp
	src/ActorLocalizationSubscriber.hpp
	src/HuberoExtractor.hpp
	src/HuberoExtractor.cpp
)

target_link_libraries(people_extraction
	${catkin_LIBRARIES} 
	${GAZEBO_LIBRARIES} 
	${roscpp_LIBRARIES}
)

add_executable(people_extraction_node src/people_extraction_node.cpp)
target_link_libraries(people_extraction_node 
	people_extraction
	${catkin_LIBRARIES}
)