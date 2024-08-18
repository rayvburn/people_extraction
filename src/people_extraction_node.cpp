/*
 * people_extraction.cpp
 *
 *  Created on: Dec 19, 2020
 *      Author: rayvburn
 */

// --------------------------------------------------------------

#include <ros/ros.h>
#include "PeopleExtraction.hpp"

int main(int argc, char** argv) {
	ros::init(argc, argv, "people_extraction");
	PeopleExtraction extraction;
	ros::spin();
	return 0;
}
