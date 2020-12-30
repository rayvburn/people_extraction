/*
 * Person.hpp
 *
 *  Created on: Dec 29, 2020
 *      Author: rayvburn
 */

#ifndef SRC_PERSON_HPP_
#define SRC_PERSON_HPP_

#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

struct Person {
	std::string name;
	std::string object_id;
	std::string frame_id;
	geometry_msgs::PoseStamped pose;
	geometry_msgs::TwistStamped vel;
	double reliability;
	double covariance[9];
	std::vector<std::string> tagnames;
	std::vector<std::string> tags;
	bool initialization;
};



#endif /* SRC_PERSON_HPP_ */
