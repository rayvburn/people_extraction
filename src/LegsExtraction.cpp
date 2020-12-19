/*
 * LegsExtraction.cpp
 *
 *  Created on: Dec 19, 2020
 *      Author: rayvburn
 */

#include "LegsExtraction.hpp"
#include <algorithm>
#include <math.h>

bool sortbysec(const std::tuple<unsigned int, double>& a, const std::tuple<unsigned int, double>& b);
constexpr double LegsExtraction::DIST_LEGS_MAX_DEFAULT;

LegsExtraction::LegsExtraction():
	dist_legs_max_(LegsExtraction::DIST_LEGS_MAX_DEFAULT)
{
	std::string obstacle_topic("");
	std::string obstacle_no_people_topic("");
	std::string people_pos_topic("");
	std::vector<std::string> people_names;
	nh_.param<std::string>("legs_extraction/obstacle_topic", obstacle_topic, "/raw_obstacles");
	nh_.param<std::string>("legs_extraction/obstacle_no_people_topic", obstacle_no_people_topic, "/raw_obstacles_no_people");
	nh_.param<std::string>("legs_extraction/people_pos_topic", people_pos_topic, "/people_tracker_measurements");
	nh_.getParam("legs_extraction/model_names", people_names);
	nh_.param<double>("legs_extraction/dist_legs_max", dist_legs_max_, LegsExtraction::DIST_LEGS_MAX_DEFAULT);

	sub_gazebo_ = nh_.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states",  1, &LegsExtraction::gazeboModelStateCallback, this);;
	sub_obstacle_ = nh_.subscribe<obstacle_detector::Obstacles>(obstacle_topic,  3, &LegsExtraction::obstacleDetectorCallback, this);;
	pub_obstacle_ = nh_.advertise<obstacle_detector::Obstacles>(obstacle_no_people_topic, 5);
	pub_people_ = nh_.advertise<people_msgs::PositionMeasurementArray>(people_pos_topic, 5);

	for (unsigned int i = 0; i < people_names.size(); i++) {
		people_data_.push_back(Person());
		people_data_.at(i).name = people_names.at(i);
	}
}

void LegsExtraction::gazeboModelStateCallback(const gazebo_msgs::ModelStates::ConstPtr &msg) {
	// compute indexes of the `people_names` objects
	std::vector<unsigned int> indices;
	for (unsigned int i = 0; i < people_data_.size(); i++) {
		auto it = std::find(msg->name.begin(), msg->name.end(), people_data_.at(i).name);
		if (it != msg->name.end()) {
			// found, compute index
			int index = it - msg->name.begin();
			// retrieve meaningful data from corresponding vectors
			if (index >= 0) {
				people_data_.at(i).pose = msg->pose.at(index);
				people_data_.at(i).vel = msg->twist.at(index);
			}
		}
	}
}

void LegsExtraction::obstacleDetectorCallback(const obstacle_detector::ObstaclesConstPtr &msg) {
	obstacle_detector::Obstacles obstacles_filtered;
	obstacles_filtered.header = msg->header;
	obstacles_filtered.segments = msg->segments;

	unsigned int debug_quantity_start = msg->circles.size();

	// circles that are too far from any person should be added to the `filtered`
	// message, whereas those indicating people's legs should be published as `people_msgs`)
	std::vector<PersonAsCircleObstacle> people_circles;
	for (unsigned int i = 0; i < people_data_.size(); i++) {
		people_circles.push_back(PersonAsCircleObstacle());
	}

	// store indexes of circles that were already associated with legs
	std::vector<unsigned int> circles_associated;

	// association
	for (unsigned int i = 0; i < people_data_.size(); i++) {
		for (unsigned int j = 0; j < msg->circles.size(); j++) {
			double distance = 0.0;
			if ((distance = computeDistance(msg->circles.at(j).center, people_data_.at(i).pose.position)) <= dist_legs_max_) {
				people_circles.at(i).circles.push_back(msg->circles.at(j));
				circles_associated.push_back(j);
			}
			printf("person %d - circle %d\tdistance %2.3f / person: x=%2.3f, y=%2.3f \t| circle: x=%2.3f, y=%2.3f\t",
				i, j, distance, people_data_.at(i).pose.position.x, people_data_.at(i).pose.position.y,
				msg->circles.at(j).center.x, msg->circles.at(j).center.y);
			if (distance <= dist_legs_max_) {
				printf("1");
			}
			printf("\r\n");
		}
	}

	// copy unassociated circles to the message
	for (unsigned int i = 0; i < msg->circles.size(); i++) {
		auto it = std::find(circles_associated.begin(), circles_associated.end(), i);
		// index not found in `associated`
		if (it == circles_associated.end()) {
			obstacles_filtered.circles.push_back(msg->circles.at(i));
		}
	}

	// ignore objects that unlikely represent people
	for (unsigned int i = 0; i < people_circles.size(); i++) { // iterate over people
		if (people_circles.at(i).circles.size() > 2) {
			ROS_INFO("found more than 2 circles representing person");
			std::vector<std::tuple<unsigned int, double>> dists;
			for (unsigned int j = 0; j < people_circles.at(i).circles.size(); j++) { // iterate over circles associated to the person
				std::tuple<unsigned int, double> tup;
				std::get<0>(tup) = j;
				std::get<1>(tup) =
					computeDistance(people_circles.at(i).circles.at(j).center, people_data_.at(i).pose.position);
				dists.push_back(tup);
			}

			// choose 2 closest circles
			sort(dists.begin(), dists.end(), sortbysec);
			// copy circles for safe overwrite
			auto people_circles_cp(people_circles);
			// delete previously associated circles (totally)
			people_circles.at(i).circles.clear();

			for (unsigned int j = 0; j < people_circles.at(i).circles.size(); j++) {
				unsigned int circle_sort_index =  std::get<0>(dists.at(j));
				obstacle_detector::CircleObstacle circle = people_circles_cp.at(i).circles.at(circle_sort_index);
				if (j < 2) {
					// update association of a circle to a person
					people_circles.at(i).circles.push_back(circle);
				} else {
					// circles which should be ignored from association - add them to `filtered obstacles`
					obstacles_filtered.circles.push_back(circle);
				}
			}
		}
	}

	// debug
	unsigned int ppl_cir = 0;
	for (unsigned int i = 0; i < people_circles.size(); i++) {
		ppl_cir += people_circles.at(i).circles.size();
	}
	unsigned int obs_cir = obstacles_filtered.circles.size();
	unsigned int debug_quantity_end = ppl_cir + obs_cir;
	printf("bilans start: %d | end: %d\t(ppl: %d, obs: %d)\r\n",
		debug_quantity_start, debug_quantity_end, ppl_cir, obs_cir);

	people_msgs::PositionMeasurementArray people_pos;
	// TODO

	// prepare `legs` for publishing
	pub_people_.publish(people_msgs::PositionMeasurementArray());
	pub_obstacle_.publish(obstacles_filtered);

}

double LegsExtraction::computeDistance(geometry_msgs::Point from, geometry_msgs::Point to) const {
	return sqrt(std::pow(from.x - to.x, 2) + std::pow(from.y - to.y, 2));
}

// Comparison function to sort the vector elements by second element of tuples
// @href https://www.geeksforgeeks.org/sorting-vector-tuple-c-ascending-order/
bool sortbysec(const std::tuple<unsigned int, double>& a, const std::tuple<unsigned int, double>& b) {
    return (std::get<1>(a) < std::get<1>(b));
}
