/*
 * LegsExtraction.hpp
 *
 *  Created on: Dec 19, 2020
 *      Author: rayvburn
 */

#ifndef SRC_LEGSEXTRACTION_HPP_
#define SRC_LEGSEXTRACTION_HPP_

#include <ros/ros.h>
#include <obstacle_detector/Obstacles.h>
#include <people_msgs/PositionMeasurementArray.h>
#include <gazebo_msgs/ModelStates.h>

class LegsExtraction {
public:
	LegsExtraction();
	virtual ~LegsExtraction() = default;

private:
	struct Person {
		std::string name;
		geometry_msgs::Pose pose;
		geometry_msgs::Twist vel;
	};

	struct PersonAsCircleObstacle {
		/// index in the `people_data_` vector
		unsigned int person_id;
		std::vector<obstacle_detector::CircleObstacle> circles;
	};

	/**
	 * @brief
	 * @note This is a high frequency topic
	 * @param msg
	 */
	void gazeboModelStateCallback(const gazebo_msgs::ModelStates::ConstPtr &msg);

	/**
	 * @brief
	 * @note This is a low frequency topic, whose callback will be used to publish `people_msg`
	 * @param msg
	 */
	void obstacleDetectorCallback(const obstacle_detector::ObstaclesConstPtr &msg);

	/**
	 * @brief Computes Euclidean distance along XY plane
	 * @param from
	 * @param to
	 * @return
	 */
	double computeDistance(geometry_msgs::Point from, geometry_msgs::Point to) const;

	static constexpr auto DIST_LEGS_MAX_DEFAULT = 0.30;
	ros::NodeHandle nh_;

	ros::Subscriber sub_obstacle_;
	ros::Subscriber sub_gazebo_;
	ros::Publisher pub_obstacle_;
	ros::Publisher pub_people_;

	std::vector<Person> people_data_;
	double dist_legs_max_;
};

#endif /* SRC_LEGSEXTRACTION_HPP_ */
