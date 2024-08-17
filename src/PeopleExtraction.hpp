/*
 * PeopleExtraction.hpp
 *
 *  Created on: Dec 29, 2020
 *      Author: rayvburn
 */

#pragma once

#include <ros/ros.h>

#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <people_msgs/People.h>
#include <people_msgs/PersonStamped.h>
#include <people_msgs/PositionMeasurementArray.h>
#include <people_msgs_utils/person.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <mutex>

class PeopleExtraction {
public:
	PeopleExtraction();
	virtual ~PeopleExtraction() = default;

private:
	/**
	 * @brief
	 * @note This is a high frequency topic
	 * @param msg
	 */
	void gazeboModelStateCallback(const gazebo_msgs::ModelStates::ConstPtr& msg);

	/**
	 * @brief Evaluates, whether the person exists in the internal database
	 * @param name: name of the person that is searched for
	 * @return
	 */
	bool doesPersonExist(const std::string& name) const;

	/**
	 * @brief Evaluates whether the given @ref model_name matches any pattern given by @ref patterns
	 *
	 * @param model_name
	 * @param patterns
	 * @return std::tuple<bool, std::string>
	 */
	std::tuple<bool, std::string> isMatching(const std::string& model_name, std::vector<std::string>& patterns) const;

	/**
	 * @brief Get rid of the unnecessary model
	 * @param model_names
	 */
	void deleteDestroyedPerson(const std::vector<std::string>& model_names);

	/**
	 * @brief Converts Gazebo ModelStates database to people representation in the @ref people_msgs_utils::People form
	 *
	 * @return people_msgs_utils::People
	 */
	people_msgs_utils::People gazeboModelsToPeople() const;

	void publish();
	void publishPeople(const people_msgs_utils::People& people);
	void publishPeoplePositions(const people_msgs_utils::People& people);

	ros::NodeHandle nh_;

	tf2_ros::Buffer tf_buffer_;
	tf2_ros::TransformListener tf_listener_;

	std::mutex mutex_;
	ros::Subscriber sub_gazebo_;
	ros::Publisher pub_people_;
	ros::Publisher pub_pos_;

	// key: name of the object in the simulation
	// value: pair with object's ID and the Gazebo data structure
	std::map<std::string, std::pair<size_t, gazebo_msgs::ModelState>> people_gazebo_;
	/// @brief Algorithm looks for a model name that meets one of the given patterns
	std::vector<std::string> people_name_patterns_;

	std::string world_tf_frame_;
	std::string target_tf_frame_;

	ros::Timer timer_pub_;
};
