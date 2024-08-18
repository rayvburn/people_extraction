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
#include <gazebo_msgs/LinkState.h>
#include <gazebo_msgs/LinkStates.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <people_msgs/People.h>
#include <people_msgs/PersonStamped.h>
#include <people_msgs/PositionMeasurementArray.h>
#include <people_msgs_utils/person.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <mutex>

#include "GazeboExtractor.hpp"
#include "HuberoExtractor.hpp"

class PeopleExtraction {
public:
	PeopleExtraction();
	virtual ~PeopleExtraction() = default;

private:
	/// Searches through parameters to obtain a simple TF to be applied once reporting localization data
	std::map<std::string, std::vector<double>> discoverTransforms(
		const std::vector<std::string>& name_patterns
	) const;

	/**
	 * @brief Converts Gazebo ModelStates database to people representation in the @ref people_msgs_utils::People form
	 *
	 * @return people_msgs_utils::People
	 */
	people_msgs_utils::People gazeboModelsToPeople(
		const std::map<std::string, std::pair<size_t, gazebo_msgs::ModelState>>& people_models
	) const;

	people_msgs_utils::People huberoActorsToPeople(
		const std::vector<std::unique_ptr<ActorLocalizationSubscriber>>& actors
	) const;

	void publish();
	void publishPeople(const people_msgs_utils::People& people);
	void publishPeoplePositions(const people_msgs_utils::People& people);

	ros::NodeHandle nh_;

	typedef GazeboExtractor<gazebo_msgs::ModelStates::ConstPtr, gazebo_msgs::ModelStates, gazebo_msgs::ModelState> GazeboModelExtractor;
	typedef GazeboExtractor<gazebo_msgs::LinkStates::ConstPtr, gazebo_msgs::LinkStates, gazebo_msgs::ModelState> GazeboLinkExtractor;
	// NOTE: unfortunately, ROS Message Filters do not work with gazebo_msgs-based topics as those messages
	// do not carry timestamp values
	std::unique_ptr<GazeboModelExtractor> model_extractor_;
	std::unique_ptr<GazeboLinkExtractor> link_extractor_;
	// HuBeRo framework-specific extractor
	std::unique_ptr<HuberoExtractor> hubero_extractor_;
	std::atomic<size_t> id_ref_;

	tf2_ros::Buffer tf_buffer_;
	tf2_ros::TransformListener tf_listener_;

	ros::Publisher pub_people_;
	ros::Publisher pub_pos_;

	std::string world_tf_frame_;
	std::string target_tf_frame_;

	ros::Timer timer_pub_;
};
