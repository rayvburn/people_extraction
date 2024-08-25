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
#include <thread>

#include "GazeboExtractor.hpp"
#include "HuberoExtractor.hpp"

class PeopleExtraction {
public:
	PeopleExtraction();
	virtual ~PeopleExtraction() = default;

private:
	/**
	 * @brief Evaluates ROS parameters searching for human group definitions
	 * Prepares @ref member_groups_arrangement_ which is a map that eases filling the ROS messages later on.
	 * Specifically, additional computations are performed at the beginning to ease the access when publishing data.
	 */
	void discoverGroupsConfiguration();

	/// Searches through parameters to obtain a simple TF to be applied once reporting localization data
	std::map<std::string, std::vector<double>> discoverTransforms(
		const std::vector<std::string>& name_patterns
	) const;

	/**
	 * @brief Converts Gazebo ModelStates database to people representation in the @ref people_msgs_utils::People form
	 *
	 * @return people_msgs_utils::People
	 */
	std::map<std::string, people_msgs_utils::Person> gazeboModelsToPeople(
		const std::map<std::string, std::pair<size_t, gazebo_msgs::ModelState>>& people_models
	) const;

	std::map<std::string, people_msgs_utils::Person> huberoActorsToPeople(
		const std::vector<std::unique_ptr<ActorLocalizationSubscriber>>& actors
	) const;

	std::vector<std::pair<people_msgs_utils::Person, people_msgs_utils::Group>> createPeopleGroupAssociations(
		const std::map<std::string, people_msgs_utils::Person>& people
	);

	void publish();
	void publishPeople(const std::vector<std::pair<people_msgs_utils::Person, people_msgs_utils::Group>>& people_grouped);
	void publishPeoplePositions(const std::vector<std::pair<people_msgs_utils::Person, people_msgs_utils::Group>>& people_grouped);

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

	// Grouping map: Group ID to entities in that group
	std::map<size_t, std::vector<std::string>> groups_arrangement_;

	tf2_ros::Buffer tf_buffer_;
	tf2_ros::TransformListener tf_listener_;

	ros::Publisher pub_people_;
	ros::Publisher pub_pos_;

	std::string world_tf_frame_;
	std::string target_tf_frame_;

	ros::Timer timer_pub_;
	std::thread topics_waiter_;
};
