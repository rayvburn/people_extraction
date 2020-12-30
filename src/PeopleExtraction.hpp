/*
 * PeopleExtraction.hpp
 *
 *  Created on: Dec 29, 2020
 *      Author: rayvburn
 */

#ifndef SRC_PEOPLEEXTRACTION_HPP_
#define SRC_PEOPLEEXTRACTION_HPP_

#include <ros/ros.h>
#include <people_msgs/People.h>
#include <people_msgs/PersonStamped.h>
#include <people_msgs/PositionMeasurementArray.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <mutex>

#include "Person.hpp"

class PeopleExtraction {
public:
	PeopleExtraction();
	virtual ~PeopleExtraction() = default;

private:
	/**
	 * @brief
	 * @note This is a high frequency topic
	 * @note TODO: message_filters, TimeSynchronizer?
	 * @param msg
	 */
	void gazeboModelStateCallback(const gazebo_msgs::ModelStates::ConstPtr &msg);

	/**
	 * @brief Evaluates, whether the person exists in the internal database
	 * @param name: name of the person that is searched for
	 * @return
	 */
	bool doesPersonExist(const std::string &name) const;

	/**
	 * @brief Finds models that meet the given pattern
	 * @param model_names
	 * @param pattern
	 * @return
	 */
	std::vector<std::string> findModels(const std::vector<std::string> model_names, std::string pattern) const;

	void publishPeople();
	void publishPeoplePositions();

	static constexpr char* GAZEBO_FRAME_ID_DEFAULT = "world";
	ros::NodeHandle nh_;

	tf2_ros::Buffer tf_buffer_;
	tf2_ros::TransformListener tf_listener_;

	std::mutex mutex_;
	ros::Subscriber sub_gazebo_;
	ros::Publisher pub_people_;
	ros::Publisher pub_pos_;

	std::vector<Person> people_data_;
	/// @brief Algorithms looks for a model name that meets one of the given patterns
	std::vector<std::string> people_name_patterns_;
	/// @brief Stores name and ID correspondence of detected people
	std::map<std::string, int> people_name_id_;

	int id_next_;
	std::string gazebo_tf_frame_;
	int callback_counter_;
	int callback_omits_;
	unsigned long int seq_;

	// possibly extend package features to:
	// filtering detected obstacles - republishing ones indicating not-legs
	friend class LegsExtraction;
};

#endif /* SRC_PEOPLEEXTRACTION_HPP_ */
