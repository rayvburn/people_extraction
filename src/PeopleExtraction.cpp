/*
 * PeopleExtraction.cpp
 *
 *  Created on: Dec 29, 2020
 *      Author: rayvburn
 */

#include "PeopleExtraction.hpp"

#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <thread>

PeopleExtraction::PeopleExtraction():
	tf_listener_(tf_buffer_)
{
	auto pub_frequency = nh_.param<double>("pub_frequency", 30.0);
	nh_.param<std::string>("world_frame", world_tf_frame_, "world");
	nh_.param<std::string>("target_frame", target_tf_frame_, "odom");

	// model name check
	nh_.getParam("model_name_patterns", people_name_patterns_);
	if (!people_name_patterns_.size()) {
		ROS_ERROR("PeopleExtraction - model_name_patterns is empty! Class would be ill-formed");
		return;
	}

	// publishing frequency check
	if (pub_frequency <= 0.0) {
		ROS_ERROR(
			"Simulator-based people data will not be published as wrong frequency was selected: %f Hz",
			pub_frequency
		);
		return;
	}

	// create ROS interfaces
	pub_people_ = nh_.advertise<people_msgs::People>("/people", 5);
	pub_pos_ = nh_.advertise<people_msgs::PositionMeasurementArray>("/people_measurements", 5);
	sub_gazebo_ = nh_.subscribe<gazebo_msgs::ModelStates>(
		"/gazebo/model_states",
		1,
		&PeopleExtraction::gazeboModelStateCallback,
		this
	);

	timer_pub_ = nh_.createTimer(
		ros::Duration(1.0 / pub_frequency),
		std::bind(
			&PeopleExtraction::publish,
			this
		)
	);
}

void PeopleExtraction::gazeboModelStateCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
	if (msg->name.size() != msg->pose.size() || msg->name.size() != msg->twist.size()) {
		ROS_ERROR("Vector sizes in the input message are not equal, cannot process further");
		return;
	}
	std::lock_guard<std::mutex> lock(mutex_);

	// update internal database dynamically
	size_t models_found = 0;
	size_t models_total = msg->name.size();
	for (size_t i = 0; i < models_total; i++) {
		// investigated object - obtain only the name at this stage
		const auto& name = msg->name.at(i);
		// check whether a model with a given name is of our interest (according to the naming patterns)
		bool pattern_matched = false;
		std::string pattern;
		std::tie(pattern_matched, pattern) = isMatching(name, people_name_patterns_);
		if (!pattern_matched) {
			continue;
		}

		models_found++;
		gazebo_msgs::ModelState model;
		model.model_name = name;
		model.pose = msg->pose.at(i);
		model.twist = msg->twist.at(i);

		// check whether the person already exists in the internal database (only need ModelState update then)
		if (doesPersonExist(name)) {
			people_gazebo_[name].second = model;
			continue;
		}
		// new model - assign an ID and update its ModelState
		people_gazebo_[name] = {people_gazebo_.size(), model};
		ROS_INFO(
			"Tracking a new person '%s' (ID: '%lu'), detected from naming pattern: '%s'",
			people_gazebo_[name].second.model_name.c_str(),
			people_gazebo_[name].first,
			pattern.c_str()
		);
	}

	// possibly delete renamed/destroyed objects
	if (models_found < people_gazebo_.size()) {
		ROS_INFO(
			"Seems that an object was deleted. Number of matching model names found: %lu, size of database: %lu",
			models_found,
			people_gazebo_.size()
		);
		deleteDestroyedPerson(msg->name);
	}
}

bool PeopleExtraction::doesPersonExist(const std::string& name) const {
	return people_gazebo_.find(name) != people_gazebo_.cend();
}

std::tuple<bool, std::string> PeopleExtraction::isMatching(const std::string& model_name, std::vector<std::string>& patterns) const {
	for (const auto& pattern: patterns) {
		// try to find
		if (model_name.find(pattern) == std::string::npos) {
			continue;
		}
		return {true, pattern};
	}
	return {false, std::string()};
}

void PeopleExtraction::deleteDestroyedPerson(const std::vector<std::string>& model_names) {
	auto i = std::begin(people_gazebo_);
	// while is safer than for here
	while (i != std::end(people_gazebo_)) {
		// map and nested pair
		auto model_name = i->second.second.model_name;
		auto it = std::find(model_names.begin(), model_names.end(), model_name);
		if (it != model_names.end()) {
			// found
			i++;
			continue;
		}
		// delete current element
		ROS_INFO("Deleting '%s' from database", model_name.c_str());
		// erase from map; ref: https://stackoverflow.com/a/2874533
		i = people_gazebo_.erase(i);
	}
}

people_msgs_utils::People PeopleExtraction::gazeboModelsToPeople() const {
	// transform localization data to the desired frame
	geometry_msgs::TransformStamped tf_stamped;
	// by default - lack of translation and rotation
	tf_stamped.transform.rotation.w = 1.0;

	// obtain the transform to the desired frame
	try {
		tf_stamped = tf_buffer_.lookupTransform(
			target_tf_frame_,
			world_tf_frame_,
			ros::Time(0)
		);
	} catch (tf2::TransformException& ex) {
		ROS_ERROR(
			"Could not find a transform from the `%s` to `%s` frame! Exception details: `%s`",
			world_tf_frame_.c_str(),
			target_tf_frame_.c_str(),
			ex.what()
		);
		// avoid flooding console with this kind of errors (localization might not be operational yet)
		std::this_thread::sleep_for(std::chrono::seconds(1));
		return people_msgs_utils::People();
	}

	// prepare output container
	people_msgs_utils::People people;

	for (auto const& model_data: people_gazebo_) {
		// retrieve from the value (key is not considered here)
		auto id = model_data.second.first;
		auto model = model_data.second.second;

		geometry_msgs::PoseWithCovariance pose;
		pose.pose = model.pose;
		pose.covariance.fill(0.0); // assuming ideal data
		// convert data from twist to pose representation (contents are the same)
		geometry_msgs::PoseWithCovariance velocity;
		velocity.covariance.fill(0.0); // assuming ideal data
		velocity.pose.position.x = model.twist.linear.x;
		velocity.pose.position.y = model.twist.linear.y;
		velocity.pose.position.z = model.twist.linear.z;
		tf2::Quaternion angular_vel;
		// NOTE: for a proper yaw angle, yaw, pitch and roll angles must be reordered (compared to the documentation)
		angular_vel.setEuler(
			model.twist.angular.x,
			model.twist.angular.y,
			model.twist.angular.z
		);
		velocity.pose.orientation = tf2::toMsg(angular_vel);

		// create a Person instance (with geom. data in the source frame)
		people_msgs_utils::Person p(
			std::to_string(id), // avoid using 'string'-type names here
			pose,
			velocity,
			1.0, // complete reliability
			false,
			true,
			id,
			ros::Time::now().toNSec(), // track age = since startup as we have ideal data
			std::string("") // group data not included
		);
		// TODO: include group data (obtain from node parameters)

		// transform to the target frame
		p.transform(tf_stamped);
		// collect
		people.push_back(p);
	}

	return people;
}

void PeopleExtraction::publish() {
	std::lock_guard<std::mutex> lock(mutex_);

	// check if there is something to publish
	if (people_gazebo_.empty()) {
		return;
	}

	// Gazebo models to people representation
	auto people = gazeboModelsToPeople();

	publishPeople(people);
	publishPeoplePositions(people);
}

void PeopleExtraction::publishPeople(const people_msgs_utils::People& people) {
	people_msgs::People people_msg;

	people_msg.header.frame_id = target_tf_frame_;
	people_msg.header.stamp = ros::Time::now();
	for (const auto& person: people) {
		people_msg.people.push_back(person.toPersonStd());
	}
	pub_people_.publish(people_msg);
}

void PeopleExtraction::publishPeoplePositions(const people_msgs_utils::People& people) {
	people_msgs::PositionMeasurementArray pos_msg_array;

	pos_msg_array.header.frame_id = target_tf_frame_;
	pos_msg_array.header.stamp = ros::Time::now();

	for (auto person: people) {
		people_msgs::PositionMeasurement pos_msg;
		pos_msg.header = pos_msg_array.header;

		pos_msg.name = person.getName();
		pos_msg.object_id = person.getName();
		pos_msg.pos.x = person.getPositionX();
		pos_msg.pos.y = person.getPositionY();
		pos_msg.pos.z = person.getPositionZ();
		pos_msg.reliability = person.getReliability();
		// measurements are ideal, so covariance = 0
		// FIXME: initialization? what is that for?
		pos_msg.initialization = 0;
		pos_msg_array.people.push_back(pos_msg);
	}
	pub_pos_.publish(pos_msg_array);
}
