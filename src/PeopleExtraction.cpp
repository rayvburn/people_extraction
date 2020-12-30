/*
 * PeopleExtraction.cpp
 *
 *  Created on: Dec 29, 2020
 *      Author: rayvburn
 */

#include "PeopleExtraction.hpp"

constexpr char* PeopleExtraction::GAZEBO_FRAME_ID_DEFAULT;

PeopleExtraction::PeopleExtraction() :
	tf_listener_(tf_buffer_),
	id_next_(0),
	gazebo_tf_frame_(PeopleExtraction::GAZEBO_FRAME_ID_DEFAULT),
	callback_counter_(0),
	callback_omits_(0),
	seq_(0)
{
	std::string people_topic("");
	std::string position_topic("");
	nh_.param<std::string>("gazebo_tf_frame", gazebo_tf_frame_, std::string(PeopleExtraction::GAZEBO_FRAME_ID_DEFAULT));
	nh_.param<int>("callback_omits", callback_omits_, callback_omits_);
	nh_.param<std::string>("people_topic", people_topic, "/people_topic");
	nh_.param<std::string>("position_topic", position_topic, "/position_topic");
	nh_.getParam("model_name_patterns", people_name_patterns_);
	if (!people_name_patterns_.size()) {
		ROS_ERROR("PeopleExtraction - model_name_patterns is empty! Class would be ill-formed");
		return;
	}
	sub_gazebo_ = nh_.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states",  1, &PeopleExtraction::gazeboModelStateCallback, this);;
	pub_people_ = nh_.advertise<people_msgs::People>(people_topic, 5);
	pub_pos_ = nh_.advertise<people_msgs::PositionMeasurementArray>(position_topic, 5);
}

void PeopleExtraction::gazeboModelStateCallback(const gazebo_msgs::ModelStates::ConstPtr &msg) {
	if (callback_counter_++ != callback_omits_) {
		return;
	}
	std::lock_guard<std::mutex> lock(mutex_);
	callback_counter_ = 0;

	// update internal database dynamically
	for (auto pattern : people_name_patterns_) {
		// evaluate, whether the object named with a given pattern exists in a simulation
		std::vector<std::string> sim_names = findModels(msg->name, pattern);
		// iterate over actual names
		for (auto sim_name : sim_names) {
			// check whether the person exists in the internal database
			if (!doesPersonExist(sim_name)) {
				Person bud;
				bud.name = sim_name;
				people_data_.push_back(bud);
				people_name_id_[sim_name] = id_next_++;
				ROS_INFO("Gazebo Callback - Added new person '%s' (ID: %d), detected from naming pattern: '%s'", sim_name.c_str(), people_name_id_[sim_name], pattern.c_str());
			}
		}
	}

	// TODO: possibly delete renamed objects

	// compute indexes of the `people_names` objects
	for (auto&& person : people_data_) {
		auto it = std::find(msg->name.begin(), msg->name.end(), person.name);
		if (it != msg->name.end()) {
			// found, compute index
			int index = it - msg->name.begin();
			// retrieve meaningful data from corresponding vectors
			if (index >= 0) {
				person.pose.header.frame_id = gazebo_tf_frame_;
				person.pose.header.seq++;
				person.pose.header.stamp = ros::Time::now();

				person.pose.pose.position = msg->pose.at(index).position;
				person.pose.pose.orientation = msg->pose.at(index).orientation;

				// copy header
				person.vel.header = person.pose.header;

				person.vel.twist.linear = msg->twist.at(index).linear;
				person.vel.twist.angular = msg->twist.at(index).angular;
			}
		}
	}

	// publishing section
	publishPeople();
	publishPeoplePositions();
}

bool PeopleExtraction::doesPersonExist(const std::string &name) const {
	for (auto person : people_data_) {
		if (person.name == name) {
			return true;
		}
	}
	return false;
}

std::vector<std::string> PeopleExtraction::findModels(const std::vector<std::string> model_names, std::string pattern) const {
	std::vector<std::string> db;
	for (auto name : model_names) {
		// find
		if (name.find(pattern) != std::string::npos) {
			// found
			db.push_back(name);
		}
	}
	return db;
}

void PeopleExtraction::publishPeople() {
	people_msgs::People people_msg;
	people_msgs::Person person_msg;

	people_msg.header.frame_id = gazebo_tf_frame_;
	people_msg.header.seq = seq_;
	people_msg.header.stamp = ros::Time::now();
	for (auto person : people_data_) {
		// http://docs.ros.org/en/kinetic/api/people_msgs/html/msg/Person.html
		person_msg.name = person.name;
		person_msg.position = person.pose.pose.position;
		person_msg.velocity.x = person.vel.twist.linear.x;
		person_msg.velocity.y = person.vel.twist.linear.y;
		person_msg.velocity.z = person.vel.twist.linear.z;
		person_msg.reliability = 1.0;
		people_msg.people.push_back(person_msg);
	}
	pub_people_.publish(people_msg);
}

void PeopleExtraction::publishPeoplePositions() {
	people_msgs::PositionMeasurementArray position_measurement_array_msg;
	people_msgs::PositionMeasurement position_measurement_msg;

	position_measurement_array_msg.header.frame_id = gazebo_tf_frame_;
	position_measurement_array_msg.header.seq = seq_;
	position_measurement_array_msg.header.stamp = ros::Time::now();

	position_measurement_msg.header = position_measurement_array_msg.header;

	for (auto person : people_data_) {
		// http://docs.ros.org/en/kinetic/api/people_msgs/html/msg/PositionMeasurement.html
		position_measurement_msg.name = person.name;
		position_measurement_msg.object_id = person.object_id;
		position_measurement_msg.pos.x = person.pose.pose.position.x;
		position_measurement_msg.pos.y = person.pose.pose.position.y;
		position_measurement_msg.pos.z = person.pose.pose.position.z;
		position_measurement_msg.reliability = 1.0;
		// measurements are ideal, so covariance = 0
		// FIXME: initialization? what is that for?
		position_measurement_msg.initialization = 0;
		position_measurement_array_msg.people.push_back(position_measurement_msg);
	}
	pub_pos_.publish(position_measurement_array_msg);
}
