/*
 * PeopleExtraction.cpp
 *
 *  Created on: Dec 29, 2020
 *      Author: rayvburn
 */

#include "PeopleExtraction.hpp"

#define DEBUG_TF

PeopleExtraction::PeopleExtraction() :
	tf_listener_(tf_buffer_),
	id_next_(0),
	gazebo_tf_frame_(PeopleExtraction::GAZEBO_FRAME_ID_DEFAULT),
	target_tf_frame_(PeopleExtraction::TARGET_FRAME_ID_DEFAULT),
	callback_counter_(0),
	callback_omits_(0),
	seq_(0)
{
	std::string people_topic("");
	std::string position_topic("");
	nh_.param<std::string>("gazebo_frame", gazebo_tf_frame_, gazebo_tf_frame_);
	nh_.param<std::string>("target_frame", target_tf_frame_, target_tf_frame_);
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
	int models_found = 0;
	for (auto pattern : people_name_patterns_) {
		// evaluate, whether the object named with a given pattern exists in a simulation
		std::vector<std::string> sim_names = findModels(msg->name, pattern);
		models_found += sim_names.size();
		// iterate over actual names
		for (auto sim_name : sim_names) {
			// check whether the person already exists in the internal database
			if (!doesPersonExist(sim_name)) {
				Person bud;
				bud.name = sim_name;
				people_data_.push_back(bud);
				people_name_id_[sim_name] = id_next_++;
				ROS_INFO("Gazebo Callback - Added new person '%s' (ID: %d), detected from naming pattern: '%s'", sim_name.c_str(), people_name_id_[sim_name], pattern.c_str());
			}
		}
	}
	// possibly delete renamed/destroyed objects
	if (models_found < people_data_.size()) {
		ROS_INFO("Seems that an object was deleted. Number of matching model names found: %d, size of database: %d",
			models_found, people_data_.size());
		deleteDestroyedPerson(msg->name);
	}

	// compute indexes of the `people_names` objects
	for (auto&& person : people_data_) {
		auto it = std::find(msg->name.begin(), msg->name.end(), person.name);
		if (it != msg->name.end()) {
			// found, compute index
			int index = it - msg->name.begin();
			// retrieve meaningful data from corresponding vectors
			if (index >= 0) {
				// fill up raw (Gazebo) values at first
				person.pose.header.frame_id = gazebo_tf_frame_;
				person.pose.header.seq++;
				person.pose.header.stamp = ros::Time::now();

				person.pose.pose.position = msg->pose.at(index).position;
				person.pose.pose.orientation = msg->pose.at(index).orientation;

				// copy header
				person.vel.header = person.pose.header;

				person.vel.twist.linear = msg->twist.at(index).linear;
				person.vel.twist.angular = msg->twist.at(index).angular;
				// try to transform to another coordinate system
				try {
					geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
						target_tf_frame_,
						gazebo_tf_frame_,
						ros::Time::now(),
						ros::Duration(1.0)
					);
					// transform pose
					geometry_msgs::PoseStamped pose_backup = person.pose;
					tf2::doTransform(person.pose, person.pose, transform);
					#ifdef DEBUG_TF
					printf("[transform pose (1)] before - position: %2.5f, %2.5f, %2.5f, orientation: %2.5f, %2.5f, %2.5f\r\n",
						pose_backup.pose.position.x,
						pose_backup.pose.position.y,
						pose_backup.pose.position.z,
						pose_backup.pose.orientation.x,
						pose_backup.pose.orientation.y,
						pose_backup.pose.orientation.z
					);
					printf("[transform pose (2)] after  - position: %2.5f, %2.5f, %2.5f, orientation: %2.5f, %2.5f, %2.5f\r\n",
						person.pose.pose.position.x,
						person.pose.pose.position.y,
						person.pose.pose.position.z,
						person.pose.pose.orientation.x,
						person.pose.pose.orientation.y,
						person.pose.pose.orientation.z
					);
					#endif
					// rotate velocity (in the local coordinate system)
					rotateTwist(person.vel, transform);
					// update header frame
					person.pose.header.frame_id = target_tf_frame_;
					person.vel.header.frame_id = target_tf_frame_;
					#ifdef DEBUG_TF
					printf("Assuming successful transformation - changed frame_id to TARGET: %s\r\n",
						person.pose.header.frame_id.c_str());
					#endif
				} catch (tf2::TransformException &e) {
					ROS_WARN("exception: %s\r\nPerson %s: could not find TF from %s to %s",
						e.what(), msg->name.at(index).c_str(), gazebo_tf_frame_.c_str(), target_tf_frame_.c_str());
				}
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

std::vector<std::string> PeopleExtraction::findModels(const std::vector<std::string> &model_names, std::string pattern) const {
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

void PeopleExtraction::deleteDestroyedPerson(const std::vector<std::string> &model_names) {
	int num_deleted = 0;
	auto i = std::begin(people_data_);
	// while is safer than for here
	while (i != std::end(people_data_)) {
		auto it = std::find(model_names.begin(), model_names.end(), i->name);
		if (it != model_names.end()) {
			// found
			i++;
			continue;
		}
		// delete current element
		ROS_INFO("Deleting '%s' from database", i->name.c_str());
		// erase from map
		num_deleted += people_name_id_.erase(i->name);
		people_data_.erase(i);
	}
	ROS_INFO("Deleted %d object(s)", num_deleted);
}

void PeopleExtraction::rotateTwist(geometry_msgs::TwistStamped &vel, const geometry_msgs::TransformStamped &transform) {
	// local coordinate system - consider rotation only
	geometry_msgs::TransformStamped transform_local = transform;
	transform_local.transform.translation.x = 0.0;
	transform_local.transform.translation.y = 0.0;
	transform_local.transform.translation.z = 0.0;

	// use msg type supported by tf2::doTransform
	// http://docs.ros.org/en/kinetic/api/tf2_geometry_msgs/html/c++/namespacetf2.html#a82ca47c6f5b0360e6c5b250dca719a78
	geometry_msgs::Vector3Stamped vel_v3;
	vel_v3.header = vel.header;
	vel_v3.vector.x = vel.twist.linear.x;
	vel_v3.vector.y = vel.twist.linear.y;
	vel_v3.vector.z = vel.twist.linear.z;
	#ifdef DEBUG_TF
	printf("[rotateTwist (1)] transform - raw tr: %2.3f, %2.3f, %2.3f | modded tr: %2.3f, %2.3f, %2.3f | rot: %2.3f, %2.3f, %2.3f\r\n",
		transform.transform.translation.x,
		transform.transform.translation.y,
		transform.transform.translation.z,
		transform_local.transform.translation.x,
		transform_local.transform.translation.y,
		transform_local.transform.translation.z,
		transform_local.transform.rotation.x,
		transform_local.transform.rotation.y,
		transform_local.transform.rotation.z,
		transform_local.transform.rotation.w
	);

	tf2::Quaternion q_rot;
	tf2::convert(transform_local.transform.rotation, q_rot);
	tf2::Matrix3x3 mat(q_rot);
	double roll, pitch, yaw;
	mat.getRPY(roll, pitch, yaw);
	printf("[rotateTwist (2)] transform - rot RPY: %2.3f, %2.3f, %2.3f\r\n", roll, pitch, yaw);
	#endif

	geometry_msgs::Vector3Stamped vel_v3_modified;
	tf2::doTransform(vel_v3, vel_v3_modified, transform_local);
	#ifdef DEBUG_TF
	printf("[rotateTwist (3)] vel_before: %2.3f, %2.3f, %2.3f | vel_after: %2.3f, %2.3f, %2.3f\r\n",
		vel_v3.vector.x,
		vel_v3.vector.y,
		vel_v3.vector.z,
		vel_v3_modified.vector.x,
		vel_v3_modified.vector.y,
		vel_v3_modified.vector.z
	);
	#endif

	// copy meaningful values to the twist msg
	vel.twist.linear.x = vel_v3_modified.vector.x;
	vel.twist.linear.y = vel_v3_modified.vector.y;
	vel.twist.linear.z = vel_v3_modified.vector.z;
	// just to be on the safe side
	vel.twist.angular.x = 0.0;
	vel.twist.angular.y = 0.0;
	vel.twist.angular.z = 0.0;
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
