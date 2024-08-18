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
	id_ref_(0),
	tf_listener_(tf_buffer_)
{
	auto pub_frequency = nh_.param<double>("pub_frequency", 30.0);
	nh_.param<std::string>("world_frame", world_tf_frame_, "world");
	nh_.param<std::string>("target_frame", target_tf_frame_, "odom");

	// obtain model and link name patterns
	std::vector<std::string> model_name_patterns;
	nh_.getParam("model_name_patterns", model_name_patterns);
	if (!model_name_patterns.size()) {
		ROS_ERROR("PeopleExtraction - model_name_patterns is empty! Class would be ill-formed");
		return;
	}
	std::vector<std::string> link_name_patterns;
	nh_.getParam("link_name_patterns", link_name_patterns);
	if (!link_name_patterns.size()) {
		ROS_ERROR("PeopleExtraction - link_name_patterns is empty! Class would be ill-formed");
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
	model_extractor_ = std::make_unique<GazeboModelExtractor>(
		nh_,
		"/gazebo/model_states",
		model_name_patterns,
		id_ref_
	);

	link_extractor_ = std::make_unique<GazeboLinkExtractor>(
		nh_,
		"/gazebo/link_states",
		link_name_patterns,
		id_ref_
	);

	pub_people_ = nh_.advertise<people_msgs::People>("/people", 5);
	pub_pos_ = nh_.advertise<people_msgs::PositionMeasurementArray>("/people_measurements", 5);

	timer_pub_ = nh_.createTimer(
		ros::Duration(1.0 / pub_frequency),
		std::bind(
			&PeopleExtraction::publish,
			this
		)
	);
}

people_msgs_utils::People PeopleExtraction::gazeboModelsToPeople(
	const std::map<std::string, std::pair<size_t, gazebo_msgs::ModelState>>& people_models
) const {
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

	for (auto const& model_data: people_models) {
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
	// combine entities obtained from "models" and from "links"
	std::map<std::string, std::pair<size_t, gazebo_msgs::ModelState>> people_gazebo;
	// start with copying 1st and extend with the 2nd container
	people_gazebo = model_extractor_->getPeople();
	for (auto const& [key, val]: link_extractor_->getPeople()) {
		people_gazebo[key] = val;
	}

	// check if there is something to publish
	if (people_gazebo.empty()) {
		return;
	}

	// Gazebo models to people representation
	auto people = gazeboModelsToPeople(people_gazebo);

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
