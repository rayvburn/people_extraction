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
	auto discovery_frequency = nh_.param<double>("discovery_frequency", 0.5);
	auto pub_frequency = nh_.param<double>("pub_frequency", 30.0);
	nh_.param<std::string>("world_frame", world_tf_frame_, "world");
	nh_.param<std::string>("target_frame", target_tf_frame_, "odom");

	// publishing frequency check
	if (pub_frequency <= 0.0) {
		ROS_ERROR(
			"Simulator-based people data will not be published as wrong frequency was selected: %f Hz",
			pub_frequency
		);
		return;
	}
	// TF frame names validity check
	if (world_tf_frame_.empty()) {
		ROS_ERROR("Cannot run HuBeRo actors data publisher, as the `target_frame` parameter is empty!");
		return;
	}
	if (target_tf_frame_.empty()) {
		ROS_ERROR("Cannot run HuBeRo actors data publisher, as the `target_frame` parameter is empty!");
		return;
	}

	// TF frame names validity check
	if (world_tf_frame_.empty()) {
		ROS_ERROR("Cannot run HuBeRo actors data publisher, as the `target_frame` parameter is empty!");
		return;
	}
	if (target_tf_frame_.empty()) {
		ROS_ERROR("Cannot run HuBeRo actors data publisher, as the `target_frame` parameter is empty!");
		return;
	}

	// obtain model and link name patterns
	std::vector<std::string> model_name_patterns;
	std::vector<std::string> link_name_patterns;
	nh_.getParam("model_name_patterns", model_name_patterns);
	nh_.getParam("link_name_patterns", link_name_patterns);

	// obtain additional transformations (patterns from above are needed)
	auto models_transforms = discoverTransforms(model_name_patterns);
	auto links_transforms = discoverTransforms(link_name_patterns);

	if (!model_name_patterns.size()) {
		ROS_WARN("Gazebo people model extractor will not be formed as the 'model_name_patterns' parameter is empty");
	} else {
		// create a ROS interface for extracting model states
		model_extractor_ = std::make_unique<GazeboModelExtractor>(
			nh_,
			"/gazebo/model_states",
			model_name_patterns,
			models_transforms,
			id_ref_
		);
	}

	if (!link_name_patterns.size()) {
		ROS_WARN("Gazebo people link extractor will not be formed as the 'link_name_patterns' parameter is empty");
	} else {
		// create a ROS interface for extracting link states
		link_extractor_ = std::make_unique<GazeboLinkExtractor>(
			nh_,
			"/gazebo/link_states",
			link_name_patterns,
			links_transforms,
			id_ref_
		);
	}

	hubero_extractor_ = std::make_unique<HuberoExtractor>(
		nh_,
		id_ref_,
		discovery_frequency
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

std::map<std::string, std::vector<double>> PeopleExtraction::discoverTransforms(
	const std::vector<std::string>& name_patterns
) const {
	std::map<std::string, std::vector<double>> transforms;
	for (const auto& pattern: name_patterns) {
		std::string param = "transforms/" + pattern;
		std::vector<double> transform;
		if (!nh_.getParam(param, transform)) {
			continue;
		}
		if (transform.size() != 6) {
			ROS_ERROR(
				"Discovered additional transform for '%s' pattern but it has %lu elements, while 6 are required",
				pattern.c_str(),
				transform.size()
			);
			continue;
		}
		transforms[pattern] = transform;
		ROS_INFO(
			"Discovered additional transform for '%s' pattern with %lu elements\r\n",
			pattern.c_str(),
			transform.size()
		);
	}
	return transforms;
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

people_msgs_utils::People PeopleExtraction::huberoActorsToPeople(
	const std::vector<std::unique_ptr<ActorLocalizationSubscriber>>& actors
) const {
	// prepare output container
	people_msgs_utils::People people;

	// access to, e.g., TF buffer, subscribers
	for (const auto& sub: actors) {
		auto odom = sub->getOdom();

		// transform localization data to the desired frame
		geometry_msgs::TransformStamped tf_stamped;
		// by default - lack of translation and rotation
		tf_stamped.transform.rotation.w = 1.0;

		// obtain the transform to the desired frame
		try {
			tf_stamped = tf_buffer_.lookupTransform(
				target_tf_frame_,
				odom.header.frame_id,
				ros::Time(0)
			);
		} catch (tf2::TransformException& ex) {
			ROS_ERROR(
				"Could not transform `%s`'s pose! Exception details: `%s`",
				sub->getName().c_str(),
				ex.what()
			);
			// avoid flooding console with this kind of errors (localization might not be operational yet)
			std::this_thread::sleep_for(std::chrono::seconds(1));
			return people;
		}

		// convert data from twist to pose representation (contents are the same)
		geometry_msgs::PoseWithCovariance odom_vel;
		odom_vel.covariance = odom.twist.covariance;
		odom_vel.pose.position.x = odom.twist.twist.linear.x;
		odom_vel.pose.position.y = odom.twist.twist.linear.y;
		odom_vel.pose.position.z = odom.twist.twist.linear.z;
		tf2::Quaternion angular_vel;
		// NOTE: for a proper yaw angle, yaw, pitch and roll angles must be reordered (compared to the documentation)
		angular_vel.setEuler(
			odom.twist.twist.angular.x,
			odom.twist.twist.angular.y,
			odom.twist.twist.angular.z
		);
		odom_vel.pose.orientation = tf2::toMsg(angular_vel);

		// create a Person instance (with geom. data in the source frame)
		people_msgs_utils::Person p(
			std::to_string(sub->getID()), // avoid using 'string'-type names here
			odom.pose,
			odom_vel,
			1.0, // complete reliability
			false,
			true,
			sub->getID(),
			ros::Time::now().toNSec(), // track age = since startup as we have ideal data
			std::string("") // group data not included
		);
		// TODO: include group data (e.g., detect "follow" task execution)

		// transform to the target frame
		p.transform(tf_stamped);

		// add to the aggregated container
		people.push_back(p);
	}
	return people;
}

void PeopleExtraction::publish() {
	// helper function
	auto extendMap = [](
		std::map<std::string, std::pair<size_t, gazebo_msgs::ModelState>>& dest,
		const std::map<std::string, std::pair<size_t, gazebo_msgs::ModelState>>& src
	) {
		for (auto const& [key, val]: src) {
			dest[key] = val;
		}
	};

	// combine entities obtained from "models" and from "links"
	std::map<std::string, std::pair<size_t, gazebo_msgs::ModelState>> people_gazebo;
	if (model_extractor_) {
		extendMap(people_gazebo, model_extractor_->getPeople());
	}
	if (link_extractor_) {
		extendMap(people_gazebo, link_extractor_->getPeople());
	}

	// HuBeRo actors
	const auto& people_hubero = hubero_extractor_->get();

	// check if there is something to publish
	if (people_gazebo.empty() && people_hubero.empty()) {
		return;
	}

	// Gazebo models to people representation
	auto people = gazeboModelsToPeople(people_gazebo);
	// HuBeRo actors to people
	for (const auto& person: huberoActorsToPeople(people_hubero)) {
		people.push_back(person);
	}

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
