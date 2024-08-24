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
	auto comp_filter_factor = nh_.param<double>(
		"velocity_filtering_factor",
		GazeboModelExtractor::COMPLEMENTARY_FILTER_INNOVATION_DEFAULT
	);
	auto gazebo_update_period = nh_.param<double>(
		"gazebo_cb_update_period",
		GazeboModelExtractor::GAZEBO_CB_DIFF_TIME
	);

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

	// obtain group arrangements (relations between humans)
	discoverGroupsConfiguration();

	if (!model_name_patterns.size()) {
		ROS_WARN("Gazebo people model extractor will not be formed as the 'model_name_patterns' parameter is empty");
	} else {
		// create a ROS interface for extracting model states
		model_extractor_ = std::make_unique<GazeboModelExtractor>(
			nh_,
			"/gazebo/model_states",
			model_name_patterns,
			models_transforms,
			id_ref_,
			comp_filter_factor,
			gazebo_update_period
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
			id_ref_,
			comp_filter_factor,
			gazebo_update_period
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

void PeopleExtraction::discoverGroupsConfiguration() {
	// for defining unique IDs of groups
	static size_t group_id = 0;

	std::vector<std::string> groups_names;
	nh_.getParam("groups/names", groups_names);
	for (const auto& gname: groups_names) {
		std::vector<std::string> group_entities;
		nh_.getParam("groups/" + gname, group_entities);

		if (group_entities.empty()) {
			ROS_WARN(
				"Group with name '%s' cannot be properly loaded as it has no entities assigned",
				gname.c_str()
			);
			continue;
		}

		// string composition is here only for debugging purposes
		std::string entity_names;
		// Add entities to the original map
		for (const auto& ename: group_entities) {
			entity_names += "'" + ename + "' ";
			groups_arrangement_[group_id].push_back(ename);
		}

		ROS_INFO(
			"Discovered group with name '%s' - it has ID of '%lu' and %lu entities: %s",
			gname.c_str(),
			group_id,
			group_entities.size(),
			entity_names.c_str()
		);

		group_id++;
	}
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

std::map<std::string, people_msgs_utils::Person> PeopleExtraction::gazeboModelsToPeople(
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
		return std::map<std::string, people_msgs_utils::Person>();
	}

	// prepare output container
	std::map<std::string, people_msgs_utils::Person> people;

	for (const auto& [model_name, model_data]: people_models) {
		// retrieve from the value (key is not considered here)
		auto id = model_data.first;
		auto model = model_data.second;

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
		// transform to the target frame
		p.transform(tf_stamped);
		// collect; note that the simulation model name is the key in this map
		people.emplace(model_name, p);
	}

	return people;
}

std::map<std::string, people_msgs_utils::Person> PeopleExtraction::huberoActorsToPeople(
	const std::vector<std::unique_ptr<ActorLocalizationSubscriber>>& actors
) const {
	// prepare output container
	std::map<std::string, people_msgs_utils::Person> people;

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

		// add to the aggregated container; note that the simulation model name is the key in this map
		people.emplace(sub->getName(), p);
	}
	return people;
}

std::vector<std::pair<people_msgs_utils::Person, people_msgs_utils::Group>> PeopleExtraction::createPeopleGroupAssociations(
	const std::map<std::string, people_msgs_utils::Person>& people
) {
	const auto time_now = ros::Time::now().toNSec();

	// duplicate the container at first, but it will have some info updated
	std::map<std::string, people_msgs_utils::Person> people_updated_with_groups = people;

	// groups container
	std::vector<people_msgs_utils::Group> groups;

	for (const auto& [group_id, group_members_names]: groups_arrangement_) {
		// in this loop, we prepare arguments for group creation
		std::map<std::string, people_msgs_utils::Person> people_in_group;
		std::vector<std::string> member_ids_in_group;

		// first iteration - gather group members and their IDs
		for (const auto& [person_full_name, person]: people) {
			// check whether the person name is found within group members patterns
			bool pattern_matched = false;
			std::string pattern;
			std::tie(pattern_matched, pattern) = link_extractor_->doesNameMatchPatterns(person_full_name, group_members_names);
			if (!pattern_matched) {
				continue;
			}
			people_in_group.emplace(person_full_name, person);
			member_ids_in_group.push_back(person.getName());
		}

		if (people_in_group.empty()) {
			continue;
		}

		// second iteration - we need to properly assign group IDs to people
		auto people_in_group_illformed = people_in_group;
		people_in_group.clear();
		for (const auto& [person_full_name, person_wo_group]: people_in_group_illformed) {
			// prepare ctor args
			geometry_msgs::PoseWithCovariance pose;
			pose.pose = person_wo_group.getPose();
			auto pose_covariance = person_wo_group.getCovariancePose();
			std::copy(pose_covariance.begin(), pose_covariance.end(), pose.covariance.begin());

			geometry_msgs::PoseWithCovariance velocity;
			velocity.pose = person_wo_group.getVelocity();
			auto velocity_covariance = person_wo_group.getCovarianceVelocity();
			std::copy(velocity_covariance.begin(), velocity_covariance.end(), velocity.covariance.begin());

			// create correctly formed instance (with group)
			people_msgs_utils::Person person_with_group(
				person_wo_group.getName(),
				pose,
				velocity,
				person_wo_group.getReliability(),
				person_wo_group.isOccluded(),
				person_wo_group.isMatched(),
				person_wo_group.getDetectionID(),
				person_wo_group.getTrackAge(),
				std::to_string(group_id)
			);
			people_in_group.emplace(person_full_name, person_with_group);
			people_updated_with_groups.insert_or_assign(person_full_name, person_with_group);
		}

		// third iteration - collect relations between members
		const double RELATION_STRENGTH = 1.0; // hard coded with a maximum value due to the ideal data
		std::vector<std::tuple<std::string, std::string, double>> member_relations;
		for (const auto& person_with_full_name_1: people_in_group) {
			auto person1 = person_with_full_name_1.second;
			for (const auto& person_with_full_name_2: people_in_group) {
				auto person2 = person_with_full_name_2.second;
				if (person1.getName() == person2.getName()) {
					continue;
				}
				member_relations.push_back(std::make_tuple(person1.getName(), person2.getName(), RELATION_STRENGTH));
			}
		}

		// fourth iteration - compute center of gravity of the group - mean of positions of the members
		geometry_msgs::Point center_of_gravity;
		for (const auto& person_with_full_name: people_in_group) {
			auto person = person_with_full_name.second;
			center_of_gravity.x += person.getPositionX();
			center_of_gravity.y += person.getPositionY();
			center_of_gravity.z += person.getPositionZ();
		}
		center_of_gravity.x /= people_in_group.size();
		center_of_gravity.y /= people_in_group.size();
		center_of_gravity.z /= people_in_group.size();

		// fifth interation: create a container with only Person objects
		std::vector<people_msgs_utils::Person> people_in_group_wo_full_names;
		for (const auto& person_with_full_name: people_in_group) {
			people_in_group_wo_full_names.push_back(person_with_full_name.second);
		}

		// collect group in an aggregated container
		groups.emplace_back(
			std::to_string(group_id),
			time_now, // time since the system startup as we have ideal data
			people_in_group_wo_full_names,
			member_ids_in_group,
			member_relations,
			center_of_gravity
		);
	}

	// prepare associations so they can be easily used in publishing methods (there is some memory overhead
	// due to duplicating groups)
	std::vector<std::pair<people_msgs_utils::Person, people_msgs_utils::Group>> people_grouped;

	// NOTE: we cannot rely on 'people' container as its Person instances have not group names assigned
	for (const auto& person_with_full_name: people_updated_with_groups) {
		// obtain only the Person instance, ignore the full name
		auto person = person_with_full_name.second;
		if (!person.isAssignedToGroup()) {
			// person without a group - create a dummy one
			people_grouped.emplace_back(person, people_msgs_utils::Group());
			continue;
		}

		// search for the correct group instance
		bool group_found = false;
		for (const auto& group: groups) {
			if (person.getGroupName() != group.getName()) {
				continue;
			}
			people_grouped.emplace_back(person, group);
			group_found = true;
		}
		if (group_found) {
			continue;
		}
		// a proper group was not found
		people_grouped.emplace_back(person, people_msgs_utils::Group());
	}
	return people_grouped;
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

	// another helper function
	auto extendPeopleMap = [](
		std::map<std::string, people_msgs_utils::Person>& dest,
		const std::map<std::string, people_msgs_utils::Person>& src
	) {
		// we cannot use `map[key] = value` as the `value` here does not have a default constructor
		for (auto const& [key, val]: src) {
			dest.emplace(key, val);
		}
	};

	// aggregated container
	std::map<std::string, people_msgs_utils::Person> people;
	// Gazebo models to people representation
	extendPeopleMap(people, gazeboModelsToPeople(people_gazebo));
	// HuBeRo actors to people
	extendPeopleMap(people, huberoActorsToPeople(people_hubero));

	// associate group data for each person
	auto people_grouped = createPeopleGroupAssociations(people);

	publishPeople(people_grouped);
	publishPeoplePositions(people_grouped);
}

void PeopleExtraction::publishPeople(
	const std::vector<std::pair<people_msgs_utils::Person, people_msgs_utils::Group>>& people_grouped
) {
	people_msgs::People people_msg;

	people_msg.header.frame_id = target_tf_frame_;
	people_msg.header.stamp = ros::Time::now();
	for (const auto& [person, group]: people_grouped) {
		people_msg.people.push_back(person.toPersonStd(group));
	}
	pub_people_.publish(people_msg);
}

void PeopleExtraction::publishPeoplePositions(
	const std::vector<std::pair<people_msgs_utils::Person, people_msgs_utils::Group>>& people_grouped
) {
	people_msgs::PositionMeasurementArray pos_msg_array;

	pos_msg_array.header.frame_id = target_tf_frame_;
	pos_msg_array.header.stamp = ros::Time::now();

	for (auto person_with_group: people_grouped) {
		auto person = person_with_group.first;

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
