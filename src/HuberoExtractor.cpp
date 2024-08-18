#include "HuberoExtractor.hpp"

#include <regex>

HuberoExtractor::HuberoExtractor(
	ros::NodeHandle& nh,
	std::atomic<size_t>& id_ref,
	double discovery_frequency
):
	nh_(nh),
	id_ref_(id_ref)
{
	if (discovery_frequency > 0.0) {
		timer_discovery_ = nh_.createTimer(
			ros::Duration(1.0 / discovery_frequency),
			std::bind(
				&HuberoExtractor::refreshActorList,
				this
			)
		);
	} else {
		ROS_WARN("Actor topics discovery will be performed only once!");
		refreshActorList();
	}
}

void HuberoExtractor::refreshActorList() {
	// Define the regex pattern that fits the parameter naming in the `hubero_ros` pkg launch -
	// equivalent to `R"(/hubero_ros/.+/navigation/odometry_topic)"`
	std::string prefix = "/hubero_ros/";
	std::string suffix = "/navigation/odometry_topic";

	// Construct the regex pattern dynamically
	auto regex_escape = [](const std::string& str) -> std::string {
		static const std::regex re(R"([-[\]{}()*+?.,\^$|#\s])");
		return std::regex_replace(str, re, R"(\$&)");
	};
	std::string pattern_str = "^" + regex_escape(prefix) + "(.+)" + regex_escape(suffix) + "$";
	std::regex pattern(pattern_str);

	std::vector<std::string> param_keys;
	// access to, e.g., node handle, subscribers
	std::lock_guard<std::mutex> lock(mutex_);
	nh_.getParamNames(param_keys);

	// stores IDs of actors and names of their localization-related topics
	std::map<std::string, std::string> param_keys_matched;

	// iterate through the vector and check for matches
	for (const auto& key: param_keys) {
		if (!std::regex_match(key, pattern)) {
			continue;
		}
		// extract the substring with the actor ID by removing the prefix and suffix
		std::string actor_name = key.substr(
			prefix.size(),
			key.size() - prefix.size() - suffix.size()
		);
		// `key` defines the name of the parameter (key); topic name is stored as the value associated with that key
		std::string topic;
		if (!nh_.getParam(key, topic)) {
			continue;
		}

		// collect for further use
		param_keys_matched[actor_name] = topic;
		// assign an ID
		detection_ids_[actor_name] = id_ref_.load();
		// increment the reference ID counter
		id_ref_++;
	}

	if (param_keys_matched.empty()) {
		ROS_WARN_DELAYED_THROTTLE(
			30.0,
			"Could not find any ROS Parameters that help finding HuBeRo actor topics. "
			"Did you spawn actors using HuBeRo framework? "
			"Also, note that this node does not work with bag files as ROS Parameter Server contents aren't captured."
		);
		return;
	}

	// check if already subscribed and (potentially) initiate subscribing the localization data
	for (const auto& association: param_keys_matched) {
		std::string actor_name = association.first;
		std::string loc_topic = association.second;
		bool already_handled = false;
		for (const auto& sub: subscribers_) {
			if (sub->getName() != actor_name) {
				continue;
			}
			already_handled = true;
			break;
		}
		if (already_handled) {
			continue;
		}
		// obtain global ID of an actor
		if (detection_ids_.find(actor_name) == detection_ids_.cend()) {
			ROS_ERROR("Could not find an ID for an actor named `%s`", actor_name.c_str());
			continue;
		}
		auto actor_id = detection_ids_[actor_name];
		// create ActorLocalizationSubscriber
		subscribers_.emplace_back(
			std::make_unique<ActorLocalizationSubscriber>(
				nh_,
				actor_id,
				actor_name,
				loc_topic
			)
		);
		ROS_INFO(
			"HuBeRo extractor discovered `%s`'s (ID=%lu) localization topic at `%s`!",
			actor_name.c_str(),
			actor_id,
			loc_topic.c_str()
		);
	}

	// TODO: handle deletion
}
