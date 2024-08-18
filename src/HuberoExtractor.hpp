#pragma once

#include <ros/ros.h>

#include <atomic>
#include <list>
#include <map>
#include <memory>
#include <mutex>

#include "ActorLocalizationSubscriber.hpp"

class HuberoExtractor {
public:
	HuberoExtractor(
		ros::NodeHandle& nh,
		std::atomic<size_t>& id_ref,
		double discovery_frequency
	);

	/// Refreshes the list of relevant HuBeRo parameters and updates the list of @ref subscribers_
	void refreshActorList();

	// Returns a list of data collecting instances
	inline const std::vector<std::unique_ptr<ActorLocalizationSubscriber>>& get() const {
		std::lock_guard<std::mutex> lock(mutex_);
		return subscribers_;
	}

protected:
	ros::NodeHandle& nh_;

	// Vector of pointers due to the fact that stored objects contain non-copyable mutexes
	std::vector<std::unique_ptr<ActorLocalizationSubscriber>> subscribers_;

	// For assigning IDs between multiple classes
	std::atomic<size_t>& id_ref_;

	std::map<std::string, size_t> detection_ids_;
	ros::Timer timer_discovery_;
	mutable std::mutex mutex_;
};
