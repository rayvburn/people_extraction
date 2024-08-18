#pragma once

#include <ros/ros.h>

#include <atomic>
#include <mutex>
#include <map>
#include <string>
#include <vector>

/**
 * @brief Class that subscribes messages from a given topic and stores them for easy access (on demand)
 *
 * @tparam Tcb defines the type of the callback routine
 * @tparam Tsub defines the type of the subscriber
 * @tparam Tcontainer defines the type of the actual data storage
 */
template <typename Tcb, typename Tsub, typename Tcontainer>
class GazeboExtractor {
public:
    GazeboExtractor(
        ros::NodeHandle& nh,
        const std::string& topic_name,
        const std::vector<std::string>& name_patterns,
        std::atomic<size_t>& id_ref
    ):
        name_patterns_(name_patterns),
        id_ref_(id_ref)
    {
        sub_ = nh.subscribe<Tsub>(
            topic_name,
            1,
            &GazeboExtractor::callback,
            this
        );
    }

    std::map<std::string, std::pair<size_t, Tcontainer>> getPeople() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return people_;
    }

protected:
	/**
	 * @brief
	 * @note Callback triggered by a high frequency Gazebo topic
	 * @param msg
	 */
	void callback(const Tcb& msg) {
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

            std::tie(pattern_matched, pattern) = isMatching(name, name_patterns_);
            if (!pattern_matched) {
                continue;
            }

            models_found++;
            Tcontainer model;
            model.model_name = name;
            model.pose = msg->pose.at(i);
            model.twist = msg->twist.at(i);

            // check whether the person already exists in the internal database (only need ModelState update then)
            if (doesPersonExist(name)) {
                people_[name].second = model;
                continue;
            }
            // new model - assign an ID and update its ModelState
            people_[name] = {id_ref_.load(), model};
            // increment the reference ID counter
            id_ref_++;
            ROS_INFO(
                "Tracking a new person '%s' (ID: '%lu'), detected from naming pattern: '%s'",
                people_[name].second.model_name.c_str(),
                people_[name].first,
                pattern.c_str()
            );
        }

        // possibly delete renamed/destroyed objects
        deleteDestroyedPerson(msg->name);
    }

    /**
	 * @brief Evaluates, whether the person exists in the internal database
	 * @param name: name of the person that is searched for
	 * @return
	 */
	bool doesPersonExist(const std::string& name) const {
        return people_.find(name) != people_.cend();
    }

	/**
	 * @brief Evaluates whether the given @ref model_name matches any pattern given by @ref patterns
	 *
	 * @param model_name
	 * @param patterns
	 * @return std::tuple<bool, std::string>
	 */
	std::tuple<bool, std::string> isMatching(
        const std::string& model_name,
        std::vector<std::string>& patterns
    ) const {
        for (const auto& pattern: patterns) {
            // try to find
            if (model_name.find(pattern) == std::string::npos) {
                continue;
            }
            return {true, pattern};
        }
        return {false, std::string()};
    }

	/**
	 * @brief Get rid of the unnecessary model
	 * @param people people identified so far
	 * @param model_names names of entities recognized in the current step
	 */
	void deleteDestroyedPerson(const std::vector<std::string>& model_names) {
        auto i = std::begin(people_);
        // while is safer than for here
        while (i != std::end(people_)) {
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
            i = people_.erase(i);
        }
    }

	ros::Subscriber sub_;
    mutable std::mutex mutex_;

    /// @brief Algorithm looks for a model name that meets one of the given patterns
	std::vector<std::string> name_patterns_;

    // For assigning IDs between multiple classes
    std::atomic<size_t>& id_ref_;

	// key: name of the object in the simulation
	// value: pair with object's ID and the Gazebo data structure
	std::map<std::string, std::pair<size_t, Tcontainer>> people_;
};
