#pragma once

#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <angles/angles.h>

#include <atomic>
#include <mutex>
#include <map>
#include <string>
#include <tuple>
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
	/// For velocity filtering, how much we trust new observations, i.e., how much we are relying on new readings
	static constexpr auto COMPLEMENTARY_FILTER_INNOVATION_DEFAULT = 0.2;
	static constexpr auto GAZEBO_CB_DIFF_TIME = 0.05;

	GazeboExtractor(
		ros::NodeHandle& nh,
		const std::string& topic_name,
		const std::vector<std::string>& name_patterns,
		const std::map<std::string, std::vector<double>>& transforms,
		std::atomic<size_t>& id_ref,
		double comp_filter_innovation = COMPLEMENTARY_FILTER_INNOVATION_DEFAULT,
		double cb_time_diff = GAZEBO_CB_DIFF_TIME
	):
		name_patterns_(name_patterns),
		transforms_(transforms),
		id_ref_(id_ref),
		time_last_update_(NAN),
		comp_filter_innovation_(comp_filter_innovation),
		cb_time_diff_(cb_time_diff)
	{
		sub_ = nh.subscribe<Tsub>(
			topic_name,
			1,
			&GazeboExtractor::callback,
			this
		);
	}

	inline std::map<std::string, std::pair<size_t, Tcontainer>> getPeople() const {
		std::lock_guard<std::mutex> lock(mutex_);
		return people_;
	}

	/**
	 * @brief Evaluates whether the given @ref model_name matches any pattern given by @ref patterns
	 *
	 * @param model_name
	 * @param patterns
	 * @return std::tuple<bool, std::string>
	 */
	static std::tuple<bool, std::string> doesNameMatchPatterns(
		const std::string& model_name,
		const std::vector<std::string>& patterns
	) {
		for (const auto& pattern: patterns) {
			// try to find
			if (model_name.find(pattern) == std::string::npos) {
				continue;
			}
			return {true, pattern};
		}
		return {false, std::string()};
	}

protected:
	/**
	 * @brief
	 * @note Callback triggered by a high frequency Gazebo topic
	 * @param msg
	 */
	void callback(const Tcb& msg) {
		if (std::isnan(time_last_update_)) {
			// omit the first callback
			time_last_update_ = ros::Time::now().toSec();
		}

		if (msg->name.size() != msg->pose.size() || msg->name.size() != msg->twist.size()) {
			ROS_ERROR("Vector sizes in the input message are not equal, cannot process further");
			return;
		}

		// check timing conditions (manual calculation of velocities)
		double time_current = ros::Time::now().toSec();
		double time_diff = time_current - time_last_update_;
		if (time_diff < cb_time_diff_) {
			return;
		}
		time_last_update_ = time_current;

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
			std::tie(pattern_matched, pattern) = doesNameMatchPatterns(name, name_patterns_);
			if (!pattern_matched) {
				continue;
			}

			models_found++;
			Tcontainer model;
			model.model_name = name;
			model.pose = msg->pose.at(i);
			model.twist = msg->twist.at(i);

			std::vector<double> transform;
			if (transforms_.find(pattern) != transforms_.cend()) {
				transform = transforms_[pattern];
			} else {
				transform = std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
			}

			// check whether the person already exists in the internal database (only need ModelState update then)
			if (doesPersonExist(name)) {
				// manually compute the new state a some Gazebo model might report wrong velocities
				model = computeState(
					people_[name].second, // previously saved "model"
					model,
					transform,
					time_diff
				);
				people_[name].second = model;
				continue;
			}

			// special stage for a proper initialization
			std::tie(model, std::ignore, std::ignore, std::ignore) = computeStateTransformed(model, transform);
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

	/// @brief Returns instance of desired type and R, P, Y angles
	std::tuple<Tcontainer, double, double, double> computeStateTransformed(
		const Tcontainer& state,
		const std::vector<double>& transform
	) const {
		Tcontainer state_corrected;
		// copy name first
		state_corrected.model_name = state.model_name;

		state_corrected.pose.position.x = state.pose.position.x + transform.at(0);
		state_corrected.pose.position.y = state.pose.position.y + transform.at(1);
		state_corrected.pose.position.z = state.pose.position.z + transform.at(2);
		// original quaternion to roll pitch yaw
		tf::Quaternion q_raw(
			state.pose.orientation.x,
			state.pose.orientation.y,
			state.pose.orientation.z,
			state.pose.orientation.w
		);
		tf::Matrix3x3 m_raw(q_raw);
		double roll, pitch, yaw;
		m_raw.getRPY(roll, pitch, yaw);
		// apply transformation as RPY
		roll = angles::normalize_angle(roll + transform.at(3));
		pitch = angles::normalize_angle(pitch + transform.at(4));
		yaw = angles::normalize_angle(yaw + transform.at(5));
		// convert back to quaternion - body orientation in the current step
		tf::Quaternion q;
		q.setRPY(roll, pitch, yaw);

		state_corrected.pose.orientation.x = q.x();
		state_corrected.pose.orientation.y = q.y();
		state_corrected.pose.orientation.z = q.z();
		state_corrected.pose.orientation.w = q.w();
		return {state_corrected, roll, pitch, yaw};
	}

	Tcontainer computeState(
		const Tcontainer& state_prev,
		const Tcontainer& state_curr_raw,
		const std::vector<double>& transform,
		double time_diff
	) const {
		// first, we need to apply the requested transformation to switch from `state_curr_raw` to `state_curr`
		Tcontainer state_curr;
		double roll_curr, pitch_curr, yaw_curr;
		std::tie(state_curr, roll_curr, pitch_curr, yaw_curr) = computeStateTransformed(state_curr_raw, transform);

		// filtering factors
		double ff_old = 1.0 - comp_filter_innovation_;
		double ff_new = comp_filter_innovation_;

		// linear velocities (twists)
		double tlx = ff_old * state_prev.twist.linear.x + ff_new * (state_curr.pose.position.x - state_prev.pose.position.x) / time_diff;
		double tly = ff_old * state_prev.twist.linear.y + ff_new * (state_curr.pose.position.y - state_prev.pose.position.y) / time_diff;
		double tlz = ff_old * state_prev.twist.linear.z + ff_new * (state_curr.pose.position.z - state_prev.pose.position.z) / time_diff;

		// body orientation in the previous step
		tf::Quaternion q_prev(
			state_prev.pose.orientation.x,
			state_prev.pose.orientation.y,
			state_prev.pose.orientation.z,
			state_prev.pose.orientation.w
		);
		tf::Matrix3x3 mat_q_prev(q_prev);
		double roll_prev, pitch_prev, yaw_prev;
		mat_q_prev.getRPY(roll_prev, pitch_prev, yaw_prev);

		// angular velocities
		double tax = ff_old * state_prev.twist.angular.x + ff_new * (roll_curr - roll_prev) / time_diff;
		double tay = ff_old * state_prev.twist.angular.y + ff_new * (pitch_curr - pitch_prev) / time_diff;
		double taz = ff_old * state_prev.twist.angular.z + ff_new * (yaw_curr - yaw_prev) / time_diff;

		// updated state
		Tcontainer state;
		// copy name
		state.model_name = state_curr_raw.model_name;
		// pose is directly inherited from the simulator
		state.pose = state_curr.pose;
		// twist calculated "manually"
		state.twist.linear.x = tlx;
		state.twist.linear.y = tly;
		state.twist.linear.z = tlz;
		state.twist.angular.x = tax;
		state.twist.angular.y = tay;
		state.twist.angular.z = taz;
		return state;
	};

	/**
	 * @brief Evaluates, whether the person exists in the internal database
	 * @param name: name of the person that is searched for
	 * @return
	 */
	bool doesPersonExist(const std::string& name) const {
		return people_.find(name) != people_.cend();
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
	std::map<std::string, std::vector<double>> transforms_;

	// For assigning IDs between multiple classes
	std::atomic<size_t>& id_ref_;

	// key: name of the object in the simulation
	// value: pair with object's ID and the Gazebo data structure
	std::map<std::string, std::pair<size_t, Tcontainer>> people_;
	/// ROS time of the last update stage
	double time_last_update_;
	/// Innovation factor of the complementary filter for smoothing velocities
	double comp_filter_innovation_;
	/// How often (in ROS time), Gazebo callback should be processed
	double cb_time_diff_;
};
