#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include <mutex>
#include <string>

class ActorLocalizationSubscriber {
public:
	/**
	 * @param nh node handle to use
	 * @param topic topic to subscribe odometry-localization messages
	 */
	ActorLocalizationSubscriber(
		ros::NodeHandle& nh,
		const size_t& actor_id,
		const std::string& actor_name,
		const std::string& topic
	):
		id_(actor_id),
		name_(actor_name),
		topic_(topic)
	{
		sub_ = nh.subscribe<nav_msgs::Odometry>(
			topic_,
			10,
			std::bind(
				&ActorLocalizationSubscriber::callback,
				this,
				std::placeholders::_1
			)
		);
	}

	nav_msgs::Odometry getOdom() const {
		std::lock_guard<std::mutex> lock(mutex_);
		return odom_;
	}

	size_t getID() const {
		return id_;
	}

	std::string getName() const {
		return name_;
	}

	std::string getTopic() const {
		return topic_;
	}

protected:
	void callback(const nav_msgs::Odometry::ConstPtr& msg) {
		std::lock_guard<std::mutex> lock(mutex_);
		odom_ = *msg;
	}

	mutable std::mutex mutex_;
	ros::Subscriber sub_;
	nav_msgs::Odometry odom_;
	size_t id_;
	std::string name_;
	std::string topic_;
};
