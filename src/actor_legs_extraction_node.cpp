/*
 * actor_legs_extraction.cpp
 *
 *  Created on: Dec 19, 2020
 *      Author: rayvburn
 */

// --------------------------------------------------------------

#include <ros/ros.h>
#include "LegsExtraction.hpp"

int main(int argc, char** argv)
{

  ros::init(argc, argv, "actor_legs_extraction");
  LegsExtraction legs;
  ros::spin();
  return 0;

}
