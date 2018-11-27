//
// Created by ishadijcks on 27-11-18.
//

#include "LaunchAPI.h"
#include "../Validator.h"
#include "march_api/Trigger.h"
#include "ros/ros.h"

bool LaunchAPI::urdf_validator(march_api::Trigger::Request& request, march_api::Trigger::Response& response)
{
  ROS_INFO("URDF validator called");

  /**
   * TODO lookup folder with all URDF files.
   * Loop through all files and perfows the check for each file
   */

  Result urdf = Validator::checkURDF("dummy_file");
  response.success = urdf.success;
  response.message = urdf.message;
  return true;
}

bool LaunchAPI::config_validator(march_api::Trigger::Request& request, march_api::Trigger::Response& response)
{
  ROS_INFO("config validator called");

  Result config = Validator::checkConfig();
  response.success = config.success;
  response.message = config.message;
  return true;
}

bool LaunchAPI::xml_validator(march_api::Trigger::Request& request, march_api::Trigger::Response& response)
{
  ROS_INFO("xml validator called");

  Result hardware = Validator::checkXml();
  response.success = hardware.success;
  response.message = hardware.message;
  return true;
}
