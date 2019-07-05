// Copyright 2019 Project March.
#include "march_safety/SafetyHandler.h"

SafetyHandler::SafetyHandler(ros::NodeHandle* n, ros::Publisher* error_publisher, ros::Publisher* sound_publisher)
  : n(n), error_publisher(error_publisher), sound_publisher(sound_publisher)
{
}

void SafetyHandler::publishErrorMessage(const std::string& message, int8_t error_type) const
{
  march_shared_resources::Error error_msg;
  std::ostringstream message_stream;
  error_msg.error_message = message;
  error_msg.type = error_type;
  error_publisher->publish(error_msg);
}

void SafetyHandler::publishErrorSound(int8_t error_type) const
{
  march_shared_resources::Sound sound;
  sound.time = ros::Time::now();
  if (error_type == march_shared_resources::Error::FATAL)
  {
    sound.file_name = "fatal.wav";
  }
  else if (error_type == march_shared_resources::Error::NON_FATAL)
  {
    sound.file_name = "non-fatal.wav";
  }
  sound_publisher->publish(sound);
}

void SafetyHandler::publishFatal(std::string message)
{
  ROS_ERROR("%s", message.c_str());

  publishErrorMessage(message, march_shared_resources::Error::FATAL);

  publishErrorSound(march_shared_resources::Error::FATAL);
}

void SafetyHandler::publishNonFatal(std::string message)
{
  ROS_ERROR("%s", message.c_str());

  publishErrorMessage(message, march_shared_resources::Error::NON_FATAL);

  publishErrorSound(march_shared_resources::Error::NON_FATAL);
}