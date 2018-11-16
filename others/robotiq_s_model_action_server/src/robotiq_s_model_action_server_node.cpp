#include "robotiq_action_server/robotiq_s_model_action_server.h"

namespace
{
  // Defines a default for the s model
  robotiq_action_server::SModelGripperParams s_defaults(std::string gripper_mode)
  {
    robotiq_action_server::SModelGripperParams params;

    if (gripper_mode == "wide_pinch")
    {
        params.min_rad_ = 0.0495;
        params.max_rad_ = 0.933;
        params.min_effort_ = 40.0; // This is a guess. Could not find data with quick search.
        params.max_effort_ = 100.0;
    }
    else
    {
        if (gripper_mode != "basic")
        {
            ROS_WARN("Gripper mode %s not known! Using basic mode.", gripper_mode.c_str());
            gripper_mode = "basic";
        }
        params.min_rad_ = 0.0495;
        params.max_rad_ = 1.222;
        params.min_effort_ = 40.0; // This is a guess. Could not find data with quick search.
        params.max_effort_ = 100.0;
    }

    params.gripper_mode_ = gripper_mode;
    return params;
  }
}

int main(int argc, char** argv)
{
  // Can be renamed with standard ROS-node launch interface
  ros::init(argc, argv, "gripper_action_server");
  
  // Private Note Handle for retrieving parameter arguments to the server
  ros::NodeHandle private_nh("~");

  std::string gripper_name;
  private_nh.param<std::string>("gripper_name", gripper_name, "gripper");

  std::string gripper_mode;
  private_nh.param<std::string>("gripper_mode", gripper_mode, "basic");

  // Fill out S-Model Params
  robotiq_action_server::SModelGripperParams cparams = s_defaults(gripper_mode);

  ROS_INFO("Initializing Robotiq action server for gripper: %s", gripper_name.c_str());

  // The name of the gripper -> this server communicates over name/inputs and name/outputs
  robotiq_action_server::SModelGripperActionServer gripper (gripper_name, cparams);

  ROS_INFO("Robotiq action-server spinning for gripper: %s", gripper_name.c_str());
  ros::spin();
}
