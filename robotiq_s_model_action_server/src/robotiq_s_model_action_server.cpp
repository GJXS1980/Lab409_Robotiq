/**
 * ActionServer interface to the control_msgs/GripperCommand action
 * for a Robotiq S-Model device
 */

#include "robotiq_action_server/robotiq_s_model_action_server.h"

// To keep the fully qualified names managable

//Anonymous namespaces are file local -> sort of like global static objects
namespace
{
  using namespace robotiq_action_server;

  /*  This struct is declared for the sole purpose of being used as an exception internally
      to keep the code clean (i.e. no output params). It is caught by the action_server and 
      should not propogate outwards. If you use these functions yourself, beware.
  */
  struct BadArgumentsError {};


  GripperOutput goalToRegisterState(const GripperCommandGoal& goal, const SModelGripperParams& params)
  {
    GripperOutput result;
    result.rACT = 0x1; // active gripper
    result.rGTO = 0x1; // go to position
    result.rATR = 0x0; // No emergency release
    if (params.gripper_mode_ == "wide_pinch")
    {
      result.rICS = 1;   // individual control of scissor
      result.rSPA = 80;  // finger speed; 80 for this specific case
      result.rSPS = 255; // scissor speed
    }
    else // basic mode
    {
      result.rSPA = 128;  // finger speed; 80 for this specific case
    }

    if (goal.command.position > params.max_rad_ || goal.command.position < params.min_rad_)
    {
      ROS_WARN("Goal gripper rad size is out of range(%f to %f): %f m",
               params.min_rad_, params.max_rad_, goal.command.position);
      throw BadArgumentsError();
    }

    if (goal.command.max_effort < params.min_effort_ || goal.command.max_effort > params.max_effort_)
    {
      ROS_WARN("Goal gripper effort out of range (%f to %f N): %f N",
               params.min_effort_, params.max_effort_, goal.command.max_effort);
      throw BadArgumentsError();
    }

    // The register value for the closed pinch mode is 113,
    // the minimal register value for the finger is 6 , so wie divide with 107
    // analog for the basic mode

    double dist_per_tick;
    if (params.gripper_mode_ == "wide_pinch")
      dist_per_tick = (params.max_rad_ - params.min_rad_) / 107;
    else // basic mode
      dist_per_tick = (params.max_rad_ - params.min_rad_) / 235;

    double eff_per_tick = (params.max_effort_ - params.min_effort_) / 255;

    result.rPRA = static_cast<uint8_t>((goal.command.position) / dist_per_tick);
    result.rFRA = static_cast<uint8_t>((goal.command.max_effort - params.min_effort_) / eff_per_tick);

    /* Register Value for scissor position: calculatet with the position of fingers (between 6 and 113)
       multiplied with max scissor position divided by max finger position.
       I'm not sure why it musst be 212 and 113 and noch 197 and 107, cause the scissor position starts with 15 and 
       the finger position with 6.
    */

    if (params.gripper_mode_ == "wide_pinch")
      result.rPRS = static_cast<uint8_t>(result.rPRA*(212.0/113.0));

    ROS_INFO("Setting goal position register to %hhu", result.rPRA);

    return result;
  }

  /*  This function is templatized because both GripperCommandResult and GripperCommandFeedback consist
      of the same fields yet have different types. Templates here act as a "duck typing" mechanism to avoid
      code duplication.
  */
  template<typename T>
  T registerStateToResultT(const GripperInput& input, const SModelGripperParams& params, uint8_t goal_pos)
  {
    T result;

    double dist_per_tick;
    if (params.gripper_mode_ == "wide_pinch")
      dist_per_tick = (params.max_rad_ - params.min_rad_) / 107;
    else // basic mode
      dist_per_tick = (params.max_rad_ - params.min_rad_) / 255;

    double eff_per_tick = (params.max_effort_ - params.min_effort_) / 255;

    result.position = input.gPOA * dist_per_tick + params.min_rad_;
    result.effort = input.gCUA * eff_per_tick + params.min_effort_;
    result.stalled = input.gIMC == 0x1 || input.gIMC == 0x2;
    result.reached_goal = input.gPOA == goal_pos;

    return result;
  }

  // Inline api-transformers to avoid confusion when reading the action_server source
  inline
  GripperCommandResult registerStateToResult(const GripperInput& input, const SModelGripperParams& params, uint8_t goal_pos)
  {
    return registerStateToResultT<GripperCommandResult>(input, params, goal_pos);
  }

  inline
  GripperCommandFeedback registerStateToFeedback(const GripperInput& input, const SModelGripperParams& params, uint8_t goal_pos)
  {
    return registerStateToResultT<GripperCommandFeedback>(input, params, goal_pos);
  }

} // end of anon namespace

namespace robotiq_action_server
{

SModelGripperActionServer::SModelGripperActionServer(const std::string& name, const SModelGripperParams& params)
  : nh_()
  , as_(nh_, name, false)
  , action_name_(name)
  , gripper_params_(params)
{
  as_.registerGoalCallback(boost::bind(&SModelGripperActionServer::goalCB, this));
  as_.registerPreemptCallback(boost::bind(&SModelGripperActionServer::preemptCB, this));

  state_sub_ = nh_.subscribe("input", 1, &SModelGripperActionServer::analysisCB, this);
  goal_pub_ = nh_.advertise<GripperOutput>("output", 1);
  as_.start();
}

void SModelGripperActionServer::goalCB()
{
  // Check to see if the gripper is in an active state where it can take goals
  if (current_reg_state_.gIMC != 0x3)
  {
    ROS_WARN("%s could not accept goal because the gripper is not yet active", action_name_.c_str());
    return;
  }

  GripperCommandGoal current_goal (*(as_.acceptNewGoal()));

  if (as_.isPreemptRequested())
  {
    as_.setPreempted();
  }

  try
  {
    if (current_goal.command.max_effort == 0.0){
      ROS_INFO("requested to move gripper with max_effort of 0.0. Defaulting to 70.0");
      current_goal.command.max_effort = 70.0;
    }
    goal_reg_state_ = goalToRegisterState(current_goal, gripper_params_);
    goal_pub_.publish(goal_reg_state_);
  }
  catch (BadArgumentsError& e)
  {
    ROS_INFO("%s No goal issued to gripper", action_name_.c_str());
  }
}

void SModelGripperActionServer::preemptCB()
{
  ROS_INFO("%s: Preempted", action_name_.c_str());
  as_.setPreempted();
}

void SModelGripperActionServer::analysisCB(const GripperInput::ConstPtr& msg)
{
  current_reg_state_ = *msg;
  // Check to see if the gripper is in its activated state
  if (current_reg_state_.gIMC != 0x3)
  {
    // Check to see if the gripper is active or if it has been asked to be active
    if (current_reg_state_.gIMC == 0x0 && goal_reg_state_.rACT != 0x1)
    {
      // If it hasn't been asked, active it
      issueActivation();
    }

    // Otherwise wait for the gripper to activate
    // TODO: If message delivery isn't guaranteed, then we may want to resend activate
    return;
  }
  if (!as_.isActive()) return;

  // Check for errors
  if (current_reg_state_.gFLT)
  {
    ROS_WARN("%s faulted with code: %x", action_name_.c_str(), current_reg_state_.gFLT);
    as_.setAborted(registerStateToResult(current_reg_state_,
                                         gripper_params_,
                                         goal_reg_state_.rPRA));
  }
  else if (current_reg_state_.gGTO && current_reg_state_.gIMC == 0x03 && current_reg_state_.gPRA == goal_reg_state_.rPRA && current_reg_state_.gSTA)
  {
    // If commanded to move and if at a goal state and if the position request matches the echo'd PR, we're
    // done with a moveregisterStateToR
    ROS_INFO("%s succeeded", action_name_.c_str());
    as_.setSucceeded(registerStateToResult(current_reg_state_,
                                           gripper_params_,
                                           goal_reg_state_.rPRA));
  }
  else
  {
    // Publish feedback
    as_.publishFeedback(registerStateToFeedback(current_reg_state_,
                                                gripper_params_,
                                                goal_reg_state_.rPRA));
  }
}

void SModelGripperActionServer::issueActivation()
{
  ROS_INFO("Activating gripper for gripper action server: %s", action_name_.c_str());
  GripperOutput out;
  out.rACT = 0x1;
  out.rSPA = 128;

  if (gripper_params_.gripper_mode_ == "wide_pinch")
    out.rMOD = 0x3;
  else
    out.rMOD = 0x0;

  // other params should be zero
  goal_reg_state_ = out;
  goal_pub_.publish(out);
}
} // end robotiq_action_server namespace
