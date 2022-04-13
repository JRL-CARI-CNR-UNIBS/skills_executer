#include <skills_executer/skills_exec.h>

namespace skills_executer
{

SkillsExec::SkillsExec(const ros::NodeHandle & n) : n_(n)
{
    twist_pub_ = n_.advertise<geometry_msgs::TwistStamped>("/target_cart_twist",1);

    skill_exec_srv_    = n_.advertiseService("/skills_exec/execute_skill", &SkillsExec::skillsExecution, this);
    start_config_clnt_ = n_.serviceClient<configuration_msgs::StartConfiguration>("/configuration_manager/start_configuration");
    ROS_WARN("Waiting for %s", start_config_clnt_.getService().c_str() );
    start_config_clnt_.waitForExistence();
    ROS_WARN("Connection ok");

    touch_action_ = std::make_shared<actionlib::SimpleActionClient<simple_touch_controller_msgs::simpleTouchAction>>("simple_touch", true);
    screw_accuracy_ = 0.1;
}

bool SkillsExec::skillsExecution(skills_executer_msgs::SkillExecution::Request  &req,
                                 skills_executer_msgs::SkillExecution::Response &res)
{
    ROS_INFO("Skill requested: %s/%s", req.action_name.c_str(), req.skill_name.c_str());
    std::string skill_type;
    if (!getParam(req.action_name, req.skill_name, "skill_type",   skill_type))
    {
        ROS_INFO("Something wrong with %s skill of %s action", req.skill_name.c_str(), req.action_name.c_str());
        return false;
    }
    ROS_INFO("Skill type: %s", skill_type.c_str());

    if ( !skill_type.compare(cart_move_type_) )
    {
        if ( !cartMove(req.action_name, req.skill_name) )
        {
            ROS_INFO("Something wrong with %s skill of %s action", req.skill_name.c_str(), req.action_name.c_str());
            return false;
        }
    }
    else if ( !skill_type.compare(simple_touch_type_) )
    {
        if ( !simpleTouch(req.action_name, req.skill_name) )
        {
            ROS_INFO("Something wrong with %s skill of %s action", req.skill_name.c_str(), req.action_name.c_str());
            return false;
        }
    }
    else
    {
        ROS_WARN("Function to execute %s type do not exist", skill_type.c_str());
    }
    return true;
}

bool SkillsExec::cartMove(const std::string &action_name, const std::string &skill_name)
{

    std::string        skill_type;
    std::string        frame_id;
    std::vector<float> twist_move;
    float              move_time;
    if (!getParam(action_name, skill_name, "skill_type",   skill_type))   return false;
    if (!getParam(action_name, skill_name, "frame_id",   frame_id))       return false;
    if (!getParam(action_name, skill_name, "twist_move", twist_move))     return false;
    if (!getParam(action_name, skill_name, "move_time",  move_time))      return false;
    ROS_INFO("%s-> frame_id: %s", skill_name.c_str(), frame_id.c_str());
    ROS_INFO("%s-> twist_move: [%lf,%lf,%lf,%lf,%lf,%lf]", skill_name.c_str(), twist_move.at(0), twist_move.at(1), twist_move.at(2), twist_move.at(3), twist_move.at(4), twist_move.at(5));
    ROS_INFO("%s-> move_time: %lf", skill_name.c_str(), move_time);

    geometry_msgs::TwistStamped twist_command;

    twist_command.header.frame_id=frame_id;
    twist_command.twist.linear.x=twist_move.at(0);
    twist_command.twist.linear.y=twist_move.at(1);
    twist_command.twist.linear.z=twist_move.at(2);
    twist_command.twist.angular.x=twist_move.at(3);
    twist_command.twist.angular.y=twist_move.at(4);
    twist_command.twist.angular.z=twist_move.at(5);

    ROS_WARN("Change configuration: %s", skill_type.c_str());

    changeConfig(skill_type);

    ROS_WARN("Execution Cart Move..");

    ros::Rate lp(100);

    double time = 0.0;

    while (time < move_time)
    {
      twist_command.header.stamp=ros::Time::now();
      twist_pub_.publish(twist_command);
      time = time+0.01;
      lp.sleep();
    }

    twist_command.twist.linear.x=0.0;
    twist_command.twist.linear.y=0.0;
    twist_command.twist.linear.z=0.0;
    twist_command.twist.angular.x=0.0;
    twist_command.twist.angular.y=0.0;
    twist_command.twist.angular.z=0.0;

    time=0.0;

    while (time<0.01)
    {
      twist_command.header.stamp=ros::Time::now();
      twist_pub_.publish(twist_command);

      ros::Duration(0.001).sleep();
      time=time+0.001;
    }

    changeConfig(watch_config_);
    return true;
}

bool SkillsExec::simpleTouch(const std::string &action_name, const std::string &skill_name)
{
    std::string         skill_type;
    std::string         target_frame;
    std::string         goal_twist_frame;
    std::vector<double> goal_twist;
    std::vector<double> target_wrench;
    std::vector<double> wrench_toll;
    std::vector<double> wrench_deadband;
    if (!getParam(action_name, skill_name, "skill_type",       skill_type) )      return false;
    if (!getParam(action_name, skill_name, "target_frame",     target_frame) )    return false;
    if (!getParam(action_name, skill_name, "goal_twist_frame", goal_twist_frame)) return false;
    if (!getParam(action_name, skill_name, "goal_twist",       goal_twist))       return false;
    if (!getParam(action_name, skill_name, "target_wrench",    target_wrench))    return false;
    if (!getParam(action_name, skill_name, "wrench_toll",      wrench_toll))      return false;
    if (!getParam(action_name, skill_name, "wrench_deadband",  wrench_deadband))  return false;
    ROS_INFO("%s-> target_frame: %s", skill_name.c_str(), target_frame.c_str());
    ROS_INFO("%s-> goal_twist_frame: %s", skill_name.c_str(), goal_twist_frame.c_str());
    ROS_INFO("%s-> goal_twist: [%lf,%lf,%lf,%lf,%lf,%lf]", skill_name.c_str(), goal_twist.at(0), goal_twist.at(1), goal_twist.at(2), goal_twist.at(3), goal_twist.at(4), goal_twist.at(5));
    if ( target_wrench.size() == 1 )
    {
        ROS_INFO("%s-> target_wrench: %lf", skill_name.c_str(), target_wrench.at(0));
    }
    else if ( target_wrench.size() == 6 )
    {
        ROS_INFO("%s-> target_wrench: [%lf,%lf,%lf,%lf,%lf,%lf]", skill_name.c_str(), target_wrench.at(0), target_wrench.at(1), target_wrench.at(2), target_wrench.at(3), target_wrench.at(4), target_wrench.at(5));
    }
    ROS_INFO("%s-> wrench_toll: [%lf,%lf,%lf,%lf,%lf,%lf]", skill_name.c_str(), wrench_toll.at(0), wrench_toll.at(1), wrench_toll.at(2), wrench_toll.at(3), wrench_toll.at(4), wrench_toll.at(5));
    ROS_INFO("%s-> wrench_deadband: [%lf,%lf,%lf,%lf,%lf,%lf]", skill_name.c_str(), wrench_deadband.at(0), wrench_deadband.at(1), wrench_deadband.at(2), wrench_deadband.at(3), wrench_deadband.at(4), wrench_deadband.at(5));

    ROS_WARN("Change configuration: %s", skill_type.c_str());

    changeConfig(skill_type);

    ROS_WARN("Execution Simple Touch..");

    simple_touch_controller_msgs::simpleTouchGoal goal_touch;
    goal_touch.target_wrench_frame = target_frame;
    goal_touch.goal_twist_frame = goal_twist_frame;
    goal_touch.goal_twist = goal_twist;
    goal_touch.target_wrench = target_wrench;
    goal_touch.wrench_toll = wrench_toll;
    goal_touch.wrench_deadband = wrench_deadband;

    touch_action_->waitForServer();
    touch_action_->sendGoalAndWait(goal_touch);

    changeConfig(watch_config_);

    return true;
}

bool SkillsExec::changeConfig(std::string config_name)
{
    configuration_msgs::StartConfiguration start_config_srv;
    start_config_srv.request.start_configuration = config_name;
    start_config_srv.request.strictness = 1;

    if (!start_config_clnt_.call(start_config_srv))
    {
      ROS_ERROR("Unable to call %s service to set controller %s",start_config_clnt_.getService().c_str(),config_name.c_str());
      return false;
    }

    if (!start_config_srv.response.ok)
    {
      ROS_ERROR("Error on service %s response", start_config_clnt_.getService().c_str());
      return false;
    }

    ROS_INFO("Controller %s started.",config_name.c_str());

//    ros::Duration(0.1).sleep();
    return true;
}

} // end namespace skills_executer
