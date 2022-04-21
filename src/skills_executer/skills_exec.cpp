#include <skills_executer/skills_exec.h>

namespace skills_executer
{

SkillsExec::SkillsExec(const ros::NodeHandle & n) : n_(n)
{
    twist_pub_        = n_.advertise<geometry_msgs::TwistStamped>("/target_cart_twist",1);
    gripper_move_pub_ = n_.advertise<sensor_msgs::JointState>("/gripper/joint_target",1);

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
        ROS_WARN("No param for %s skill of %s action", req.skill_name.c_str(), req.action_name.c_str());
        res.result = skills_executer_msgs::SkillExecutionResponse::NoParam;
        return true;
    }
    ROS_INFO("Skill type: %s", skill_type.c_str());

    if ( !skill_type.compare(cart_move_type_) )
        return cartMove(req.action_name, req.skill_name);
    else if ( !skill_type.compare(simple_touch_type_) )
        return simpleTouch(req.action_name, req.skill_name);
    else if ( !skill_type.compare(gripper_move_type_) )
        return gripperMove(req.action_name, req.skill_name);
    else
        return skills_executer_msgs::SkillExecutionResponse::NoSkillType;
}

int SkillsExec::gripperMove(const std::string &action_name, const std::string &skill_name)
{
    double torque;
    double velocity;
    double position;
    bool set = false;
    if (!getParam(action_name, skill_name, "torque", torque))
    {
        torque = 0.0;
    }
    else
    {
        set = true;
    }
    if (!getParam(action_name, skill_name, "velocity", velocity))
    {
        velocity = 0.0;
    }
    else
    {
        set = true;
    }
    if (!getParam(action_name, skill_name, "position", position))
    {
        position = 0.0;
    }
    else
    {
        set = true;
    }

    if ( !set )
    {
        ROS_WARN("/%s/%s: no setted params", action_name.c_str(), skill_name.c_str());
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }

    ROS_INFO("%s-> torque: %lf",   skill_name.c_str(), torque);
    ROS_INFO("%s-> velocity: %lf", skill_name.c_str(), velocity);
    ROS_INFO("%s-> position: %lf", skill_name.c_str(), position);

    sensor_msgs::JointState gripper_move_msg;
    gripper_move_msg.name.push_back("");
    gripper_move_msg.position.push_back(position);
    gripper_move_msg.velocity.push_back(velocity);
    gripper_move_msg.effort.push_back(torque);

    gripper_move_msg.header.stamp=ros::Time::now();
    gripper_move_pub_.publish(gripper_move_msg);

//    Inserire la parte che controlla se la skill è andata a buon fine.

    return skills_executer_msgs::SkillExecutionResponse::Success;
}

int SkillsExec::cartMove(const std::string &action_name, const std::string &skill_name)
{

    std::string        skill_type;
    std::string        frame_id;
    std::vector<float> twist_move;
    float              move_time;
    if (!getParam(action_name, skill_name, "skill_type", skill_type))
    {
        ROS_WARN("The parameter %s/%s/skill_type is not setted", action_name.c_str(), skill_name.c_str());
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }
    if (!getParam(action_name, skill_name, "frame_id",   frame_id))
    {
        ROS_WARN("The parameter %s/%s/frame_id is not setted", action_name.c_str(), skill_name.c_str());
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }
    if (!getParam(action_name, skill_name, "twist_move", twist_move))
    {
        ROS_WARN("The parameter %s/%s/twist_move is not setted", action_name.c_str(), skill_name.c_str());
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }
    if (!getParam(action_name, skill_name, "move_time",  move_time))
    {
        ROS_WARN("The parameter %s/%s/move_time is not setted", action_name.c_str(), skill_name.c_str());
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }
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

    if ( !changeConfig(skill_type) )
        return skills_executer_msgs::SkillExecutionResponse::ProblemConfManager;

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

//    time=0.0;

//    while (time<0.01)
//    {
//      twist_command.header.stamp=ros::Time::now();
//      twist_pub_.publish(twist_command);

//      ros::Duration(0.001).sleep();
//      time=time+0.001;
//    }


//    Inserire la parte che controlla se la skill è andata a buon fine.

    twist_command.header.stamp=ros::Time::now();
    twist_pub_.publish(twist_command);

    if ( !changeConfig(watch_config_) )
        return skills_executer_msgs::SkillExecutionResponse::ProblemConfManager;

    return skills_executer_msgs::SkillExecutionResponse::Success;
}

int SkillsExec::simpleTouch(const std::string &action_name, const std::string &skill_name)
{
    std::string         skill_type;
    std::string         target_frame;
    std::string         goal_twist_frame;
    std::vector<double> goal_twist;
    std::vector<double> target_wrench;
    std::vector<double> wrench_toll;
    std::vector<double> wrench_deadband;
    if (!getParam(action_name, skill_name, "skill_type", skill_type) )
    {
        ROS_WARN("The parameter %s/%s/skill_type is not setted", action_name.c_str(), skill_name.c_str());
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }
    if (!getParam(action_name, skill_name, "target_frame", target_frame) )
    {
        ROS_WARN("The parameter %s/%s/target_frame is not setted", action_name.c_str(), skill_name.c_str());
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }
    if (!getParam(action_name, skill_name, "goal_twist_frame", goal_twist_frame))
    {
        ROS_WARN("The parameter %s/%s/goal_twist_frame is not setted", action_name.c_str(), skill_name.c_str());
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }
    if (!getParam(action_name, skill_name, "goal_twist", goal_twist))
    {
        ROS_WARN("The parameter %s/%s/goal_twist is not setted", action_name.c_str(), skill_name.c_str());
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }
    if (!getParam(action_name, skill_name, "target_wrench", target_wrench))
    {
        ROS_WARN("The parameter %s/%s/target_wrench is not setted", action_name.c_str(), skill_name.c_str());
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }
    if (!getParam(action_name, skill_name, "wrench_toll", wrench_toll))
    {
        ROS_WARN("The parameter %s/%s/wrench_toll is not setted", action_name.c_str(), skill_name.c_str());
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }
    if (!getParam(action_name, skill_name, "wrench_deadband", wrench_deadband))
    {
        ROS_WARN("The parameter %s/%s/wrench_deadband is not setted", action_name.c_str(), skill_name.c_str());
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }
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
    ROS_INFO("%s-> wrench_toll: [%lf,%lf,%lf,%lf,%lf,%lf]",     skill_name.c_str(), wrench_toll.at(0), wrench_toll.at(1), wrench_toll.at(2), wrench_toll.at(3), wrench_toll.at(4), wrench_toll.at(5));
    ROS_INFO("%s-> wrench_deadband: [%lf,%lf,%lf,%lf,%lf,%lf]", skill_name.c_str(), wrench_deadband.at(0), wrench_deadband.at(1), wrench_deadband.at(2), wrench_deadband.at(3), wrench_deadband.at(4), wrench_deadband.at(5));

    ROS_WARN("Change configuration: %s", skill_type.c_str());

    if ( !changeConfig(skill_type) )
        return skills_executer_msgs::SkillExecutionResponse::ProblemConfManager;

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

    if ( !changeConfig(watch_config_) )
        return skills_executer_msgs::SkillExecutionResponse::ProblemConfManager;

    return skills_executer_msgs::SkillExecutionResponse::Success;
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
