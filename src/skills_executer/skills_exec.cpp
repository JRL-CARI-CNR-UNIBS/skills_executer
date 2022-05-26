#include <skills_executer/skills_exec.h>

namespace skills_executer
{

SkillsExec::SkillsExec(const ros::NodeHandle & n) : n_(n)
{
    twist_pub_        = n_.advertise<geometry_msgs::TwistStamped>("/target_cart_twist",1);
    gripper_move_pub_ = n_.advertise<sensor_msgs::JointState>("/gripper/joint_target",1);

    skill_exec_srv_ = n_.advertiseService("/skills_exec/execute_skill", &SkillsExec::skillsExecution, this);

    start_config_clnt_ = n_.serviceClient<configuration_msgs::StartConfiguration>("/configuration_manager/start_configuration");
    ROS_WARN("Waiting for %s", start_config_clnt_.getService().c_str() );
    start_config_clnt_.waitForExistence();
    ROS_WARN("Connection ok");

    touch_action_         = std::make_shared<actionlib::SimpleActionClient<simple_touch_controller_msgs::SimpleTouchAction>>("simple_touch", true);
    relative_move_action_ = std::make_shared<actionlib::SimpleActionClient<relative_cartesian_controller_msgs::RelativeMoveAction>>("relative_move", true);
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

    if ( !skill_type.compare(cart_vel_type_) )
    {
        res.result = cartVel(req.action_name, req.skill_name);
        ROS_INFO("cartVel result: %d", res.result);
    }
    else if ( !skill_type.compare(cart_pos_type_) )
    {
        res.result = cartPos(req.action_name, req.skill_name);
        ROS_INFO("cartPos result: %d", res.result);
    }
    else if ( !skill_type.compare(simple_touch_type_) )
    {
        res.result = simpleTouch(req.action_name, req.skill_name);
        ROS_INFO("simpleTouch result: %d", res.result);
    }
    else if ( !skill_type.compare(parallel_2f_gripper_move_type_) )
    {
        res.result = parallel2fGripperMove(req.action_name, req.skill_name);
        ROS_INFO("parallel2fGripperMove result: %d", res.result);
    }
    else if ( !skill_type.compare(robotiq_gripper_move_type_) )
    {
        res.result = robotiqGripperMove(req.action_name, req.skill_name);
        ROS_INFO("parallel2fGripperMove result: %d", res.result);
    }
    else if ( !skill_type.compare(ur_load_program_) )
    {
        res.result = urLoadProgram(req.action_name, req.skill_name);
        ROS_INFO("urLoadProgram result: %d", res.result);
    }
    else if ( !skill_type.compare(ur_connect_dashboard_) )
    {
        res.result = urConnectDashboard(req.action_name, req.skill_name);
        ROS_INFO("urConnectDashboard result: %d", res.result);
    }
    else
    {
        ROS_INFO("result: NoSkillType");
        res.result = skills_executer_msgs::SkillExecutionResponse::NoSkillType;
    }

    ROS_INFO("Return true");
    return true;

}

int SkillsExec::urLoadProgram(const std::string &action_name, const std::string &skill_name)
{
    std::string hw_name;
    if (not getParam(action_name, skill_name, "ur_hw_name", hw_name))
    {
        ROS_WARN("The parameter %s/%s/ur_hw_name is not set", action_name.c_str(), skill_name.c_str());
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }
    std::vector<std::string> programs;
    if (not getParam(action_name, skill_name, "programs", programs))
    {
        ROS_WARN("The parameter %s/%s/programs is not set", action_name.c_str(), skill_name.c_str());
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }
    bool play;
    if (not getParam(action_name, skill_name, "play", play))
        play = true;
    double pause;

    ros::ServiceClient load_clnt = n_.serviceClient<ur_dashboard_msgs::Load>("/"+hw_name+"/dashboard/load_program");
    ros::ServiceClient is_loaded_clnt = n_.serviceClient<ur_dashboard_msgs::GetLoadedProgram>("/"+hw_name+"/dashboard/get_loaded_program");
    ros::ServiceClient status_clnt = n_.serviceClient<ur_dashboard_msgs::GetProgramState>("/"+hw_name+"/dashboard/program_state");
    ros::ServiceClient play_clnt = n_.serviceClient<std_srvs::Trigger>("/"+hw_name+"/dashboard/play");


    ROS_WARN("Waiting for %s", load_clnt.getService().c_str());
    load_clnt.waitForExistence();
    ROS_WARN("Waiting for %s", is_loaded_clnt.getService().c_str());
    is_loaded_clnt.waitForExistence();
    ROS_WARN("Waiting for %s", play_clnt.getService().c_str());
    play_clnt.waitForExistence();
    ROS_WARN("Waiting for %s", status_clnt.getService().c_str());
    status_clnt.waitForExistence();

    ROS_WARN("Connection ok");

    for(const std::string& program:programs)
    {
        ur_dashboard_msgs::Load load_srv;
        std_srvs::Trigger       play_srv;
        ur_dashboard_msgs::GetLoadedProgram is_loaded_srv;
        ur_dashboard_msgs::GetProgramState status_srv;

        load_srv.request.filename = program;

        if(not load_clnt.call(load_srv) )
        {
            ROS_ERROR("Unable to load program: %s",load_srv.request.filename.c_str());
            return skills_executer_msgs::SkillExecutionResponse::Error;
        }
        else
        {
            if(not load_srv.response.success)
            {
                ROS_ERROR("load_srv failed: %s",load_srv.request.filename.c_str());
                return skills_executer_msgs::SkillExecutionResponse::Fail;
            }
        }

        do
        {
            if(not is_loaded_clnt.call(is_loaded_srv) )
            {
                ROS_ERROR("Unable to get loaded program");
                return skills_executer_msgs::SkillExecutionResponse::Error;
            }
            else
            {
                if(not is_loaded_srv.response.success)
                {
                    ROS_ERROR("is_loaded_srv failed");
                    return skills_executer_msgs::SkillExecutionResponse::Fail;
                }
            }
            ros::Duration(0.01).sleep();

            ROS_ERROR_STREAM("programma carico: "<<is_loaded_srv.response.program_name);
            ROS_ERROR_STREAM("progrqamma voluto: "<<program);
            ROS_ERROR("---------");

        }while(is_loaded_srv.response.program_name.compare(program) != 0);

        ROS_WARN("Program %s loaded",load_srv.request.filename.c_str());

        ros::Duration(0.5).sleep();

        if(play)
        {
            if(not play_clnt.call(play_srv) )
            {
                ROS_ERROR("Unable to play program: %s",load_srv.request.filename.c_str());
                return skills_executer_msgs::SkillExecutionResponse::Error;
            }
            else
            {
                if(not play_srv.response.success)
                {
                    ROS_ERROR("play_srv failed: %s, message: %s",load_srv.request.filename.c_str(),play_srv.response.message);
                    return skills_executer_msgs::SkillExecutionResponse::Fail;
                }
            }

            ROS_WARN("Program %s launched",load_srv.request.filename.c_str());

            do
            {
                if(not status_clnt.call(status_srv) )
                {
                    ROS_ERROR("Unable to get status program: %s",load_srv.request.filename.c_str());
                    return skills_executer_msgs::SkillExecutionResponse::Error;
                }
                else
                {
                    if(not status_srv.response.success)
                    {
                        ROS_ERROR("status_srv failed: %s",load_srv.request.filename.c_str());
                        return skills_executer_msgs::SkillExecutionResponse::Fail;
                    }
                }
                ros::Duration(0.01).sleep();


                ROS_ERROR_STREAM("stato programma: "<<status_srv.response.state.state);
                ROS_ERROR_STREAM("stato voluto: "<<ur_dashboard_msgs::ProgramState::STOPPED);
                ROS_ERROR("---------");

            }while(status_srv.response.state.state.compare(ur_dashboard_msgs::ProgramState::STOPPED) != 0);
        }
    }
    return skills_executer_msgs::SkillExecutionResponse::Success;
}

int SkillsExec::urConnectDashboard(const std::string &action_name, const std::string &skill_name)
{
    std::string hw_name;
    if (not getParam(action_name, skill_name, "ur_hw_name", hw_name))
    {
        ROS_WARN("The parameter %s/%s/ur_hw_name is not set", action_name.c_str(), skill_name.c_str());
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }

    ros::ServiceClient connect_clnt = n_.serviceClient<std_srvs::Trigger>("/"+hw_name+"/dashboard/connect");

    ROS_WARN("Waiting for %s", connect_clnt.getService().c_str());
    connect_clnt.waitForExistence();

    ROS_WARN("Connection ok");

    std_srvs::Trigger connect_srv;

    if(not connect_clnt.call(connect_srv) )
    {
        ROS_ERROR("Unable to connect to dashboard");
        return skills_executer_msgs::SkillExecutionResponse::Error;
    }
    else
    {
       while(not connect_srv.response.success)
       {
           ROS_ERROR("trying to connectto dashboard");
           ros::Duration(0.01).sleep();
       }

//        if(not connect_srv.response.success)
//        {
//            ROS_ERROR("connect_srv failed");
//            return skills_executer_msgs::SkillExecutionResponse::Fail;
//        }
    }
    return skills_executer_msgs::SkillExecutionResponse::Success;
}


int SkillsExec::parallel2fGripperMove(const std::string &action_name, const std::string &skill_name)
{
    double torque;
    double velocity;
    double position;
    bool set = false;
    ROS_INFO("Read params");

    if (!getParam(action_name, skill_name, "torque", torque))
    {
        torque = 0.0;
    }
    else
    {
        ROS_INFO("Read torque: %lf", torque);
        set = true;
    }
    if (!getParam(action_name, skill_name, "velocity", velocity))
    {
        velocity = 0.0;
    }
    else
    {
        ROS_INFO("Read velocity: %lf", velocity);
        set = true;
    }
    if (!getParam(action_name, skill_name, "position", position))
    {
        position = 0.0;
    }
    else
    {
        ROS_INFO("Read position: %lf", position);
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

int SkillsExec::robotiqGripperMove(const std::string &action_name, const std::string &skill_name)
{
    ros::ServiceClient gripper_clnt = n_.serviceClient<manipulation_msgs::JobExecution>("/robotiq_gripper");
    ROS_WARN("Waiting for %s", gripper_clnt.getService().c_str() );
    gripper_clnt.waitForExistence();
    ROS_WARN("Connection ok");
    manipulation_msgs::JobExecution gripper_srv;

    if (!getParam(action_name, skill_name, "property_id", gripper_srv.request.property_id))
    {
        ROS_WARN("The parameter %s/%s/property_id is not setted", action_name.c_str(), skill_name.c_str());
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }
    if (!getParam(action_name, skill_name, "skill_name", gripper_srv.request.skill_name))
    {
        ROS_WARN("The parameter %s/%s/skill_name is not setted", action_name.c_str(), skill_name.c_str());
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }
    if (!getParam(action_name, skill_name, "tool_id", gripper_srv.request.tool_id))
    {
        ROS_WARN("The parameter %s/%s/tool_id is not setted", action_name.c_str(), skill_name.c_str());
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }

    if ( !gripper_clnt.call(gripper_srv) )
    {
        ROS_ERROR("Unable to move gripper to %s state",gripper_srv.request.property_id.c_str());
        return skills_executer_msgs::SkillExecutionResponse::Error;
    }

    return skills_executer_msgs::SkillExecutionResponse::Success;
}

int SkillsExec::cartVel(const std::string &action_name, const std::string &skill_name)
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

int SkillsExec::cartPos(const std::string &action_name, const std::string &skill_name)
{
    double rotZdeg, rotYdeg, rotXdeg, traXmm, traYmm, traZmm, target_angular_velocity, target_linear_velocity, vel;
    std::string skill_type;
    geometry_msgs::PoseStamped relative_pose;
    tf2::Quaternion quat;
    std::vector<double> position, orientation;

    if (!getParam(action_name, skill_name, "skill_type", skill_type))
    {
        ROS_WARN("The parameter %s/%s/skill_type is not setted", action_name.c_str(), skill_name.c_str());
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }
    if( getParam(action_name, skill_name, "rotZdeg", rotZdeg) )
    {
        double angle = rotZdeg*pi_/180;
        quat.setRPY(0,0,angle);
        //        relative_pose.pose.orientation=tf::createQuaternionFromRPY(0.0,0.0,angle);
        relative_pose.pose.orientation.x = quat.getX();
        relative_pose.pose.orientation.y = quat.getY();
        relative_pose.pose.orientation.z = quat.getZ();
        relative_pose.pose.orientation.w = quat.getW();
        relative_pose.pose.position.x = 0.0;
        relative_pose.pose.position.y = 0.0;
        relative_pose.pose.position.z = 0.0;
        ROS_INFO("Readed rotZdeg: %lf", rotZdeg);
        ROS_INFO("Position: [%lf,%lf,%lf]", relative_pose.pose.position.x, relative_pose.pose.position.y, relative_pose.pose.position.z);
        ROS_INFO("Orientation: [%lf,%lf,%lf,%lf]", relative_pose.pose.orientation.x, relative_pose.pose.orientation.y, relative_pose.pose.orientation.z, relative_pose.pose.orientation.w);
    }
    else if( getParam(action_name, skill_name, "rotYdeg", rotYdeg) )
    {
        double angle = rotYdeg*pi_/180;
        quat.setRPY(0,angle,0);
        //        relative_pose.pose.orientation=tf::createQuaternionFromRPY(0.0,angle,0.0);
        relative_pose.pose.orientation.x = quat.getX();
        relative_pose.pose.orientation.y = quat.getY();
        relative_pose.pose.orientation.z = quat.getZ();
        relative_pose.pose.orientation.w = quat.getW();
        relative_pose.pose.position.x = 0.0;
        relative_pose.pose.position.y = 0.0;
        relative_pose.pose.position.z = 0.0;
        ROS_INFO("Readed rotYdeg: %lf", rotYdeg);
        ROS_INFO("Position: [%lf,%lf,%lf]", relative_pose.pose.position.x, relative_pose.pose.position.y, relative_pose.pose.position.z);
        ROS_INFO("Orientation: [%lf,%lf,%lf,%lf]", relative_pose.pose.orientation.x, relative_pose.pose.orientation.y, relative_pose.pose.orientation.z, relative_pose.pose.orientation.w);
    }
    else if( getParam(action_name, skill_name, "rotXdeg", rotXdeg) )
    {
        double angle = rotXdeg*pi_/180;
        quat.setRPY(angle,0,0);
        //        relative_pose.pose.orientation=tf::createQuaternionFromRPY(angle,0.0,0.0);
        relative_pose.pose.orientation.x = quat.getX();
        relative_pose.pose.orientation.y = quat.getY();
        relative_pose.pose.orientation.z = quat.getZ();
        relative_pose.pose.orientation.w = quat.getW();
        relative_pose.pose.position.x = 0.0;
        relative_pose.pose.position.y = 0.0;
        relative_pose.pose.position.z = 0.0;
        ROS_INFO("Readed rotXdeg: %lf", rotXdeg);
        ROS_INFO("Position: [%lf,%lf,%lf]", relative_pose.pose.position.x, relative_pose.pose.position.y, relative_pose.pose.position.z);
        ROS_INFO("Orientation: [%lf,%lf,%lf,%lf]", relative_pose.pose.orientation.x, relative_pose.pose.orientation.y, relative_pose.pose.orientation.z, relative_pose.pose.orientation.w);
    }
    else if( getParam(action_name, skill_name, "traXmm", traXmm) )
    {
        relative_pose.pose.position.x = traXmm/1000;
        relative_pose.pose.position.y = 0.0;
        relative_pose.pose.position.z = 0.0;
        relative_pose.pose.orientation.x = 0.0;
        relative_pose.pose.orientation.y = 0.0;
        relative_pose.pose.orientation.z = 0.0;
        relative_pose.pose.orientation.w = 1.0;
        ROS_INFO("Readed traXmm: %lf", traXmm);
        ROS_INFO("Position: [%lf,%lf,%lf]", relative_pose.pose.position.x, relative_pose.pose.position.y, relative_pose.pose.position.z);
        ROS_INFO("Orientation: [%lf,%lf,%lf,%lf]", relative_pose.pose.orientation.x, relative_pose.pose.orientation.y, relative_pose.pose.orientation.z, relative_pose.pose.orientation.w);
    }
    else if( getParam(action_name, skill_name, "traYmm", traYmm) )
    {
        relative_pose.pose.position.x = 0.0;
        relative_pose.pose.position.y = traYmm/1000;
        relative_pose.pose.position.z = 0.0;
        relative_pose.pose.orientation.x = 0.0;
        relative_pose.pose.orientation.y = 0.0;
        relative_pose.pose.orientation.z = 0.0;
        relative_pose.pose.orientation.w = 1.0;
        ROS_INFO("Readed traYmm: %lf", traYmm);
        ROS_INFO("Position: [%lf,%lf,%lf]", relative_pose.pose.position.x, relative_pose.pose.position.y, relative_pose.pose.position.z);
        ROS_INFO("Orientation: [%lf,%lf,%lf,%lf]", relative_pose.pose.orientation.x, relative_pose.pose.orientation.y, relative_pose.pose.orientation.z, relative_pose.pose.orientation.w);
    }
    else if( getParam(action_name, skill_name, "traZmm", traZmm) )
    {
        relative_pose.pose.position.x = 0.0;
        relative_pose.pose.position.y = 0.0;
        relative_pose.pose.position.z = traZmm/1000;
        relative_pose.pose.orientation.x = 0.0;
        relative_pose.pose.orientation.y = 0.0;
        relative_pose.pose.orientation.z = 0.0;
        relative_pose.pose.orientation.w = 1.0;
        ROS_INFO("Readed traZmm: %lf", traZmm);
        ROS_INFO("Position: [%lf,%lf,%lf]", relative_pose.pose.position.x, relative_pose.pose.position.y, relative_pose.pose.position.z);
        ROS_INFO("Orientation: [%lf,%lf,%lf,%lf]", relative_pose.pose.orientation.x, relative_pose.pose.orientation.y, relative_pose.pose.orientation.z, relative_pose.pose.orientation.w);
    }
    else
    {
        if ( !getParam(action_name, skill_name, "position", position) )
        {
            ROS_WARN("The parameter %s/%s/position is not setted", action_name.c_str(), skill_name.c_str());
            return skills_executer_msgs::SkillExecutionResponse::NoParam;
        }
        else
        {
            if ( !position.size() == 3 )
            {
                ROS_WARN("The position size is not 3");
                return skills_executer_msgs::SkillExecutionResponse::NoParam;
            }
            relative_pose.pose.position.x = position.at(0);
            relative_pose.pose.position.y = position.at(1);
            relative_pose.pose.position.z = position.at(2);
            ROS_INFO("Readed position: [%lf,%lf,%lf]", relative_pose.pose.position.x, relative_pose.pose.position.y, relative_pose.pose.position.z);
        }
        if ( !getParam(action_name, skill_name, "orientation", orientation) )
        {
            ROS_WARN("The parameter %s/%s/orientation is not setted", action_name.c_str(), skill_name.c_str());
            return skills_executer_msgs::SkillExecutionResponse::NoParam;
        }
        else
        {
            if ( !orientation.size() == 4 )
            {
                ROS_WARN("The orientation size is not 4");
                return skills_executer_msgs::SkillExecutionResponse::NoParam;
            }
            relative_pose.pose.orientation.x = orientation.at(0);
            relative_pose.pose.orientation.y = orientation.at(1);
            relative_pose.pose.orientation.z = orientation.at(2);
            relative_pose.pose.orientation.w = orientation.at(3);
            ROS_INFO("Readed orientation: [%lf,%lf,%lf,%lf]", relative_pose.pose.orientation.x, relative_pose.pose.orientation.y, relative_pose.pose.orientation.z, relative_pose.pose.orientation.w);
        }
    }
    if ( !getParam(action_name, skill_name, "frame", relative_pose.header.frame_id) )
    {
        ROS_WARN("The parameter %s/%s/frame is not setted", action_name.c_str(), skill_name.c_str());
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }
    if ( !getParam(action_name, skill_name, "linear_velocity_m_s", target_linear_velocity) )
    {
        if ( !getParam(action_name, skill_name, "linear_velocity_mm_s", vel) )
        {
            ROS_WARN("The parameter %s/%s/linear_velocity_m_s or linear_velocity_mm_s is not setted", action_name.c_str(), skill_name.c_str());
            ROS_WARN("The default value is linear_velocity_m_s = 0.250 m/s");
            target_linear_velocity = 0.250;
        }
        else
        {
            target_linear_velocity = vel / 1000;
            ROS_INFO("Readed linear_velocity_mm_s: %lf", vel);
        }
    }
    else
    {
        ROS_INFO("Readed linear_velocity_m_s: %lf", target_linear_velocity);
    }
    if ( !getParam(action_name, skill_name, "angular_velocity_rad_s", target_angular_velocity) )
    {
        if ( !getParam(action_name, skill_name, "angular_velocity_deg_s", vel) )
        {
            ROS_WARN("The parameter %s/%s/angular_velocity_rad_s or angular_velocity_deg_s is not setted", action_name.c_str(), skill_name.c_str());
            ROS_WARN("The default value is angular_velocity_deg_s = 30 deg/sec");
            target_angular_velocity = 30*pi_/180;
        }
        else
        {
            target_angular_velocity = vel*pi_/180;
            ROS_INFO("Readed angular_velocity_deg_s: %lf", vel);
            ROS_INFO("Angular_velocity_rad_s: %lf", target_angular_velocity);
        }
    }
    else
    {
        ROS_INFO("Readed angular_velocity_rad_s: %lf", target_angular_velocity);
    }

    if ( !changeConfig(skill_type) )
        return skills_executer_msgs::SkillExecutionResponse::ProblemConfManager;


    relative_cartesian_controller_msgs::RelativeMoveGoal rel_move_goal;
    rel_move_goal.target_angular_velocity = target_angular_velocity;
    rel_move_goal.target_linear_velocity = target_linear_velocity;
    rel_move_goal.relative_pose = relative_pose;

    ROS_INFO("Goal:");
    ROS_INFO("Frame: %s", rel_move_goal.relative_pose.header.frame_id.c_str());
    ROS_INFO("Position: [%lf,%lf,%lf]", rel_move_goal.relative_pose.pose.position.x, rel_move_goal.relative_pose.pose.position.y, rel_move_goal.relative_pose.pose.position.z);
    ROS_INFO("Orientation: [%lf,%lf,%lf,%lf]", rel_move_goal.relative_pose.pose.orientation.x, rel_move_goal.relative_pose.pose.orientation.y, rel_move_goal.relative_pose.pose.orientation.z, rel_move_goal.relative_pose.pose.orientation.w);
    ROS_INFO("Velocity: lin %lf, rot %lf", rel_move_goal.target_linear_velocity, rel_move_goal.target_angular_velocity);
    relative_move_action_->waitForServer();
    relative_move_action_->sendGoalAndWait(rel_move_goal);

    if ( !changeConfig(watch_config_) )
        return skills_executer_msgs::SkillExecutionResponse::ProblemConfManager;

    return skills_executer_msgs::SkillExecutionResponse::Success;
}

int SkillsExec::simpleTouch(const std::string &action_name, const std::string &skill_name)
{
    std::string         skill_type;
    std::string         goal_twist_frame;
    std::vector<double> goal_twist;

    double target_force;
    bool   relative_target;
    double release;
    int    release_condition;

    int version;

    if (!getParam(action_name, skill_name, "skill_type", skill_type) )
    {
        ROS_WARN("The parameter %s/%s/skill_type is not setted", action_name.c_str(), skill_name.c_str());
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }
    ROS_INFO("/%s/%s-> skill_type: %s", action_name.c_str(), skill_name.c_str(), skill_type.c_str());
    if (!getParam(action_name, skill_name, "goal_twist_frame", goal_twist_frame))
    {
        ROS_WARN("The parameter %s/%s/goal_twist_frame is not setted", action_name.c_str(), skill_name.c_str());
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }
    ROS_INFO("/%s/%s-> goal_twist_frame: %s", action_name.c_str(), skill_name.c_str(), goal_twist_frame.c_str());
    if (!getParam(action_name, skill_name, "goal_twist", goal_twist))
    {
        ROS_WARN("The parameter %s/%s/goal_twist is not setted", action_name.c_str(), skill_name.c_str());
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }
    if ( goal_twist.size() == 1)
    {
        ROS_INFO("/%s/%s-> goal_twist: %lf", action_name.c_str(), skill_name.c_str(), goal_twist.at(0));
    }
    else if ( goal_twist.size() == 6 )
    {
        ROS_INFO("/%s/%s-> goal_twist: [%lf,%lf,%lf,%lf,%lf,%lf]", action_name.c_str(), skill_name.c_str(), goal_twist.at(0), goal_twist.at(1), goal_twist.at(2), goal_twist.at(3), goal_twist.at(4), goal_twist.at(5));
    }
    else
    {
        ROS_ERROR("/%s/%s-> goal_twist has wrong size", action_name.c_str(), skill_name.c_str());
    }

    if (!getParam(action_name, skill_name, "target_force", target_force))
    {
        ROS_WARN("The parameter %s/%s/target_wrench or target_force are not setted", action_name.c_str(), skill_name.c_str());
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }
    ROS_INFO("/%s/%s-> target_force: %lf", action_name.c_str(), skill_name.c_str(), target_force);

    if (!getParam(action_name, skill_name, "release", release))
    {
        ROS_WARN("The parameter %s/%s/release is not setted", action_name.c_str(), skill_name.c_str());
    }
    else
    {
        ROS_INFO("/%s/%s-> release: %lf", action_name.c_str(), skill_name.c_str(), release);
    }

    if (!getParam(action_name, skill_name, "release_condition", release_condition))
    {
        ROS_WARN("The parameter %s/%s/release_condition is not setted", action_name.c_str(), skill_name.c_str());
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }
    else
    {
        ROS_INFO("/%s/%s-> release_condition: %d", action_name.c_str(), skill_name.c_str(), release_condition);
    }

    if (!getParam(action_name, skill_name, "relative_target", relative_target))
    {
        ROS_WARN("The parameter %s/%s/relative_target is not setted", action_name.c_str(), skill_name.c_str());
        return skills_executer_msgs::SkillExecutionResponse::NoParam;
    }
    if (relative_target)
        ROS_INFO("/%s/%s-> relative_target: true", action_name.c_str(), skill_name.c_str());
    else
        ROS_INFO("/%s/%s-> relative_target: false", action_name.c_str(), skill_name.c_str());

    ROS_WARN("Change configuration: %s", skill_type.c_str());

    if ( !changeConfig(skill_type) )
        return skills_executer_msgs::SkillExecutionResponse::ProblemConfManager;

    ROS_WARN("Execution Simple Touch..");

    simple_touch_controller_msgs::SimpleTouchGoal goal_touch;

    goal_touch.goal_twist = goal_twist;
    goal_touch.goal_twist_frame = goal_twist_frame;
    goal_touch.relative_target = relative_target;
    goal_touch.release = release;
    goal_touch.release_condition = release_condition;
    goal_touch.target_force = target_force;

    touch_action_->waitForServer();
    touch_action_->sendGoalAndWait(goal_touch);

    if ( !changeConfig(watch_config_) )
        return skills_executer_msgs::SkillExecutionResponse::ProblemConfManager;

    return skills_executer_msgs::SkillExecutionResponse::Success;
}

int touchBoard(const std::string &action_name, const std::string &skill_name)
{
//    std::string reference_frame;
//    if (!getParam(action_name, skill_name, "reference_frame", reference_frame))
//    {
//        ROS_WARN("The parameter %s/%s/reference_frame is not setted", action_name.c_str(), skill_name.c_str());
//        return skills_executer_msgs::SkillExecutionResponse::NoParam;
//    }

//    double goal_twist;
//    if (!getParam(action_name, skill_name, "goal_twist", goal_twist))
//    {
//        ROS_WARN("The parameter %s/%s/velocity is not setted", action_name.c_str(), skill_name.c_str());
//        return skills_executer_msgs::SkillExecutionResponse::NoParam;
//    }

//    double translation;
//    if (!getParam(action_name, skill_name, "velocity", velocity_touch))
//    {
//        ROS_WARN("The parameter %s/%s/velocity is not setted", action_name.c_str(), skill_name.c_str());
//        return skills_executer_msgs::SkillExecutionResponse::NoParam;
//    }

//    double target_force;
//    if (!getParam(action_name, skill_name, "target_force", target_force))
//    {
//        ROS_WARN("The parameter %s/%s/target_force is not setted", action_name.c_str(), skill_name.c_str());
//        return skills_executer_msgs::SkillExecutionResponse::NoParam;
//    }

//    double release;
//    if (!getParam(action_name, skill_name, "release", release))
//    {
//        ROS_WARN("The parameter %s/%s/release is not setted", action_name.c_str(), skill_name.c_str());
//        return skills_executer_msgs::SkillExecutionResponse::NoParam;
//    }

//    int release_condition;
//    if (!getParam(action_name, skill_name, "release_condition", release_condition))
//    {
//        ROS_WARN("The parameter %s/%s/release_condition is not setted", action_name.c_str(), skill_name.c_str());
//        return skills_executer_msgs::SkillExecutionResponse::NoParam;
//    }

//    if ( !changeConfig(simple_touch_type_) )
//        return skills_executer_msgs::SkillExecutionResponse::ProblemConfManager;

//    ROS_WARN("Execution Simple Touch..");

//    simple_touch_controller_msgs::SimpleTouchGoal goal_touch;

//    goal_touch.goal_twist = goal_twist;
//    goal_touch.goal_twist_frame = reference_frame;
//    goal_touch.relative_target = true;
//    goal_touch.release = release;
//    goal_touch.release_condition = release_condition;
//    goal_touch.target_force = target_force;

//    touch_action_->waitForServer();
//    touch_action_->sendGoalAndWait(goal_touch);

//    //SALVA POSA

//    if ( !changeConfig(watch_config_) )
//        return skills_executer_msgs::SkillExecutionResponse::ProblemConfManager;


//    if ( !changeConfig(cart_pos_type_) )
//        return skills_executer_msgs::SkillExecutionResponse::ProblemConfManager;


//    if ( !changeConfig(simple_touch_type_) )
//        return skills_executer_msgs::SkillExecutionResponse::ProblemConfManager;

//    ROS_WARN("Execution Simple Touch..");

//    simple_touch_controller_msgs::SimpleTouchGoal goal_touch;

//    goal_touch.goal_twist = goal_twist;
//    goal_touch.goal_twist_frame = reference_frame;
//    goal_touch.relative_target = true;
//    goal_touch.release = release;
//    goal_touch.release_condition = release_condition;
//    goal_touch.target_force = target_force;

//    touch_action_->waitForServer();
//    touch_action_->sendGoalAndWait(goal_touch);


//    if ( !changeConfig(watch_config_) )
//        return skills_executer_msgs::SkillExecutionResponse::ProblemConfManager;









//    return skills_executer_msgs::SkillExecutionResponse::Success;

    return 0;
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

int SkillsExec::reset_ur10e_ft_sensor()
{
    ros::ServiceClient reset_force_sensor_clnt = n_.serviceClient<std_srvs::Trigger>("/ur10e_hw/zero_ftsensor");
    reset_force_sensor_clnt.waitForExistence();
    std_srvs::Trigger zero_srv;
    if ( !reset_force_sensor_clnt.call(zero_srv) )
    {
        ROS_ERROR("Unable to reset force sensor");
        return skills_executer_msgs::SkillExecutionResponse::Error;
    }
    return skills_executer_msgs::SkillExecutionResponse::Success;
}


} // end namespace skills_executer
