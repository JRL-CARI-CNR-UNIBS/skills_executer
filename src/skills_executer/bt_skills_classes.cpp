#include <skills_executer/bt_skills_classes.h>

//std::vector<std::string> SkillActionNode::skillNames(const std::string& action_skill_name)
std::vector<std::string> bt_skills_classes::skillNames(const std::string& action_skill_name)
{
    std::size_t found = action_skill_name.find("/");
    std::vector<std::string> names;
    if (found==std::string::npos)
    {
        ROS_WARN("The name of action don't respect the standard");
        return names;
    }
    std::string action_name = action_skill_name;
    action_name.erase(action_name.begin()+found, action_name.end());
    std::string skill_name = action_skill_name;
    skill_name.erase(skill_name.begin(), skill_name.begin()+found+1);

    ROS_INFO("action_skill_name: %s",action_skill_name.c_str());
    ROS_INFO("action_name: %s",action_name.c_str());
    ROS_INFO("skill_name: %s",skill_name.c_str());

    names.push_back(action_name);
    names.push_back(skill_name);
    return names;
}

SkillActionNode::SkillActionNode(const std::string &name) : BT::SyncActionNode(name, {})
{
    skill_exec_clnt_ = n_.serviceClient<skills_executer_msgs::SkillExecution>("/skills_exec/execute_skill");
    ROS_WARN("Waiting for %s", skill_exec_clnt_.getService().c_str() );
    skill_exec_clnt_.waitForExistence();
    ROS_WARN("Connection ok");
}

BT::NodeStatus SkillActionNode::tick()
{
    std::vector<std::string> names = bt_skills_classes::skillNames(name());
    ROS_INFO("action_name: %s",names.at(0).c_str());
    ROS_INFO("skill_name: %s",names.at(1).c_str());
    if (names.empty())
    {
        ROS_WARN("The name of action don't respect the standard");
        return BT::NodeStatus::FAILURE;
    }
    skills_executer_msgs::SkillExecution skill_exec_srv;
    skill_exec_srv.request.action_name = names.at(0);
    skill_exec_srv.request.skill_name = names.at(1);
//        mettere qua tutto il necessario

    if (!skill_exec_clnt_.call(skill_exec_srv))
    {
        ROS_ERROR("Fail to call service: %s", skill_exec_clnt_.getService().c_str());
        ROS_ERROR("action_name: %s, skill_name: %s", skill_exec_srv.request.action_name.c_str(), skill_exec_srv.request.skill_name.c_str());
        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::SUCCESS;
}

//TaskNode::TaskNode(const std::string &name) : BT::SyncActionNode(name, {})
//{
//    skill_exec_clnt_ = n_.serviceClient<skills_executer_msgs::SkillExecution>("/skills_exec/execute_skill");
//    ROS_WARN("Waiting for %s", skill_exec_clnt_.getService().c_str() );
//    skill_exec_clnt_.waitForExistence();
//    ROS_WARN("Connection ok");
//}

//BT::NodeStatus TaskNode::tick()
//{
//    std::string task_param = "task/";
//    task_param.append(name());
//    std::vector<std::string> action_skill_names;
//    if ( !n_.getParam(task_param, action_skill_names) )
//    {
//        ROS_WARN("%s not set", task_param.c_str());
//        return BT::NodeStatus::FAILURE;
//    }

//    for (std::string action_skill_name : action_skill_names)
//    {
//        std::vector<std::string> names = skillNames(action_skill_name);
//        ROS_INFO("action_name: %s",names.at(0).c_str());
//        ROS_INFO("skill_name: %s",names.at(1).c_str());
//        if (names.empty())
//        {
//            ROS_WARN("The name of action don't respect the standard");
//            return BT::NodeStatus::FAILURE;
//        }
//        skills_executer_msgs::SkillExecution skill_exec_srv;
//        skill_exec_srv.request.action_name = names.at(0);
//        skill_exec_srv.request.skill_name = names.at(1);
//    //        mettere qua tutto il necessario

//        if (!skill_exec_clnt_.call(skill_exec_srv))
//        {
//            ROS_ERROR("Fail to call service: %s", skill_exec_clnt_.getService().c_str());
//            return BT::NodeStatus::FAILURE;
//        }
//    }

//    return BT::NodeStatus::SUCCESS;
//}
