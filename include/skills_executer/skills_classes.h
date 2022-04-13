#ifndef SKILLS_CLASSES_H
#define SKILLS_CLASSES_H

#include <ros/ros.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <skills_executer_msgs/SkillExecution.h>

std::vector<std::string> skillNames(const std::string& action_skill_name);

class SkillActionNode : public BT::SyncActionNode
{
public:
    SkillActionNode(const std::string& name);

    BT::NodeStatus tick() override;

private:
    ros::NodeHandle n_;
    ros::ServiceClient skill_exec_clnt_;
    std::string skill_name;
};


#endif // SKILLS_CLASSES_H
