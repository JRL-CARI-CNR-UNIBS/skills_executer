#ifndef BT_SKILLS_CLASSES_H
#define BT_SKILLS_CLASSES_H

#include <ros/ros.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <skills_executer_msgs/SkillExecution.h>

class SkillActionNode : public BT::SyncActionNode
{
public:
    SkillActionNode(const std::string& name);

    std::vector<std::string> skillNames(const std::string& action_skill_name);
    BT::NodeStatus tick() override;

private:
    ros::NodeHandle n_;
    ros::ServiceClient skill_exec_clnt_;
};

#endif // SKILLS_CLASSES_H
