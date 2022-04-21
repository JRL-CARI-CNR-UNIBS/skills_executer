#ifndef SKILLS_EXEC_H
#define SKILLS_EXEC_H

#include <ros/ros.h>
#include <skills_executer_msgs/SkillExecution.h>
#include <actionlib/client/simple_action_client.h>
#include <configuration_msgs/StartConfiguration.h>
#include <simple_touch_controller_msgs/simpleTouchAction.h>
#include <simple_touch_controller_msgs/simpleTouchGoal.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/JointState.h>

namespace skills_executer
{

class SkillsExec
{
public:
    SkillsExec(const ros::NodeHandle & n);
    bool skillsExecution(skills_executer_msgs::SkillExecution::Request  &req,
                         skills_executer_msgs::SkillExecution::Response &res);

    bool changeConfig(std::string config_name);
    int gripperMove(const std::string &action_name, const std::string &skill_name);
    int cartMove   (const std::string &action_name, const std::string &skill_name);
    int simpleTouch(const std::string &action_name, const std::string &skill_name);

    template<typename T> bool getParam(const std::string &action_name, const std::string &skill_name, const std::string &param_name, T &param_value);

private:
    double screw_accuracy_;
    double max_screw_accuracy_ = 10.0;
    std::string param_ns_ = "exec_params";
    ros::NodeHandle n_;
    ros::ServiceServer skill_exec_srv_;
    ros::ServiceClient start_config_clnt_;
    std::shared_ptr<actionlib::SimpleActionClient<simple_touch_controller_msgs::simpleTouchAction>> touch_action_;
    ros::Publisher twist_pub_;
    ros::Publisher gripper_move_pub_;

    std::string cart_move_type_    = "cart_move";
    std::string simple_touch_type_ = "simple_touch";
    std::string gripper_move_type_ = "gripper_move";

    std::string watch_config_        = "watch";
    std::string cart_move_config_    = "cartesian_velocity";
    std::string simple_touch_config_ = "simple_touch";
};

template<typename T>
inline bool SkillsExec::getParam(const std::string &action_name, const std::string &skill_name, const std::string &param_name, T &param_value)
{
    std::string param_str = "/"+param_ns_+"/"+action_name+"/"+skill_name+"/"+param_name;
    if ( !n_.getParam(param_str, param_value) )
    {
        ROS_WARN("%s not set", param_str.c_str());
        return false;
    }
    return true;
}


} // end namespace skills_executer_msgs

#endif // SKILLS_EXEC_H
