#ifndef SKILLS_EXEC_H
#define SKILLS_EXEC_H

#include <ros/ros.h>
#include <skills_executer_msgs/SkillExecution.h>
#include <actionlib/client/simple_action_client.h>
#include <configuration_msgs/StartConfiguration.h>
#include <simple_touch_controller_msgs/SimpleTouchAction.h>
#include <simple_touch_controller_msgs/SimpleTouchActionGoal.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/JointState.h>
#include <manipulation_msgs/JobExecution.h>
#include <std_srvs/Trigger.h>
#include <relative_cartesian_controller_msgs/RelativeMoveAction.h>
#include <relative_cartesian_controller_msgs/RelativeMoveGoal.h>
#include <tf/tf.h>
#include <ur_dashboard_msgs/Load.h>
#include <std_srvs/Trigger.h>

namespace skills_executer
{

class SkillsExec
{
public:
    SkillsExec(const ros::NodeHandle & n);
    bool skillsExecution(skills_executer_msgs::SkillExecution::Request  &req,
                         skills_executer_msgs::SkillExecution::Response &res);

    bool changeConfig(std::string config_name);
    int urLoadProgram(const std::string &action_name, const std::string &skill_name);
    int parallel2fGripperMove  (const std::string &action_name, const std::string &skill_name);
    int robotiqGripperMove     (const std::string &action_name, const std::string &skill_name);
    int cartVel                (const std::string &action_name, const std::string &skill_name);
    int cartPos                (const std::string &action_name, const std::string &skill_name);
    int simpleTouch            (const std::string &action_name, const std::string &skill_name);
    int reset_ur10e_ft_sensor  ();

    template<typename T> bool getParam(const std::string &action_name, const std::string &skill_name, const std::string &param_name, T &param_value);

private:
    double screw_accuracy_;
    double max_screw_accuracy_ = 10.0;
    double pi_ = 3.14159265358979323846;
    std::string param_ns_ = "exec_params";
    ros::NodeHandle n_;
    ros::ServiceServer skill_exec_srv_;
    ros::ServiceClient start_config_clnt_;
    std::shared_ptr<actionlib::SimpleActionClient<simple_touch_controller_msgs::SimpleTouchAction>> touch_action_;
    std::shared_ptr<actionlib::SimpleActionClient<relative_cartesian_controller_msgs::RelativeMoveAction>> relative_move_action_;
    ros::Publisher twist_pub_;
    ros::Publisher gripper_move_pub_;

    std::string cart_vel_type_    = "cartesian_velocity";
    std::string cart_pos_type_    = "cartesian_position";
    std::string simple_touch_type_ = "simple_touch";
    std::string parallel_2f_gripper_move_type_ = "parallel_2f_gripper_move";
    std::string robotiq_gripper_move_type_ = "robotiq_gripper_move";
    std::string ur_load_program_ = "ur_load_program_";

    std::string watch_config_        = "watch";
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
