#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <control_toolbox/pid.h>
#include <std_msgs/Float64MultiArray.h>

namespace solo_joint_controllers{

class JointGroupFeedforwardEffortController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
    public:
        JointGroupFeedforwardEffortController();
        
        bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &nh);
        
        void update(const ros::Time& time, const ros::Duration& period);

        void starting(const ros::Time& time);

        void setJointEffortCallback(const std_msgs::Float64MultiArrayConstPtr &msg);

    private:
        std::vector<hardware_interface::JointHandle> joints_hw;
        std::vector<control_toolbox::Pid> pid_controller;
        ros::Subscriber command_subscriber;
        std::vector<double> joint_feedforward_torques;
        std::vector<double> commanded_effort;
        std::vector<double> joint_position;
        std::vector<double> joint_velocity;
        std::vector<double> error;
        std::vector<double> error_dot;
        int joint_number;

};
PLUGINLIB_EXPORT_CLASS(solo_joint_controllers::JointGroupFeedforwardEffortController, controller_interface::ControllerBase);
}