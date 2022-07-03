#include <solo_joint_controllers/joint_group_feedforward_effort_controller.h>


namespace solo_joint_controllers{

    JointGroupFeedforwardEffortController::JointGroupFeedforwardEffortController(){

    }

    bool JointGroupFeedforwardEffortController::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &nh){

        // Get Yaml parameters
        XmlRpc::XmlRpcValue joints_name, pid_gains;
        nh.getParam("joints", joints_name);
        nh.getParam("pid", pid_gains);

        // Resize data
        joint_number = joints_name.size();
        pid_controller.resize(joint_number);
        joint_feedforward_torques.resize(joint_number);
        commanded_effort.resize(joint_number);
        joint_position.resize(joint_number);
        joint_velocity.resize(joint_number);
        error.resize(joint_number);
        error_dot.resize(joint_number);
        joints_hw.resize(joint_number);
        for (int i = 0; i < joint_number; i++){
            joints_hw[i] = hw->getHandle(joints_name[i]);
            pid_controller[i].initPid(pid_gains["p"], pid_gains["i"], pid_gains["d"], 10000, -10000); 
        }
        
        command_subscriber = nh.subscribe<std_msgs::Float64MultiArray>("command", 1, &JointGroupFeedforwardEffortController::setJointEffortCallback, this);

        return true;
    }

    void JointGroupFeedforwardEffortController::starting(const ros::Time& time) { 
        for (int i = 0; i < joint_number; i++){
            pid_controller[i].reset();
        }
    }

    void JointGroupFeedforwardEffortController::update(const ros::Time& time, const ros::Duration& period){
        for (int i = 0; i < joint_number; i++){
            // Get Joint State
            joint_position[i] = joints_hw[i].getPosition();
            joint_velocity[i] = joints_hw[i].getVelocity();

            // Compute Error
            error[i] = 0;
            error_dot[i] = -joint_velocity[i];

            // Compute Effort
            commanded_effort[i] = joint_feedforward_torques[i] + pid_controller[i].computeCommand(error[i], error_dot[i], period);
            joints_hw[i].setCommand(commanded_effort[i]);
        }
    }

    void JointGroupFeedforwardEffortController::setJointEffortCallback(const std_msgs::Float64MultiArrayConstPtr &msg){
        if (msg->layout.dim[0].size != joint_number){
            ROS_WARN("Wrong dimension at command");
        }
        else{
            // Update Feedforward Torque
            for (int i = 0; i < joint_number; i++){
                joint_feedforward_torques[i] = msg->data[i];
            }
        }
    }

}