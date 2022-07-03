#ifndef SOLO8_CENTROID_MPC_H
#define SOLO8_CENTROID_MPC_H

// ROS
#include "ros/ros.h"


// Eigen
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SVD>
#include <Eigen/QR>  
#include <unsupported/Eigen/MatrixFunctions>
#include <unsupported/Eigen/KroneckerProduct>

// Messages
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>

#include "tf/tf.h"

// OSQP
#include "osqp.h"
#include <OsqpEigen/OsqpEigen.h>
#include <OsqpEigen/Solver.hpp>


class Solo8CentroidMPC
{

    public:
        Solo8CentroidMPC(ros::NodeHandle& nh_);
        ~Solo8CentroidMPC();

        void jointStateCallback(const sensor_msgs::JointState::ConstPtr&);
        void gazeboModelStateCallback(const gazebo_msgs::ModelStates::ConstPtr&);
        void comRefPoseCallback(const geometry_msgs::PoseStamped::ConstPtr&);
        void updateContactJacobians();
        void pubTorqueCmd();
        void pubGrfCmd();
        void updateRotationMatrix();
        void updateStateSpace();
        void updateStateSpaceLinear();
        void updatePredictionModel();
        void initMatrices();
        void updateOptMatrices();
        void updateConstMatrices();
        int osqpMpcSetup();
        int osqpRcfSetup();
        void timeControllerCallback(const ros::TimerEvent& event);
        void initRcfMatrices();
        void updateRcfMatrices();
        int osqpRcfSolve();
        int osqpMpcSolve();
        void updateTransformation();
        void updateFootholdPosition();

    private:
        
        // Mode 0: RCF, 1: MPC
        int controller_mode;

        // Robot Properties
        double trunk_mass = 1.43315091;
        double upper_leg_mass = 0.14853845;
        double lower_leg_mass = 0.03070001;
        double upper_leg_length = 0.160;
        double lower_leg_length = 0.160;
        double gravity = 9.81;
        double total_mass;
        int num_joint = 8;
        double discrete_time = 0.04;
        int num_states = 15;
        int num_control = 12;
        int num_out = 3;
        double mu;          // Friction Coefficient
        double minFootForces;
        double maxFootForces;
        Eigen::Matrix3d inertia_b;
        Eigen::Matrix3d inertia_w;
        double d_haa_x = 0.190;
        double d_haa_y = 0.15005;
        double d_haa_z = 0.0;

        
        // Robot Variables
        Eigen::Matrix<double, 8, 1> q;
        Eigen::Matrix<double, 8, 1> qd;
        Eigen::Matrix<double, 8, 1> qdd;
        Eigen::Matrix<double, 8, 1> torque;

        Eigen::Matrix<double, 3, 1> com_pos;
        Eigen::Matrix<double, 3, 1> com_vel;
        Eigen::Matrix<double, 3, 1> com_pos_ref;
        Eigen::Matrix<double, 3, 1> com_vel_ref;
        Eigen::Matrix<double, 3, 1> com_ang_b_ref;
        Eigen::Matrix<double, 3, 1> com_ang_w_ref;
        Eigen::Matrix<double, 3, 1> com_vel_ang_b_ref;
        Eigen::Matrix<double, 3, 1> com_vel_ang_w_ref;
        Eigen::Matrix<double, 12, 1> grf_force;
        Eigen::Quaterniond com_angular_quaternion;
        Eigen::Vector3d com_ang_b;
        Eigen::Vector3d com_ang_w;
        Eigen::Vector3d com_vel_ang_b;
        Eigen::Vector3d com_vel_ang_w;
        Eigen::Matrix3d rot_com_to_world;
        Eigen::Matrix3d T_vel_ang_w_b;
        Eigen::Matrix<double, 3, 4> footholdPositions_b;
        Eigen::Matrix<double, 3, 4> footholdPositions_w;

        // Jacobians
        Eigen::Matrix<double, 3, 2> Jc_LF, Jc_RF, Jc_LH, Jc_RH;
        Eigen::Matrix<double, 3, 3> rot_pc_LF, rot_pc_RF, rot_pc_LH, rot_pc_RH;
        Eigen::Matrix<double, 12, 8> Jc;
        Eigen::Matrix<double, 8, 12> Jc_pinv;

        // Transforms
        Eigen::Matrix<double, 4, 4> T_com_LF_foot, T_com_RF_foot, T_com_LH_foot, T_com_RH_foot;
        Eigen::Matrix<double, 3, 3> rot_com_LF_foot, rot_com_RF_foot, rot_com_LH_foot, rot_com_RH_foot;
        Eigen::Matrix<double, 3, 1> d_com_LF_foot, d_com_RF_foot, d_com_LH_foot, d_com_RH_foot;
        Eigen::Matrix<double, 3, 3> skew_LF_foot_b, skew_RF_foot_b, skew_LH_foot_b, skew_RH_foot_b;
        //State Space
        Eigen::Matrix<double, 15, 15> Ac, Ad;
        Eigen::Matrix<double, 15, 12> Bc, Bd;
        Eigen::Matrix<double, 3, 15> Cc, Cd;
        Eigen::Matrix<double, 15, 1> xss;
        Eigen::Matrix<double, 15, 1> xss_ref;
        Eigen::Matrix<double, 12, 1> force_grf;

        // Mpc Optimization
        Eigen::MatrixXd Aopt, Bopt, Bopt_temp, Copt;
        Eigen::MatrixXd Hopt, Fopt;
        Eigen::VectorXd fosqp;
        Eigen::SparseMatrix<double> hessian_sparse;
        int Ny, Nu;
        Eigen::MatrixXd Q, R;
        Eigen::MatrixXd Qbar, Rbar;
        Eigen::MatrixXd kf, fc, kf_leg, fc_leg;
        Eigen::MatrixXd Gf, Wf;
        Eigen::MatrixXd Uopt;
        Eigen::MatrixXd Xref;
        Eigen::SparseMatrix<double> Aineq;
        // Eigen::MatrixXd bineq;
        Eigen::VectorXd bineq, lower_bound; 
        Eigen::Matrix<double, 3, 1> Qlinear;
        Eigen::Matrix<double, 3, 1> Qvellinear; 
        Eigen::Matrix<double, 3, 1> Qang;
        Eigen::Matrix<double, 3, 1> Qvelang; 
        double Rparam;  

        Eigen::Matrix<double, 3, 1> gravity_vector;
        Eigen::MatrixXd Identity3;

        // Publishers and Subscribers
        ros::Publisher joint_effort_cmd_pub_;
        ros::Publisher grf_cmd_pub_;
        ros::Subscriber joint_state_sub_;
        ros::Subscriber gazebo_model_state_sub_;
        ros::Subscriber com_ref_point_sub_;

        // Timer
        ros::Timer timer_control;

        // OSQP
        OsqpEigen::Solver osqp_solver;
        Eigen::VectorXd OSQP_solution;

        // RCF
        Eigen::MatrixXd Hrcf, Frcf;
        Eigen::MatrixXd wrench_matrix_rcf;
        Eigen::MatrixXd Qrcf, Rrcf;
        Eigen::Matrix<double, 3, 3> Rcf_Kp_linear;
        Eigen::Matrix<double, 3, 3> Rcf_Kd_linear;
        Eigen::Matrix<double, 3, 3> Rcf_Kp_ang;
        Eigen::Matrix<double, 3, 3> Rcf_Kd_ang;
        Eigen::Vector3d Rcf_acc_linear_des; 
        Eigen::Vector3d Rcf_acc_ang_des; 
        Eigen::Matrix<double, 6, 1> Rcf_acc_des;
        int flag_test;
        
        

};

#endif // SOLO8_CENTROID_MPC_H