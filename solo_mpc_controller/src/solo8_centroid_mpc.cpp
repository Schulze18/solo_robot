#include <solo_mpc_controller/solo8_centroid_mpc.h>


Solo8CentroidMPC::Solo8CentroidMPC(ros::NodeHandle& nh_){

    ROS_INFO("Solo8 MPC Constructor");

    // Get Parameters
    nh_.param<int>("mode", this->controller_mode, 0);
    nh_.param<double>("time", sampling_time, 0.004);

    nh_.param<double>("Qx", Qlinear(0,0), 100);
    nh_.param<double>("Qy", Qlinear(1,0), 100);
    nh_.param<double>("Qz", Qlinear(2,0), 500);
    nh_.param<double>("Qxd", Qvellinear(0,0), 20);
    nh_.param<double>("Qyd", Qvellinear(1,0), 20);
    nh_.param<double>("Qzd", Qvellinear(2,0), 100);
    nh_.param<double>("Qroll", Qang(0,0), 1000);
    nh_.param<double>("Qpitch", Qang(1,0), 1000);
    nh_.param<double>("Qyaw", Qang(2,0), 500);
    nh_.param<double>("Qrolld", Qvelang(0,0), 20);
    nh_.param<double>("Qpitchd", Qvelang(1,0), 20);
    nh_.param<double>("Qyawd", Qvelang(2,0), 20);
    nh_.param<double>("R", Rparam, 0.001);

    nh_.param<int>("Ny", Ny, 25);
    nh_.param<int>("Nu", Nu, 10);

    flag_test = 1;

    joint_state_sub_ = nh_.subscribe<sensor_msgs::JointState> ("/joint_states", 1, &Solo8CentroidMPC::jointStateCallback, this);
    gazebo_model_state_sub_ = nh_.subscribe<gazebo_msgs::ModelStates> ("/gazebo/model_states", 1, &Solo8CentroidMPC::gazeboModelStateCallback, this);
    com_ref_point_sub_ = nh_.subscribe<geometry_msgs::PoseStamped> ("/com_pose_ref", 1, &Solo8CentroidMPC::comRefPoseCallback, this);

    joint_effort_cmd_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/group_feedforward_joints_effort_controllers/command", 5);
    grf_cmd_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/computed_grf", 5);

    if(this->controller_mode == 0){
        ROS_WARN("RCF Controller selected.");
        initMatrices();
        initRcfMatrices();
        updateTransformation();
        updateFootholdPosition();
        updateRcfMatrices();
        osqpRcfSetup();
    }
    if(this->controller_mode == 1){
        ROS_WARN("MPC Controller selected.");
        initMatrices();
        updateTransformation();
        updateFootholdPosition();
        updateStateSpace();
        updatePredictionModel();
        updateOptMatrices();
        updateConstMatrices();
        osqpMpcSetup();
    }

    // Create Timer
    timer_control = nh_.createTimer(ros::Duration(this->sampling_time), &Solo8CentroidMPC::timeControllerCallback, this);

}


Solo8CentroidMPC::~Solo8CentroidMPC(){

}


void Solo8CentroidMPC::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg){
    /* ROS_WARN("Receive joint state "); */

    // Get Joint State
    // LF Joints
    this->q(0) = msg->position[0];
    this->q(1) = msg->position[1];
    this->qd(0) = msg->velocity[0];
    this->qd(1) = msg->velocity[1];

    // RF Joints
    this->q(2) = msg->position[4];
    this->q(3) = msg->position[5];
    this->qd(2) = msg->velocity[4];
    this->qd(3) = msg->velocity[5];

    // LH Joints
    this->q(4) = msg->position[2];
    this->q(5) = msg->position[3];
    this->qd(4) = msg->velocity[2];
    this->qd(5) = msg->velocity[3];

    // RH Joints
    this->q(6) = msg->position[6];
    this->q(7) = msg->position[7];
    this->qd(6) = msg->velocity[6];
    this->qd(7) = msg->velocity[7];

    // Update Contact Jacobian
    /* updateRotationMatrix();
    updateContactJacobians(); */

    /* double total_mass =  (4*upper_leg_mass + 4*lower_leg_mass + trunk_mass); */
    /* grf_force(2) = total_mass*gravity/4;
    grf_force(5) = total_mass*gravity/4;
    grf_force(8) = total_mass*gravity/4;
    grf_force(11) = total_mass*gravity/4; */
    

   /*  torque = -Jc.transpose()*grf_force; */
    
    /* Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cqr(Jc);
    Jc_pinv = cqr.pseudoInverse();
    torque = -Jc_pinv*grf_force; */
    
    // LF
    /* torque(0) = -0.18783615494898598;
    torque(1) = 0.349032660791293;
    
    // RF
    torque(2) = -0.18783615494898598;
    torque(3) = 0.349032660791293;
    
    // LH
    torque(4) =  0.18783615494898598;
    torque(5) = -0.349032660791293;

    // RH
    torque(6) = 0.18783615494898598;
    torque(7) = -0.349032660791293; */

    /* std::cout << "GRF " << grf_force << std::endl << std::endl;
    std::cout << "Torque " << torque << std::endl << std::endl; */

    /* pubTorqueCmd(); */

}


void Solo8CentroidMPC::gazeboModelStateCallback(const gazebo_msgs::ModelStates::ConstPtr& msg){
    /* ROS_WARN("Receive gazebo model state "); */

    // Get CoM State
    this->com_pos(0,0) = msg->pose[1].position.x;
    this->com_pos(1,0) = msg->pose[1].position.y;
    this->com_pos(2,0) = msg->pose[1].position.z;
    /* std::cout << "CoM Pos" << std::endl << this->com_pos << std::endl; */

    this->com_angular_quaternion.x() = msg->pose[1].orientation.x;
    this->com_angular_quaternion.y() = msg->pose[1].orientation.y;
    this->com_angular_quaternion.z() = msg->pose[1].orientation.z;
    this->com_angular_quaternion.w() = msg->pose[1].orientation.w;
    /* this->rot_com_to_world = this->com_angular_quaternion.toRotationMatrix();
    this->com_ang_b = this->rot_com_to_world.eulerAngles(0,1,2); */
    // ROS_INFO("CoM Ang Quat %f  %f  %f %f", this->com_angular_quaternion.x(),  this->com_angular_quaternion.y(),  this->com_angular_quaternion.z(),  this->com_angular_quaternion.w());
    tf::Quaternion quat_ang;
	tf::quaternionMsgToTF(msg->pose[1].orientation, quat_ang);
    /* tf::Matrix3x3(quat_ang).getEulerYPR(this->com_ang_b[2], this->com_ang_b[1], this->com_ang_b[0]);
    ROS_INFO("CoM Ang Pos %f  %f  %f", this->com_ang_b[0], this->com_ang_b[1], this->com_ang_b[2]); */
    tf::Matrix3x3(quat_ang).getEulerYPR(this->com_ang_w[2], this->com_ang_w[1], this->com_ang_w[0]);
    // ROS_INFO("CoM Ang Pos World %f  %f  %f", this->com_ang_w[0], this->com_ang_w[1], this->com_ang_w[2]);

    // Update Rotational Matrix
    this->rot_com_to_world(0,0) = cos(this->com_ang_w[2])*cos(this->com_ang_w[1]);
    this->rot_com_to_world(0,1) = sin(this->com_ang_w[2])*cos(this->com_ang_w[1]);
    this->rot_com_to_world(0,2) = -sin(this->com_ang_w[1]);
    this->rot_com_to_world(1,0) = cos(this->com_ang_w[2])*sin(this->com_ang_w[1])*sin(this->com_ang_w[0]) - sin(this->com_ang_w[2])*cos(this->com_ang_w[0]); 
    this->rot_com_to_world(1,1) = sin(this->com_ang_w[2])*sin(this->com_ang_w[1])*sin(this->com_ang_w[0]) + cos(this->com_ang_w[2])*cos(this->com_ang_w[0]); 
    this->rot_com_to_world(1,2) = cos(this->com_ang_w[1])*sin(this->com_ang_w[0]);
    this->rot_com_to_world(2,0) = cos(this->com_ang_w[2])*sin(this->com_ang_w[1])*cos(this->com_ang_w[0]) + sin(this->com_ang_w[2])*cos(this->com_ang_w[0]);  
    this->rot_com_to_world(2,1) = sin(this->com_ang_w[2])*sin(this->com_ang_w[1])*cos(this->com_ang_w[0]) - cos(this->com_ang_w[2])*sin(this->com_ang_w[0]); 
    this->rot_com_to_world(2,2) = cos(this->com_ang_w[1])*cos(this->com_ang_w[0]);
    this->rot_com_to_world =  this->rot_com_to_world.transpose();

    this->com_ang_b = this->com_ang_w;

    // Get Velocity
    this->com_vel(0,0) = msg->twist[1].linear.x;
    this->com_vel(1,0) = msg->twist[1].linear.y;
    this->com_vel(2,0) = msg->twist[1].linear.z;
    this->com_vel_ang_w(0,0) = msg->twist[1].angular.x;
    this->com_vel_ang_w(1,0) = msg->twist[1].angular.y;
    this->com_vel_ang_w(2,0) = msg->twist[1].angular.z;

    T_vel_ang_w_b(0,0) = cos(com_ang_w(2))/cos(com_ang_w(1));
    T_vel_ang_w_b(0,1) = sin(com_ang_w(2))/cos(com_ang_w(1));
    T_vel_ang_w_b(0,2) = 0;
    T_vel_ang_w_b(1,0) = -sin(com_ang_w(2));
    T_vel_ang_w_b(1,1) = cos(com_ang_w(2));
    T_vel_ang_w_b(1,2) = 0;
    T_vel_ang_w_b(2,0) = tan(com_ang_w(1))*cos(com_ang_w(2));
    T_vel_ang_w_b(2,1) = tan(com_ang_w(1))*sin(com_ang_w(2));
    T_vel_ang_w_b(2,2) = 1;

    this->com_vel_ang_b = T_vel_ang_w_b*this->com_vel_ang_w;
    /* std::cout << "CoM Vel" << std::endl << this->com_vel << std::endl; */
    /* ROS_INFO("CoM Ang Vel %f  %f  %f", this->com_vel_ang_b[0], this->com_vel_ang_b[1], this->com_vel_ang_b[2]); */
    
}


void Solo8CentroidMPC::pubTorqueCmd(){
    std_msgs::Float64MultiArray msg;

    msg.layout.dim.push_back(std_msgs::MultiArrayDimension());  
    msg.layout.dim[0].size = 8;
    msg.layout.dim[0].stride = 1;
    msg.layout.dim[0].label = "Computed Torque";

    for (int i = 0; i < num_joint; i++){
        msg.data.push_back(torque(i));
    }

    joint_effort_cmd_pub_.publish(msg);

}


void Solo8CentroidMPC::updateRotationMatrix(){

    // LF
    rot_pc_LF(0,0) = cos(this->q(0) + this->q(1));
    rot_pc_LF(0,1) = -sin(this->q(0) + this->q(1));
    rot_pc_LF(1,0) = sin(this->q(0) + this->q(1));
    rot_pc_LF(1,1) = cos(this->q(0) + this->q(1));
    rot_pc_LF(2,2) = 1;

    // RF
    rot_pc_RF(0,0) = cos(this->q(2) + this->q(3));
    rot_pc_RF(0,1) = -sin(this->q(2) + this->q(3));
    rot_pc_RF(1,0) = sin(this->q(2) + this->q(3));
    rot_pc_RF(1,1) = cos(this->q(2) + this->q(3));
    rot_pc_RF(2,2) = 1;

    // LH
    rot_pc_LH(0,0) = cos(this->q(4) + this->q(5));
    rot_pc_LH(0,1) = -sin(this->q(4) + this->q(5));
    rot_pc_LH(1,0) = sin(this->q(4) + this->q(5));
    rot_pc_LH(1,1) = cos(this->q(4) + this->q(5));
    rot_pc_LH(2,2) = 1;

    // RH
    rot_pc_RH(0,0) = cos(this->q(6) + this->q(7));
    rot_pc_RH(0,1) = -sin(this->q(6) + this->q(7));
    rot_pc_RH(1,0) = sin(this->q(6) + this->q(7));
    rot_pc_RH(1,1) = cos(this->q(6) + this->q(7));
    rot_pc_RH(2,2) = 1;

}


void Solo8CentroidMPC::initMatrices(){

    // Initiate variables
    Jc.setZero();
    Jc_LF.setZero();
    Jc_RF.setZero();
    Jc_LH.setZero();
    Jc_RH.setZero();
    rot_pc_LF.setZero();
    rot_pc_RF.setZero();
    rot_pc_LH.setZero();
    rot_pc_RH.setZero();
    grf_force.setZero();
    Ac.setZero();
    Bc.setZero();
    Cc.setZero();
    Ad.setZero();
    Bd.setZero();
    Cd.setZero();
    gravity_vector.setZero();
    xss.setZero();
    xss_ref.setZero();
    inertia_b.setZero();
    inertia_w.setZero();
    T_com_LF_foot.setZero();
    T_com_RF_foot.setZero();
    T_com_LH_foot.setZero();
    T_com_RH_foot.setZero();
    rot_com_LF_foot.setZero();
    rot_com_RF_foot.setZero();
    rot_com_LH_foot.setZero();
    rot_com_RH_foot.setZero();
    skew_LF_foot_b.setZero();
    skew_RF_foot_b.setZero();
    skew_LH_foot_b.setZero();
    skew_RH_foot_b.setZero();
    

    total_mass = (4*upper_leg_mass + 4*lower_leg_mass + trunk_mass);

    mu = 0.6;
    minFootForces = 1;
    maxFootForces = 20;
    gravity_vector(2,0) = -gravity;
    inertia_b(0,0) = 0.00578574;
    inertia_b(1,1) = 0.01938108;
    inertia_b(2,2) = 0.02476124;
    rot_com_to_world =  Eigen::MatrixXd::Identity(3,3);
    Identity3 = Eigen::MatrixXd::Identity(3,3);

    Aopt = Eigen::MatrixXd::Zero(num_states*Ny, num_states);
    Bopt = Eigen::MatrixXd::Zero(num_states*Ny, num_control*Nu);
    Bopt_temp = Eigen::MatrixXd::Zero(num_states, num_control*Nu);
    Q = Eigen::MatrixXd::Zero(num_states, num_states);
    Qbar = Eigen::MatrixXd::Zero(num_states*Ny, num_states*Ny);
    R = Rparam*Eigen::MatrixXd::Identity(num_control, num_control);
    Rbar = Eigen::MatrixXd::Identity(num_control*Nu, num_control*Nu);
    Hopt = Eigen::MatrixXd::Zero(num_control*Nu, num_control*Nu);
    Fopt = Eigen::MatrixXd::Zero(1, num_control*Nu);
    Uopt = Eigen::MatrixXd::Zero(num_control*Nu, 1);
    Xref = Eigen::MatrixXd::Zero(num_states*Ny, 1);
    bineq = Eigen::MatrixXd::Zero(num_control*Nu, 1);
    fosqp = Eigen::VectorXd::Zero(num_control*Nu, 1);

    // Friction Constraints Matrices
    kf_leg = Eigen::MatrixXd::Zero(6, 3);
    kf = Eigen::MatrixXd::Zero(24, 12);
    fc_leg = Eigen::MatrixXd::Zero(6, 1);
    fc = Eigen::MatrixXd::Zero(24, 1);

    kf_leg(0,0) = 1; kf_leg(0,2) = -mu;
    kf_leg(1,0) = -1; kf_leg(1,2) = -mu;
    kf_leg(2,1) = 1; kf_leg(2,2) = -mu;
    kf_leg(3,1) = -1; kf_leg(3,2) = -mu;
    kf_leg(4,2) = 1;
    kf_leg(5,2) = -1;
    
    fc_leg(4,0) = maxFootForces;
    fc_leg(5,0) = -minFootForces;

    Gf = Eigen::MatrixXd::Zero(kf.rows()*Nu, kf.cols()*Nu);
    Wf = Eigen::MatrixXd::Zero(fc.rows()*Nu, 1);
    for(int i = 0; i < 4; i++){
        kf.block(6*i, 3*i, 6, 3) = kf_leg;
        fc.block(6*i, 0, 6, 1) = fc_leg;
    }
    for(int i = 0; i < Nu; i++){
        Gf.block(i*kf.rows(), i*kf.cols(), kf.rows(), kf.cols()) = kf;
        Wf.block(i*fc.rows(), 0, fc.rows(), fc.cols()) = fc;
    }

    // Weight Matrices
    Q(0,0) = Qlinear(0,0);
    Q(1,1) = Qlinear(1,0);
    Q(2,2) = Qlinear(2,0);
    //
    Q(3,3) = Qvellinear(0,0);
    Q(4,4) = Qvellinear(1,0);
    Q(5,5) = Qvellinear(2,0);

    Q(6,6) = Qang(0,0);
    Q(7,7) = Qang(1,0);
    Q(8,8) = Qang(2,0);
    //
    Q(9,9) = Qvelang(0,0);
    Q(10,10) = Qvelang(1,0);
    Q(11,11) = Qvelang(2,0);

    for(int i = 0; i < Ny; i++){
        Qbar.block(num_states*i,num_states*i,num_states,num_states) = Q;
    }
    for(int i = 0; i < Nu; i++){
        Rbar.block(num_control*i,num_control*i,num_control,num_control) = R;
    } 
  
    // Reference Values
    com_pos_ref.setZero();
    com_vel_ref.setZero();
    com_ang_b_ref.setZero();
    com_ang_w_ref.setZero();
    com_vel_ang_w_ref.setZero();
    com_vel_ang_b_ref.setZero();

    com_pos_ref(2) = 0.25;
    
}


void Solo8CentroidMPC::updateStateSpaceLinear(){
    /* State Space
    [rd rdd gd]' = [0 1 0; 0 0 1; 0 0 0][r rd g]'
                 + [0 0 0 0; 1/m 1/m 1/m 1/m; 0 0 0 0][F_lf; F_rf; F_lh; F_rh]

    [r] = [1 0 0][r rd g]'
    */
    // Assemble State
    this->xss.block(0, 0, 3, 1) = this->com_pos;
    this->xss.block(3, 0, 3, 1) = this->com_vel;
    this->xss.block(6, 0, 3, 1) = this->gravity_vector;

    this->xss_ref.block(0, 0, 3, 1) = this->com_pos_ref;
    this->xss_ref.block(3, 0, 3, 1) = this->com_vel_ref;
    this->xss_ref.block(6, 0, 3, 1) = this->gravity_vector;

    this->Xref = kroneckerProduct( Eigen::MatrixXd::Ones(Ny,1), this->xss_ref);

    Ac.block(0, 3, 3, 3) = Identity3;
    Ac.block(3, 6, 3, 3) = Identity3;

    Bc.block(3, 0, 3, 3) = Identity3 / total_mass;
    Bc.block(3, 3, 3, 3) = Identity3 / total_mass;
    Bc.block(3, 6, 3, 3) = Identity3 / total_mass;
    Bc.block(3, 9, 3, 3) = Identity3 / total_mass;

    Cc.block(0, 0, 3, 3) = Identity3;

    // Discretization
    Ad = Eigen::MatrixXd::Identity(num_states,num_states) + sampling_time*Ac;
    Bd = sampling_time*Bc;
    Cd = Cc;

}


void Solo8CentroidMPC::updateStateSpace(){
    /* State Space
    [rd rdd thd omegad gd]' = [0 1 0 0 0; 0 0 0 0 1; 0 0 0 T 0; 0 0 0 0 0; 0 0 0 0 0]
                              [r rd th omega g]'
                 + [0 0 0 0 0; 1/m 1/m 1/m 1/m; 0 0 0 0 0; Ip_lf Ip_rf Ip_lh Ip_rh; 0 0 0 0 0]
                            [F_lf; F_rf; F_lh; F_rh]

    [r] = [1 0 0 0 0; 0 0 1 0 0][r rd th omega g]'
    */
    // Assemble State
    this->xss.block(0, 0, 3, 1) = this->com_pos;
    this->xss.block(3, 0, 3, 1) = this->com_vel;
    this->xss.block(6, 0, 3, 1) = this->com_ang_b;
    this->xss.block(9, 0, 3, 1) = this->com_vel_ang_b;
    this->xss.block(12, 0, 3, 1) = this->gravity_vector;

    this->xss_ref.block(0, 0, 3, 1) = this->com_pos_ref;
    this->xss_ref.block(3, 0, 3, 1) = this->com_vel_ref;
    this->xss_ref.block(6, 0, 3, 1) = this->com_ang_b_ref;
    this->xss_ref.block(9, 0, 3, 1) = this->com_vel_ang_b_ref;
    this->xss_ref.block(12, 0, 3, 1) = this->gravity_vector;

    this->Xref = kroneckerProduct( Eigen::MatrixXd::Ones(Ny,1), this->xss_ref);

    // std::cout << "xss " << std::endl << this->xss.transpose() << std::endl;
    // std::cout << "xss_ref " << std::endl << this->xss_ref.transpose() << std::endl;

    // State Space
    Ac.block(0, 3, 3, 3) = Identity3;
    Ac.block(3, 12, 3, 3) = Identity3;
    Ac.block(6, 9, 3, 3) = T_vel_ang_w_b;

    Bc.block(3, 0, 3, 3) = Identity3 / total_mass;
    Bc.block(3, 3, 3, 3) = Identity3 / total_mass;
    Bc.block(3, 6, 3, 3) = Identity3 / total_mass;
    Bc.block(3, 9, 3, 3) = Identity3 / total_mass;
    //
    Bc.block(9, 0, 3, 3) = inertia_w.inverse()*skew_LF_foot_b;
    Bc.block(9, 3, 3, 3) = inertia_w.inverse()*skew_RF_foot_b;
    Bc.block(9, 6, 3, 3) = inertia_w.inverse()*skew_LH_foot_b;
    Bc.block(9, 9, 3, 3) = inertia_w.inverse()*skew_RH_foot_b;

    Cc.block(0, 0, 3, 3) = Identity3;
    Cc.block(3, 6, 3, 3) = Identity3;

    // Discretization
    Ad = Eigen::MatrixXd::Identity(num_states,num_states) + sampling_time*Ac;
    Bd = sampling_time*Bc;
    Cd = Cc;
}


void Solo8CentroidMPC::updatePredictionModel(){

    Aopt.block(0, 0, num_states, num_states) = Ad;
    for(int i = 1; i < Ny; i++){
        Aopt.block(num_states*i, 0, num_states, num_states) = Ad*Aopt.block(num_states*(i-1), 0, num_states, num_states);
    }

    for(int i = 0; i < Ny; i++){
        Bopt_temp.setZero();
        for(int j = 0; j < Nu; j++){
            if (i>j-1){
                Bopt_temp.block(0, num_control*j, num_states, num_control) =  Ad.pow(i-j)*Bd;
            }  
        }
        Bopt.block(num_states*i, 0, num_states, num_control*Nu) = Bopt_temp;
    }
}


void Solo8CentroidMPC::updateOptMatrices(){

    Hopt = Rbar + Bopt.transpose()*Qbar*Bopt;
    Fopt = (Aopt*xss - Xref).transpose()*Qbar*Bopt;

}


void Solo8CentroidMPC::updateConstMatrices(){
    Aineq = Gf.sparseView();
    bineq = Wf;
}


void Solo8CentroidMPC::initRcfMatrices(){
    wrench_matrix_rcf = Eigen::MatrixXd::Zero(6, num_control);
    wrench_matrix_rcf.block(0, 0, 3, 3) = Identity3 / total_mass;
    wrench_matrix_rcf.block(0, 3, 3, 3) = Identity3 / total_mass;
    wrench_matrix_rcf.block(0, 6, 3, 3) = Identity3 / total_mass;
    wrench_matrix_rcf.block(0, 9, 3, 3) = Identity3 / total_mass;

    Rcf_acc_linear_des = Eigen::MatrixXd::Zero(3, 1);
    Rcf_acc_ang_des = Eigen::MatrixXd::Zero(3, 1);
    Qrcf = Eigen::MatrixXd::Identity(6, 6);
    Rrcf = Eigen::MatrixXd::Identity(num_control, num_control);

    Rcf_Kp_linear.setZero();
    Rcf_Kp_linear(0, 0) = 150;
    Rcf_Kp_linear(1, 1) = 150;
    Rcf_Kp_linear(2, 2) = 2000;
    Rcf_Kd_linear.setZero();
    Rcf_Kd_linear(0, 0) = 100;
    Rcf_Kd_linear(1, 1) = 100;
    Rcf_Kd_linear(2, 2) = 100;

    Rcf_Kp_ang.setZero();
    Rcf_Kp_ang(0, 0) = 1000;
    Rcf_Kp_ang(1, 1) = 1000;
    Rcf_Kp_ang(2, 2) = 100;
    Rcf_Kd_ang.setZero();
    Rcf_Kd_ang(0, 0) = 200;
    Rcf_Kd_ang(1, 1) = 200;
    Rcf_Kd_ang(2, 2) = 100;
}


void Solo8CentroidMPC::updateRcfMatrices(){
   
    // Update Desired Acc
    Rcf_acc_linear_des = Rcf_Kp_linear*(com_pos_ref - com_pos) + Rcf_Kd_linear*(com_vel_ref - com_vel);
    
    Rcf_acc_ang_des = Rcf_Kp_ang*(com_ang_b_ref-com_ang_b) + Rcf_Kd_ang*(-com_vel_ang_b);
    inertia_w = rot_com_to_world*inertia_b*rot_com_to_world.transpose();

    Rcf_acc_des.block(0,0,3,1) = Rcf_acc_linear_des;
    Rcf_acc_des.block(3,0,3,1) = Rcf_acc_ang_des;


    wrench_matrix_rcf.block(3, 0, 3, 3) = inertia_w.inverse()*skew_LF_foot_b;
    wrench_matrix_rcf.block(3, 3, 3, 3) = inertia_w.inverse()*skew_RF_foot_b;
    wrench_matrix_rcf.block(3, 6, 3, 3) = inertia_w.inverse()*skew_LH_foot_b;
    wrench_matrix_rcf.block(3, 9, 3, 3) = inertia_w.inverse()*skew_RH_foot_b;

    Hrcf = Rrcf + wrench_matrix_rcf.transpose()*Qrcf*wrench_matrix_rcf;
    Frcf = -Rcf_acc_des.transpose()*Qrcf*wrench_matrix_rcf;
}


void Solo8CentroidMPC::timeControllerCallback(const ros::TimerEvent& event){

    if(this->controller_mode == 0){
        ROS_INFO("New RCF Torque computed at %fs", this->sampling_time);    
        
        updateTransformation();    
        updateFootholdPosition();
        updateRcfMatrices();
        osqpRcfSolve();

        grf_force = OSQP_solution;

    }
    else if(this->controller_mode == 1){
        ROS_INFO("New MPC Torque computed at %fs", this->sampling_time);
        
        // Update all data
        updateTransformation();
        updateFootholdPosition();
        updateStateSpace();
        updatePredictionModel();
        updateOptMatrices();
        updateConstMatrices();

        // Solve
        osqpMpcSolve();

        /* std::cout << "OSQP solution " << OSQP_solution << std::endl;  */
        Uopt = OSQP_solution;
        grf_force = Uopt.block(0, 0, num_control, 1);
    
    }

    updateContactJacobians();

    torque = -Jc.transpose()*grf_force;
  
    /* std::cout << "GRF Torque " << torque << std::endl; */
    pubTorqueCmd();
    pubGrfCmd();

}


int Solo8CentroidMPC::osqpMpcSetup(){

    // Solver Settings
    this->osqp_solver.settings()->setVerbosity(false);  
    this->osqp_solver.settings()->setWarmStart(false);
    this->osqp_solver.settings()->setMaxIteration(3000);
    this->osqp_solver.settings()->setCheckTermination(50);
    this->osqp_solver.data()->setNumberOfVariables(num_control*Nu);
    this->osqp_solver.data()->setNumberOfConstraints(Aineq.rows());

    hessian_sparse = Hopt.sparseView();
    fosqp = Fopt.transpose();
    lower_bound = Eigen::VectorXd::Constant(Aineq.rows(), -1e7);

    if( !this->osqp_solver.data()->setHessianMatrix( hessian_sparse ) )  return 1;
    if( !this->osqp_solver.data()->setGradient( fosqp ) ) return 1; 
    if( !this->osqp_solver.data()->setLinearConstraintsMatrix( Aineq ) ) return 1;
    if( !this->osqp_solver.data()->setUpperBound( bineq ) ) return 1;
    if( !this->osqp_solver.data()->setLowerBound( lower_bound ) ) return 1;
    
    if( !this->osqp_solver.initSolver() ){
        std::cout << "Can't init solver" << std::endl;
    }
    else{
        std::cout << "OSQP initialized." << std::endl;
    }
    
    return 0;

}


int Solo8CentroidMPC::osqpRcfSetup(){

    // Solver Settings
    this->osqp_solver.settings()->setVerbosity(false);  
    this->osqp_solver.settings()->setWarmStart(true);
    this->osqp_solver.settings()->setMaxIteration(3000);
    this->osqp_solver.settings()->setCheckTermination(50);
    this->osqp_solver.data()->setNumberOfVariables(num_control);
    this->osqp_solver.data()->setNumberOfConstraints(kf.rows());

    hessian_sparse = Hrcf.sparseView();
    fosqp = Frcf;
    Aineq = kf.sparseView();
    bineq = fc;
    lower_bound = Eigen::VectorXd::Constant(kf.rows(), -1e7);

    // Setup Osqp
    if( !this->osqp_solver.data()->setHessianMatrix( hessian_sparse ) )  return 1;
    if( !this->osqp_solver.data()->setGradient( fosqp ) ) return 1; 
    if( !this->osqp_solver.data()->setLinearConstraintsMatrix( Aineq ) ) return 1;
    if( !this->osqp_solver.data()->setUpperBound( bineq ) ) return 1;
    if( !this->osqp_solver.data()->setLowerBound( lower_bound ) ) return 1;
    
   
    if( !this->osqp_solver.initSolver() ){
        std::cout << "Can't init solver" << std::endl;
    }
    else{
        std::cout << "OSQP initialized." << std::endl;
    }

    return 0;

}


int Solo8CentroidMPC::osqpRcfSolve(){

    // Update Matrices
    hessian_sparse = Hrcf.sparseView();
    fosqp = Frcf;
    if( !this->osqp_solver.updateHessianMatrix( hessian_sparse ) )  return 1;
    if( !this->osqp_solver.updateGradient( fosqp ) )  return 1;
   
    if( !this->osqp_solver.solve()){
        std::cout << "ERROR" << std::endl;
    }
    OSQP_solution = this->osqp_solver.getSolution();

    return 0;

}


int Solo8CentroidMPC::osqpMpcSolve(){

    // Update Matrices
    hessian_sparse = Hopt.sparseView();
    fosqp = Fopt.transpose();
 
    // Update Osqp Data
    if( !this->osqp_solver.updateHessianMatrix( hessian_sparse ) )  return 1;
    if( !this->osqp_solver.updateGradient( fosqp ) )  return 1;

    if( !this->osqp_solver.solve()){
        std::cout << "ERROR" << std::endl;
    }
    else{
        OSQP_solution = this->osqp_solver.getSolution();
    }

    return 0;

}


void Solo8CentroidMPC::comRefPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    // Get linear reference
    this->com_pos_ref(0,0) = msg->pose.position.x;
    this->com_pos_ref(1,0) = msg->pose.position.y;
    this->com_pos_ref(2,0) = msg->pose.position.z;
   
    // Get Orientation Reference
    tf::Quaternion quat_ang;
	tf::quaternionMsgToTF(msg->pose.orientation, quat_ang);
    tf::Matrix3x3(quat_ang).getEulerYPR(this->com_ang_b_ref[2], this->com_ang_b_ref[1], this->com_ang_b_ref[0]);
}

void Solo8CentroidMPC::updateTransformation(){

    // LF
    rot_com_LF_foot(0,0) = -sin(q(0) + q(1));
    rot_com_LF_foot(0,1) = -cos(q(0) + q(1));
    rot_com_LF_foot(1,2) = 1;
    rot_com_LF_foot(2,0) = -cos(q(0) + q(1));
    rot_com_LF_foot(2,1) = sin(q(0) + q(1));
    // Translation
    d_com_LF_foot(0,0) = d_haa_x - lower_leg_length*sin(q(0) + q(1)) - upper_leg_length*sin(q(0));
    d_com_LF_foot(1,0) = d_haa_y;
    d_com_LF_foot(2,0) = d_haa_z - lower_leg_length*cos(q(0) + q(1)) - upper_leg_length*cos(q(0));
    T_com_LF_foot.block(0, 0, 3, 3) = rot_com_LF_foot;
    T_com_LF_foot.block(0, 3, 3, 1) = d_com_LF_foot;
    T_com_LF_foot(3,3) = 1;

    // RF
    rot_com_RF_foot(0,0) = sin(q(2) + q(3));
    rot_com_RF_foot(0,1) = cos(q(2) + q(3));
    rot_com_RF_foot(1,2) = -1;
    rot_com_RF_foot(2,0) = -cos(q(2) + q(3));
    rot_com_RF_foot(2,1) = sin(q(2) + q(3));
    // Translation
    d_com_RF_foot(0,0) = d_haa_x + lower_leg_length*sin(q(2) + q(3)) + upper_leg_length*sin(q(2));
    d_com_RF_foot(1,0) = -d_haa_y;
    d_com_RF_foot(2,0) = d_haa_z - lower_leg_length*cos(q(2) + q(3)) - upper_leg_length*cos(q(2));
    T_com_RF_foot.block(0, 0, 3, 3) = rot_com_RF_foot;
    T_com_RF_foot.block(0, 3, 3, 1) = d_com_RF_foot;
    T_com_RF_foot(3,3) = 1;

    // LH
    rot_com_LH_foot(0,0) = -sin(q(4) + q(5));
    rot_com_LH_foot(0,1) = -cos(q(4) + q(5));
    rot_com_LH_foot(1,2) = 1;
    rot_com_LH_foot(2,0) = -cos(q(4) + q(5));
    rot_com_LH_foot(2,1) = sin(q(4) + q(5));
    d_com_LH_foot(0,0) = - d_haa_x - lower_leg_length*sin(q(4) + q(5)) - upper_leg_length*sin(q(4));
    d_com_LH_foot(1,0) = d_haa_y;
    d_com_LH_foot(2,0) = d_haa_z - lower_leg_length*cos(q(4) + q(5)) - upper_leg_length*cos(q(4));
    T_com_LH_foot.block(0, 0, 3, 3) = rot_com_LH_foot;
    T_com_LH_foot.block(0, 3, 3, 1) = d_com_LH_foot;
    T_com_LH_foot(3,3) = 1;

    // RH
    rot_com_RH_foot(0,0) = sin(q(6) + q(7));
    rot_com_RH_foot(0,1) = cos(q(6) + q(7));
    rot_com_RH_foot(1,2) = -1;
    rot_com_RH_foot(2,0) = -cos(q(6) + q(7));
    rot_com_RH_foot(2,1) = sin(q(6) + q(7));
    d_com_RH_foot(0,0) = -d_haa_x + lower_leg_length*sin(q(6) + q(7)) + upper_leg_length*sin(q(6));
    d_com_RH_foot(1,0) = -d_haa_y;
    d_com_RH_foot(2,0) = d_haa_z - lower_leg_length*cos(q(6) + q(7)) - upper_leg_length*cos(q(6));
    T_com_RH_foot.block(0, 0, 3, 3) = rot_com_RH_foot;
    T_com_RH_foot.block(0, 3, 3, 1) = d_com_RH_foot;
    T_com_RH_foot(3,3) = 1;


    inertia_w = rot_com_to_world*inertia_b*rot_com_to_world.transpose();
}


void Solo8CentroidMPC::updateFootholdPosition(){
    
    // Base 
    // LF
    footholdPositions_b.block(0,0,3,1) = d_com_LF_foot;
    // RF
    footholdPositions_b.block(0,1,3,1) = d_com_RF_foot;
    // LH
    footholdPositions_b.block(0,2,3,1) = d_com_LH_foot;
    // RH
    footholdPositions_b.block(0,3,3,1) = d_com_RH_foot;

    // World
    // LF
    footholdPositions_w.block(0,0,3,1) = footholdPositions_b.block(0,0,3,1) + com_pos;
    // RF
    footholdPositions_w.block(0,1,3,1) = footholdPositions_b.block(0,1,3,1) + com_pos;
    // LH
    footholdPositions_w.block(0,2,3,1) = footholdPositions_b.block(0,2,3,1) + com_pos;
    // RH
    footholdPositions_w.block(0,3,3,1) = footholdPositions_b.block(0,3,3,1) + com_pos;

    // Compute Skew matrices
    // LF
    skew_LF_foot_b = Solo8CentroidMPC::skewMat(footholdPositions_b.col(0));

    // RF
    skew_RF_foot_b = Solo8CentroidMPC::skewMat(footholdPositions_b.col(1));

    // LH
    skew_LH_foot_b = Solo8CentroidMPC::skewMat(footholdPositions_b.col(2));

    // RH
    skew_RH_foot_b = Solo8CentroidMPC::skewMat(footholdPositions_b.col(3));

}


void Solo8CentroidMPC::pubGrfCmd(){
    std_msgs::Float64MultiArray msg;

    msg.layout.dim.push_back(std_msgs::MultiArrayDimension());  
    msg.layout.dim[0].size = 12;
    msg.layout.dim[0].stride = 1;
    msg.layout.dim[0].label = "Computed GRF";

    for (int i = 0; i < 4; i++){
        for(int j = 0; j < 3; j++){
            msg.data.push_back( grf_force(3*i + j, 0) );
        }
    }

    grf_cmd_pub_.publish(msg);

}


Eigen::Matrix3d Solo8CentroidMPC::skewMat(Eigen::Vector3d input_skew){

    Eigen::Matrix3d skew_mat;
    skew_mat << 0.0, -input_skew(2,0), input_skew(1,0), input_skew(2,0), 0.0, -input_skew(0,0), -input_skew(1,0), input_skew(0,0), 0.0;

    return skew_mat;

}