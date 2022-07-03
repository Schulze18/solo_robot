#include <solo_mpc_controller/solo8_centroid_mpc.h>

void Solo8CentroidMPC::updateContactJacobians(){

    // LF
    Jc_LF(0, 0) = -lower_leg_length*cos(this->q(0) + this->q(1)) - upper_leg_length*cos(this->q(0));
    Jc_LF(0, 1) = -lower_leg_length*cos(this->q(0) + this->q(1));

    Jc_LF(2, 0) = lower_leg_length*sin(this->q(0) + this->q(1)) + upper_leg_length*sin(this->q(0));
    Jc_LF(2, 1) = lower_leg_length*sin(this->q(0) + this->q(1));

    // RF
    Jc_RF(0, 0) = lower_leg_length*cos(this->q(2) + this->q(3)) + upper_leg_length*cos(this->q(2));
    Jc_RF(0, 1) = lower_leg_length*cos(this->q(2) + this->q(3));

    Jc_RF(2, 0) = lower_leg_length*sin(this->q(2) + this->q(3)) + upper_leg_length*sin(this->q(2));
    Jc_RF(2, 1) = lower_leg_length*sin(this->q(2) + this->q(3));

    // LH
    Jc_LH(0, 0) = -lower_leg_length*cos(this->q(4) + this->q(5)) - upper_leg_length*cos(this->q(4));
    Jc_LH(0, 1) = -lower_leg_length*cos(this->q(4) + this->q(5));

    Jc_LH(2, 0) = lower_leg_length*sin(this->q(4) + this->q(5)) + upper_leg_length*sin(this->q(4));
    Jc_LH(2, 1) = lower_leg_length*sin(this->q(4) + this->q(5));

    // RH
    Jc_RH(0, 0) = lower_leg_length*cos(this->q(6) + this->q(7)) + upper_leg_length*cos(this->q(6));
    Jc_RH(0, 1) = lower_leg_length*cos(this->q(6) + this->q(7));

    Jc_RH(2, 0) = lower_leg_length*sin(this->q(6) + this->q(7)) + upper_leg_length*sin(this->q(6));
    Jc_RH(2, 1) = lower_leg_length*sin(this->q(6) + this->q(7)); 

    /* // LF
    Jc_LF(0, 0) = -lower_leg_length*cos(this->q(0) + this->q(1)) - upper_leg_length*cos(this->q(0));
    Jc_LF(0, 1) = -lower_leg_length*cos(this->q(0) + this->q(1));

    Jc_LF(2, 0) = -lower_leg_length*sin(this->q(0) + this->q(1)) - upper_leg_length*sin(this->q(0));
    Jc_LF(2, 1) = -lower_leg_length*sin(this->q(0) + this->q(1));

    // RF
    Jc_RF(0, 0) = -lower_leg_length*cos(this->q(2) + this->q(3)) - upper_leg_length*cos(this->q(2));
    Jc_RF(0, 1) = -lower_leg_length*cos(this->q(2) + this->q(3));

    Jc_RF(2, 0) = -lower_leg_length*sin(this->q(2) + this->q(3)) - upper_leg_length*sin(this->q(2));
    Jc_RF(2, 1) = -lower_leg_length*sin(this->q(2) + this->q(3));

    // LH
    Jc_LH(0, 0) = -lower_leg_length*cos(this->q(4) + this->q(5)) - upper_leg_length*cos(this->q(4));
    Jc_LH(0, 1) = -lower_leg_length*cos(this->q(4) + this->q(5));

    Jc_LH(2, 0) = -lower_leg_length*sin(this->q(4) + this->q(5)) - upper_leg_length*sin(this->q(4));
    Jc_LH(2, 1) = -lower_leg_length*sin(this->q(4) + this->q(5));

    // RH
    Jc_RH(0, 0) = -lower_leg_length*cos(this->q(6) + this->q(7)) - upper_leg_length*cos(this->q(6));
    Jc_RH(0, 1) = -lower_leg_length*cos(this->q(6) + this->q(7));

    Jc_RH(2, 0) = -lower_leg_length*sin(this->q(6) + this->q(7)) - upper_leg_length*sin(this->q(6));
    Jc_RH(2, 1) = -lower_leg_length*sin(this->q(6) + this->q(7));  */


    // Update Contact Jacobian
    Jc.block(0, 0, 3, 2) = Jc_LF;
    Jc.block(3, 2, 3, 2) = Jc_RF;
    Jc.block(6, 4, 3, 2) = Jc_LH;
    Jc.block(9, 6, 3, 2) = Jc_RH;

    /* Jc.block(0, 0, 3, 2) = rot_pc_LF*Jc_LF;
    Jc.block(3, 2, 3, 2) = rot_pc_RF*Jc_RF;
    Jc.block(6, 4, 3, 2) = rot_pc_LH*Jc_LH;
    Jc.block(9, 6, 3, 2) = rot_pc_RH*Jc_RH; */

    /* std::cout << "Jc " << Jc << std::endl << std::endl; */

}


