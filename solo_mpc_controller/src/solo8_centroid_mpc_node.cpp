#include <solo_mpc_controller/solo8_centroid_mpc.h>


int main(int argc, char **argv) {
  
    ros::init(argc, argv, "solo8_centroid_mpc");
    ros::NodeHandle nh("~");

    Solo8CentroidMPC Solo8CentroidMPC(nh);

    std::cout << "Solo8 MPC node running" << std::endl;

    ros::spin();
    return 0;

}
