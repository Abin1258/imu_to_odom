#include <imu_to_odom/imu_to_odom.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "imu_to_odom");

  ros::NodeHandle nh;

  ImuOdom *imu_to_odom = new ImuOdom(nh);
  
  ros::spin();
  

  return 0;
}
