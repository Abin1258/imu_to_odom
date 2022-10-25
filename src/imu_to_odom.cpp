#include <imu_to_odom/imu_integrator.hpp>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "imu_to_odom");

  ros::NodeHandle nh;

  ImuIntegrator *imu_integrator = new ImuIntegrator(nh);
  
  ros::spin();

  return 0;
}
