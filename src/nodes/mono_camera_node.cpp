#include <ros/ros.h>
#include <mono_camera.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pp");

  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  avt_vimba_camera::MonoCamera mc(nh,nhp);

  ros::spin();
  return 0;
}