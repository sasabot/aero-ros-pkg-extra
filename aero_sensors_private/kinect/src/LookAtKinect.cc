#include "aero_sensors_private/RotateKinectAddon.hh"
#include <tf/transform_listener.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "look_at_kinect_node");
  ros::NodeHandle nh("~");

  double ref;
  nh.param<double>("ref", ref, 0.0);
  ref *= M_PI / 180.0;

  tf::TransformListener listener;
  aero::addon::RotateKinectAddon kinect(nh);

  ros::Rate rate(50);

  tf::StampedTransform tr;

  while (ros::ok()) {
    try {
      listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(5.0));
      listener.lookupTransform("/map", "/base_link", ros::Time(0), tr);
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s",ex.what());
      rate.sleep();
      continue;
    }

    double yaw = tf::getYaw(tr.getRotation());
    double desired = ref - yaw;

    while (desired > 180.0 * M_PI / 180.0) desired -= 360.0 * M_PI / 180.0;
    while (desired < -180.0 * M_PI / 180.0) desired += 360.0 * M_PI / 180.0;

    if (desired > 100.0 * M_PI/180.0) desired = -180.0 * M_PI / 180.0;
    kinect.rotateKinectTo(static_cast<int>(-desired * 180.0 / M_PI));
    rate.sleep();
  }

  ros::shutdown();
  return 0;
}
