#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float32.h>

float angle = 0.0;

void SubscribeAngle(const std_msgs::Float32::ConstPtr& _angle)
{
  angle = -_angle->data * 0.017453292519943295;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sensor_plate_tf_broadcaster");
  ros::NodeHandle nh("~");

  ros::Subscriber angle_subscriber =
    nh.subscribe("/sensor_plate_controller/state", 100, &SubscribeAngle);

  std::string base_frame, sensor_frame;
  nh.getParam("base_frame", base_frame);
  nh.getParam("sensor_frame", sensor_frame);
  float rx, ry, z, rot1, rot2, rot3;
  nh.getParam("rx", rx);
  nh.getParam("ry", ry);
  nh.getParam("z", z);
  nh.getParam("rot1", rot1);
  nh.getParam("rot2", rot2);
  nh.getParam("rot3", rot3);

  float theta0 = -atan2(rx, ry);
  float radius = sqrt(std::pow(rx, 2) + std::pow(ry, 2));

  tf::TransformBroadcaster br;
  tf::Transform transform;
  ros::Rate rate(10.0);

  while (nh.ok())
  {
    ros::spinOnce();
    transform.setOrigin(tf::Vector3(
        radius * cos(angle + theta0), radius * sin(angle + theta0), z));
    transform.setRotation(
        tf::Quaternion(0, 0, angle) * tf::Quaternion(rot1, rot2, rot3));

    br.sendTransform(tf::StampedTransform(
        transform, ros::Time::now(), base_frame, sensor_frame));

    rate.sleep();
  }

  return 0;
};
