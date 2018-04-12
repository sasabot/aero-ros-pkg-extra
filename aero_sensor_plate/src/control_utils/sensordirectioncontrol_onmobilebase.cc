#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf/transform_listener.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Point.h>

#include <chrono>

#define T_ 5000 // 5 seconds

double ref_x_;
double ref_y_;


void axisControl(const geometry_msgs::Point::ConstPtr &_msg) {
  ref_x_ = _msg->x;
  ref_y_ = _msg->y;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "position_plate_direction_node");
  ros::NodeHandle nh("~");

  std::string ns;
  nh.getParam("ns", ns);

  std::string baselink("/base_link");
  nh.getParam("base", baselink);

  ref_x_ = 1.0;
  nh.getParam("xf", ref_x_);
  ref_y_ = 0.0;
  nh.getParam("yf", ref_y_);

  ros::Publisher plate_control_publisher_ = nh.advertise<std_msgs::Float32>("/sensor_plate_controller/command", 1000);
  ros::Subscriber sub_axis = nh.subscribe("/sensor_plate_controller/look_at", 10, &axisControl);

  // sensordirection inputaxis handled through this node
  ros::Publisher pub =
    nh.advertise<geometry_msgs::PoseStamped>
    ("/" + ns + "/global/sensordirection/inputaxis", 10);
  geometry_msgs::PoseStamped msg;
  msg.header.frame_id = "map";
  msg.pose.position.z = 0.0;

  double ref = 0.0;

  tf::TransformListener listener;

  ros::Rate rate(50);

  tf::StampedTransform tr;

  const double M_PI2 = M_PI * 2;
  const double LIMIT = 100.0 * M_PI/180.0;

  while (ros::ok()) {
    try {
      listener.waitForTransform("/map", baselink, ros::Time(0), ros::Duration(5.0));
      listener.lookupTransform("/map", baselink, ros::Time(0), tr);
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s",ex.what());
      rate.sleep();
      continue;
    }

    ros::spinOnce();

    double yaw = tf::getYaw(tr.getRotation());
    Eigen::Vector3d vec(ref_x_, ref_y_, 0.0);

    if (vec.y() > 0)
      ref = acos(vec.x()/vec.norm());
    else
      ref = -acos(vec.x()/vec.norm());
    double desired = ref - yaw;

    while (desired > M_PI) desired -= M_PI2;
    while (desired < -M_PI) desired += M_PI2;

    if (desired > LIMIT) desired = -M_PI;
    std_msgs::Float32 angle;
    angle.data = -desired * 180.0 / M_PI;
    plate_control_publisher_.publish(angle);

    msg.header.stamp = ros::Time::now();
    msg.pose.position.x = vec.x();
    msg.pose.position.y = vec.y();
    pub.publish(msg);

    rate.sleep();
  }

  ros::shutdown();
  return 0;
}
