#include "aero_sensors_private/RotateKinectAddon.hh"
#include <tf/transform_listener.h>

#include <roboenvcv/Int32Stamped.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>

#include <chrono>

#define T_ 5000 // 5 seconds

int target_;
Eigen::Vector3d vec_;
std::vector<Eigen::Vector3d> heads_;
bool all_lost_;

bool force_controlled_;
bool force_lock_;
std::chrono::high_resolution_clock::time_point force_begin_;
double ref_x_;
double ref_y_;

void getTarget(const std_msgs::String::ConstPtr &_msg) {
  if (_msg->data.find("negotiation_target:") != std::string::npos) {
    auto posi = _msg->data.find(":");
    auto posj = _msg->data.find(" ");
    target_ = std::stoi(_msg->data.substr(posi+1, posj - posi));
    if (target_ >= heads_.size() && all_lost_)
      target_ = -1;
  }
};

void getHeadPositionsGlobal(const geometry_msgs::PoseArray::ConstPtr &_msg) {
  heads_.resize(_msg->poses.size());
  auto h = heads_.begin();
  for (auto p = _msg->poses.begin(); p != _msg->poses.end(); ++p) {
    if (p->position.z > 0.001) // head cannot be of this height, person is lost
      *h = Eigen::Vector3d(p->position.x, p->position.y, 0);
    ++h;
  }
};

void getAllLost(const roboenvcv::Int32Stamped::ConstPtr &_msg) {
  if (_msg->data > 0) {
    all_lost_ = false;
  } else {
    target_ = -1;
    all_lost_ = true;
  }
};

void forceControl(const geometry_msgs::Point::ConstPtr &_msg) {
  force_controlled_ = true;
  force_begin_ = std::chrono::high_resolution_clock::now();
  ref_x_ = _msg->x;
  ref_y_ = _msg->y;
  if (_msg->z < -0.0001)
    force_lock_ = true;
  else
    force_lock_ = false;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "look_at_kinect_node");
  ros::NodeHandle nh("~");

  std::string ns("kinect");
  nh.getParam("ns", ns);

  bool track_head = false;
  nh.getParam("track_head", track_head);

  ref_x_ = 1.0;
  nh.getParam("xf", ref_x_);
  ref_y_ = 0.0;
  nh.getParam("yf", ref_y_);

  ros::Subscriber sub_target = nh.subscribe("/negomo/vp/negomo/var", 10, &getTarget);
  ros::Subscriber sub_heads = nh.subscribe("/sensor/face/position/global",
                                           10, &getHeadPositionsGlobal);
  ros::Subscriber sub_lost = nh.subscribe("/roboenvcv/personcount", 10, &getAllLost);

  ros::Subscriber sub_force_control = nh.subscribe("/kinect_controller/look_at", 10, &forceControl);

  // sensordirection inputaxis handled through LookAtKinect2
  ros::Publisher pub =
    nh.advertise<geometry_msgs::PoseStamped>
    ("/" + ns + "/global/sensordirection/inputaxis", 10);
  geometry_msgs::PoseStamped msg;
  msg.header.frame_id = "map";
  msg.pose.position.z = 0.0;

  target_ = -1;
  all_lost_ = false;
  vec_ = Eigen::Vector3d(ref_x_, ref_y_, 0.0);
  force_controlled_ = false;
  force_lock_ = false;
  double ref = 0.0;

  tf::TransformListener listener;
  aero::addon::RotateKinectAddon kinect(nh);

  ros::Rate rate(50);

  tf::StampedTransform tr;

  const double M_PI2 = M_PI * 2;
  const double LIMIT = 100.0 * M_PI/180.0;

  while (ros::ok()) {
    try {
      listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(5.0));
      listener.lookupTransform("/map", "/base_link", ros::Time(0), tr);
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s",ex.what());
      rate.sleep();
      continue;
    }

    ros::spinOnce();

    double yaw = tf::getYaw(tr.getRotation());
    Eigen::Vector3d vec(ref_x_, ref_y_, 0.0);
    if (target_ >= 0 && !force_controlled_ && track_head)
      if (target_ < heads_.size()) { // if target person, track
        double x = tr.getOrigin().getX();
        double y = tr.getOrigin().getY();
        Eigen::Vector3d origin(x, y, 0);
        vec = Eigen::Vector3d(heads_.at(target_) - origin);
        vec_ = vec;
      } else { // there is person but no targets, camera stay at position
        vec = vec_;
      }
    // else: no person at all, look at defined position

    if (vec.y() > 0)
      ref = acos(vec.x()/vec.norm());
    else
      ref = -acos(vec.x()/vec.norm());
    double desired = ref - yaw;

    while (desired > M_PI) desired -= M_PI2;
    while (desired < -M_PI) desired += M_PI2;

    if (desired > LIMIT) desired = -M_PI;
    kinect.rotateKinectTo(static_cast<int>(-desired * 180.0 / M_PI));

    msg.header.stamp = ros::Time::now();
    msg.pose.position.x = vec.x();
    msg.pose.position.y = vec.y();
    pub.publish(msg);

    // when not locked, force control is released after time elapse
    if (force_controlled_ && !force_lock_
        && std::chrono::duration_cast<std::chrono::milliseconds>
        (std::chrono::high_resolution_clock::now() - force_begin_).count() > T_)
      force_controlled_ = false;

    rate.sleep();
  }

  ros::shutdown();
  return 0;
}
