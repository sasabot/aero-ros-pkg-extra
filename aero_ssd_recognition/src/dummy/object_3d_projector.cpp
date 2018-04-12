#include <aero_ssd_recognition/dummy/object_3d_projector.h>

namespace aero_ssd_recognition
{

  void DummyObject3DProjector::onInit()
  {
    DiagnosticNodelet::onInit();
    pnh_->param("approximate_sync", approximate_sync_, true);
    pub_ = advertise<aero_recognition_msgs::LabeledPoseArray>(*pnh_, "output", 1);
    debug_pub_ = advertise<geometry_msgs::PoseArray>(*pnh_, "debug_output", 1);
    onInitPostProcess();
  }

  void DummyObject3DProjector::subscribe()
  {
    sub_box_ = pnh_->subscribe("input_boxes", 10, &DummyObject3DProjector::apply, this);
    ros::V_string names = boost::assign::list_of("~input_boxes");
    jsk_topic_tools::warnNoRemap(names);
  }

  void DummyObject3DProjector::unsubscribe()
  {
    sub_box_.shutdown();
  }

  void DummyObject3DProjector::apply(const aero_recognition_msgs::Scored2DBoxArray::ConstPtr& boxes_msg)
  {

    std::cout << "3d projector" << std::endl;

    aero_recognition_msgs::LabeledPoseArray poses_out_msg;
    geometry_msgs::PoseArray poses_debug_msg;
    poses_out_msg.header = boxes_msg->header;
    poses_debug_msg.header = boxes_msg->header;
    for(auto box : boxes_msg->boxes){
      std::cout << "project : " << box.label << " ";

      // dummy data
      double x, y, z;
      if (box.label == "caffelatte") {
        x = 0.2;//0.3;
        y = 0.25;//-0.2;
        z = 0.6;//0.8;
      } else if (box.label == "onigiri") {
        x = -0.2;
        y = 0.25;
        z = 0.6;
      } else if (box.label == "sandwich") {
        x = 0.1;
        y = 0.1;
        z = 0.7;
      }

      aero_recognition_msgs::LabeledPose pose;
      pose.label = box.label;
      pose.pose.position.x = x;
      pose.pose.position.y = y;
      pose.pose.position.z = z;
      pose.pose.orientation.w = 1.0;
      poses_out_msg.poses.push_back(pose);
      geometry_msgs::Pose debug_pose;
      debug_pose.position.x = x;
      debug_pose.position.y = y;
      debug_pose.position.z = z;
      debug_pose.orientation.w = 1.0;
      poses_debug_msg.poses.push_back(debug_pose);
      std::cout << "pos : " << x << " " << y << " " << z << std::endl;
    }

    if(poses_out_msg.poses.size() > 0){
      pub_.publish(poses_out_msg);
      debug_pub_.publish(poses_debug_msg);
    }
  }
};

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (aero_ssd_recognition::DummyObject3DProjector, nodelet::Nodelet);
