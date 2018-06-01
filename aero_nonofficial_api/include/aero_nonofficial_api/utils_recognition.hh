#ifndef AERO_NONOFFICIAL_API_UTIL_RECOGNITION_
#define AERO_NONOFFICIAL_API_UTIL_RECOGNITION_

namespace aero
{
  namespace recognition
  {
    aero::interface::AeroMoveitInterface::Ptr ri;
    std::string camera_parent_link;
    aero::Transform camera_relative_pose; // camera_parent_link coords
    void setCameraTransform(aero::interface::AeroMoveitInterface::Ptr _robot,
                            std::string _link, aero::Transform _pose) {
      ri = _robot;
      camera_parent_link = _link;
      camera_relative_pose = _pose;
    }

    aero::Vector3 camera2Base(aero::Vector3 _pos, bool _update) {
      if (ri == nullptr) {
        ROS_ERROR("please initiate with setCameraTransform to use camera2Base!");
        return _pos;
      }
      if (_update) {
        ri->setRobotStateToCurrentState();
        ri->updateLinkTransforms();
      }
      aero::Vector3 b2h_pos =
        ri->kinematic_state->getGlobalLinkTransform(camera_parent_link).translation();
      aero::Quaternion b2h_qua
        (ri->kinematic_state->getGlobalLinkTransform(camera_parent_link).rotation());
      aero::Vector3 o2c =
        aero::Quaternion(camera_relative_pose.linear()) * _pos;
      return b2h_pos + b2h_qua * camera_relative_pose.translation() + b2h_qua * o2c;
    }

    // for simulation
    aero::Vector3 base2Camera(aero::Vector3 _pos, bool _update) {
      if (ri == nullptr) {
        ROS_ERROR("please initiate with setCameraTransform to use camera2Base!");
        return _pos;
      }
      if (_update) {
        ri->setRobotStateToCurrentState();
        ri->updateLinkTransforms();
      }
      aero::Vector3 b2h_pos =
        ri->kinematic_state->getGlobalLinkTransform(camera_parent_link).translation();
      aero::Quaternion b2h_qua
        (ri->kinematic_state->getGlobalLinkTransform(camera_parent_link).rotation());
      aero::Vector3 o2c =
        b2h_qua.inverse() * (_pos - b2h_pos - b2h_qua * camera_relative_pose.translation());
      return aero::Quaternion(camera_relative_pose.linear()).inverse() * o2c;
    }

    aero::Vector3 object_ref; // base_link coords
    aero::Vector3 object_pos; // base_link coords
    bool object_found;
    // callbackSimple: find any item closest to object_ref
    void callbackSimple(const aero_recognition_msgs::LabeledPoseArray::ConstPtr &_msg) {
      float min_diff = std::numeric_limits<float>::max();
      for (auto o = _msg->poses.begin(); o != _msg->poses.end(); ++o) {
        aero::Vector3 pos(o->pose.position.x, o->pose.position.y, o->pose.position.z);
        pos = camera2Base(pos, false);
        float diff = (pos - object_ref).norm();
        if (diff < min_diff) {
          min_diff = diff;
          object_pos = pos;
        }
        ROS_INFO("%s:item %s (%f, %f, %f)->(%f, %f, %f)", __FUNCTION__,
                 o->label.c_str(), o->pose.position.x, o->pose.position.y, o->pose.position.z,
                 pos.x(), pos.y(), pos.z());
      }
      object_found = true;
    }

    bool subscriber_initialized = false;
    ros::Subscriber object_sub;
    void initialize(ros::NodeHandle _nh) {
      if (subscriber_initialized)
        return;
      object_sub =
        _nh.subscribe("/object_3d_projector/output", 1, &callbackSimple);
      subscriber_initialized = true;
    }

    bool start(aero::Vector3 _pos=aero::Vector3(0.0, 0.0, 0.0)) {
      if (!subscriber_initialized)
        ROS_WARN("are you sure everything is setup correctly? forgot initialize(nh)?");
      object_ref = _pos;
      object_found = false;
#if defined(USE_GAZEBO_SIM_)
      return true;
#else
      std_srvs::SetBool srv;
      srv.request.data = true;
      return ros::service::call("/object_detector/set_mode", srv.request, srv.response);
#endif
    }

    bool finish() {
      object_found = false;
#if defined(USE_GAZEBO_SIM_)
      return true;
#else
      std_srvs::SetBool srv;
      srv.request.data = false;
      return ros::service::call("/object_detector/set_mode", srv.request, srv.response);
#endif
    }

  }
}

#endif
