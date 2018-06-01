#ifndef AERO_NONOFFICIAL_API_UTIL_GAZEBO_
#define AERO_NONOFFICIAL_API_UTIL_GAZEBO_

#include "aero_benchmarks/utils.hh"

#include <std_srvs/Empty.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/GetModelProperties.h>
#include <gazebo_msgs/SetModelState.h>
#include <control_msgs/JointTrajectoryControllerState.h>

namespace aero
{
  namespace gazebo
  {
    ros::ServiceClient spawn_model;
    ros::ServiceClient delete_model;
    ros::ServiceClient reset_world;
    ros::ServiceClient pause_physics;
    ros::ServiceClient unpause_physics;
    ros::ServiceClient get_model_properties;
    ros::ServiceClient set_model_state;

    ros::Subscriber rarm_state;
    double rarm_joints_diff;
    void rarm_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr _msg) {
      rarm_joints_diff = 0.0;
      for (auto err = _msg->error.positions.begin(); err != _msg->error.positions.end(); ++err)
        rarm_joints_diff += fabs(*err);
    }

    ros::Subscriber larm_state;
    double larm_joints_diff;
    void larm_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr _msg) {
      larm_joints_diff = 0.0;
      for (auto err = _msg->error.positions.begin(); err != _msg->error.positions.end(); ++err)
        larm_joints_diff += fabs(*err);
    }

    ros::Subscriber lifter_state;
    double lifter_joints_diff;
    void lifter_callback(const control_msgs::JointTrajectoryControllerState::ConstPtr _msg) {
      lifter_joints_diff = 0.0;
      for (auto err = _msg->error.positions.begin(); err != _msg->error.positions.end(); ++err)
        lifter_joints_diff += fabs(*err);
    }

    void connectGazebo(ros::NodeHandle _nh) {
      spawn_model = _nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_sdf_model");
      delete_model = _nh.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");
      reset_world = _nh.serviceClient<std_srvs::Empty>("/gazebo/reset_world");
      pause_physics = _nh.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
      unpause_physics = _nh.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
      get_model_properties =
        _nh.serviceClient<gazebo_msgs::GetModelProperties>("/gazebo/get_model_properties");
      set_model_state =
        _nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
      // for correct wait interpolation
      rarm_state = _nh.subscribe<control_msgs::JointTrajectoryControllerState>
        ("/rarm_controller/state", 1, &rarm_callback);
      larm_state = _nh.subscribe<control_msgs::JointTrajectoryControllerState>
        ("/larm_controller/state", 1, &larm_callback);
      lifter_state = _nh.subscribe<control_msgs::JointTrajectoryControllerState>
        ("/lifter_controller/state", 1, &lifter_callback);
    }

    bool pausePhysics() {
      std_srvs::Empty srv;
      bool flag = pause_physics.call(srv);
      usleep(1000 * 1000);
      return flag;
    }

    bool unpausePhysics() {
      usleep(1000 * 1000); // just in case
      std_srvs::Empty srv;
      bool flag = unpause_physics.call(srv);
      usleep(1000 * 1000);
      return flag;
    }

    bool checkExist(std::string _name) {
      gazebo_msgs::GetModelProperties srv;
      srv.request.model_name = _name;
      if (!get_model_properties.call(srv)) {
        ROS_ERROR("unexpected failure for checking object in gazebo");
        std::exit(0);
      }
      return srv.response.success;
    }

    bool moveModel(std::string _name, aero::Transform _from,
                   aero::Transform _to, std::string _parent) {
      gazebo_msgs::SetModelState srv;
      srv.request.model_state.model_name = _name;
      aero::Vector3 pdiff = _to.translation() - _from.translation();
      aero::Quaternion qdiff =
        aero::Quaternion(_to.linear()) * aero::Quaternion(_from.linear()).inverse();
      aero::Transform pose = aero::Translation(pdiff) * qdiff;
      tf::poseEigenToMsg(pose, srv.request.model_state.pose);
      srv.request.model_state.reference_frame = _parent;
      if (!set_model_state.call(srv)) {
        ROS_ERROR("unexpected failure for setting object pose in gazebo");
        std::exit(0);
        return false;
      }
      return srv.response.success;
    }

    bool addSDF(std::string _name, std::string _filename, aero::Transform _pose) {
      std::string path = ros::package::getPath("aero_benchmarks") + "/gazebo_models/";
      std::ifstream f(path + _filename);
      std::stringstream buffer;
      buffer << f.rdbuf();
      gazebo_msgs::SpawnModel srv;
      srv.request.model_name = _name;
      srv.request.model_xml = buffer.str();
      tf::poseEigenToMsg(_pose, srv.request.initial_pose);
      f.close();
      return spawn_model.call(srv);
    }

    bool createShelfSDF(std::vector<aero::Vector3> _boards) {
      // overwrite model.sdf in aero_benchmarks/fcsc_shelf
      std::string path = ros::package::getPath("aero_benchmarks") + "/gazebo_models/";
      std::ifstream ifs(path + "fcsc_shelf/model.sdf");
      std::string line;
      int overwrite_line = -1;
      std::vector<std::string> lines;
      while (std::getline(ifs, line)) {
        if (overwrite_line >= 0) {
          line = "<pose>" + std::to_string(_boards.at(overwrite_line).x()) + " "
            + std::to_string(_boards.at(overwrite_line).y()) + " "
            + std::to_string(_boards.at(overwrite_line).z()) + "  0 0 0</pose>";
          overwrite_line = -1;
        } else if (line.find("link name=\"board0") != std::string::npos) {
          overwrite_line = 0;
        } else if (line.find("link name=\"board1") != std::string::npos) {
          overwrite_line = 1;
        }
        lines.push_back(line);
      }
      ifs.close();
      std::ofstream ofs(path + "fcsc_shelf/model.sdf");
      std::ofstream ofs13(path + "fcsc_shelf/model-1.3.sdf");
      std::ofstream ofs14(path + "fcsc_shelf/model-1.4.sdf");
      for (auto l = lines.begin(); l != lines.end(); ++l) {
        ofs << *l << "\n";
        ofs13 << *l << "\n";
        ofs14 << *l << "\n";
      }
      ofs.close();
      ofs13.close();
      ofs14.close();
    }

    bool createBoxSDF(aero::Vector3 _size, float _mass, float _mu=0.01, float _mu2=0.01) {
      // overwrite model.sdf in aero_benchmarks/diybox
      std::string path = ros::package::getPath("aero_benchmarks") + "/gazebo_models/";
      std::ifstream ifs(path + "diybox/model.sdf");
      std::string line;
      std::vector<std::string> lines;
      while (std::getline(ifs, line)) {
        if (line.find("<size>") != std::string::npos)
          line = "<size>" + std::to_string(_size.x()) + " "
            + std::to_string(_size.y()) + " " + std::to_string(_size.z()) + "</size>";
        else if (line.find("<mass ") != std::string::npos)
          line = "<mass value=\"" + std::to_string(_mass) + "\"/>";
        else if (line.find("<intertia ") != std::string::npos)
          line = "<inertia ixx=\""
            + std::to_string(_mass * (std::pow(_size.z(), 2) + std::pow(_size.x(), 2)) / 12.0)
            + "\" ixy=\"0.0\" ixz=\"0.0\" iyy=\""
            + std::to_string(_mass * (std::pow(_size.y(), 2) + std::pow(_size.x(), 2)) / 12.0)
            + " iyz=\"0.0\" izz=\""
            + std::to_string(_mass * (std::pow(_size.y(), 2) + std::pow(_size.z(), 2)) / 12.0)
            + "\"/>";
        else if (line.find("<mu>") != std::string::npos)
          line = "<mu>" + std::to_string(_mu) + "</mu>";
        else if (line.find("<mu2>") != std::string::npos)
          line = "<mu2>" + std::to_string(_mu2) + "</mu2>";
        lines.push_back(line);
      }
      ifs.close();
      std::ofstream ofs(path + "diybox/model.sdf");
      std::ofstream ofs13(path + "diybox/model-1.3.sdf");
      std::ofstream ofs14(path + "diybox/model-1.4.sdf");
      for (auto l = lines.begin(); l != lines.end(); ++l) {
        ofs << *l << "\n";
        ofs13 << *l << "\n";
        ofs14 << *l << "\n";
      }
      ofs.close();
      ofs13.close();
      ofs14.close();
    }

    bool deleteSDF(std::string _name) {
      ROS_WARN("deleteSDF is not recommended! is buggy with collision check!");
      gazebo_msgs::DeleteModel srv;
      srv.request.model_name = _name;
      return delete_model.call(srv);
    }

    bool resetWorld() {
      std_srvs::Empty srv;
      return reset_world.call(srv);
    }

    // for correct waitInterpolation

    enum struct controller : int {rarm, larm, lifter};
    bool waitInterpolation(aero::interface::AeroMoveitInterface::Ptr _robot,
                           controller _range, float _tolerance=0.02, float _timeout=5.0) {
      _robot->waitInterpolation();
      ros::spinOnce();
      auto s = ros::Time::now();
      float diff = 0.0;
      if (_range == controller::rarm) {
        ROS_INFO("joints diff after wait interpolation is %f", rarm_joints_diff);
        while (rarm_joints_diff > _tolerance && (ros::Time::now() - s).toSec() < _timeout) {
          ros::spinOnce();
          usleep(10 * 1000);
          diff = rarm_joints_diff;
        }
      } else if (_range == controller::larm) {
        ROS_INFO("joints diff after wait interpolation is %f", larm_joints_diff);
        while (larm_joints_diff > _tolerance && (ros::Time::now() - s).toSec() < _timeout) {
          ros::spinOnce();
          usleep(10 * 1000);
          diff = larm_joints_diff;
        }
      } else if (_range == controller::lifter) {
        ROS_INFO("joints diff after wait interpolation is %f", lifter_joints_diff);
        while (lifter_joints_diff > _tolerance && (ros::Time::now() - s).toSec() < _timeout) {
          ros::spinOnce();
          usleep(10 * 1000);
          diff = lifter_joints_diff;
        }
      }
      ROS_INFO("sleeped an extra %f seconds", (ros::Time::now() - s).toSec());
      if (diff > _tolerance) {
        ROS_WARN("waitInterpolation timed out with diff %f", diff);
        return false;
      }
      return true;
    }

    // sleep in simulation time
    inline void simsleep(float _sec) {
      auto s = ros::Time::now();
      while ((ros::Time::now() - s).toSec() < _sec) {
        usleep(10 * 1000);
      }
      ROS_INFO("slept %f seconds", (ros::Time::now() - s).toSec());
    }
  }
}

#endif
