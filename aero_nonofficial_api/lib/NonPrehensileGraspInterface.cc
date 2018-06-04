#include "aero_nonofficial_api/NonPrehensileGraspInterface.hh"

aero::interface::npgrasp_standard::npgrasp_standard
(ros::NodeHandle &_nh, const aero::interface::npgrasp_standard::npgrasptype _type)
  : NonPrehensileGraspInterface(_nh) {
  type_ = _type;
  // reach offset is required to solve collision avoidance
  switch (type_) {
  case aero::interface::npgrasp_standard::npgrasptype::tumblev:
  case aero::interface::npgrasp_standard::npgrasptype::tumbleforce: {
    offset_ = aero::Vector3(0.02, 0.0, 0.03);    
    break;
  }
  default:
    break;
  }
};

aero::interface::npgrasp_standard::~npgrasp_standard() {
}

bool aero::interface::npgrasp_standard::execute
(aero::interface::AeroMoveitInterface::Ptr _robot,
 aero::interface::AeroMotionPlanningInterface::Ptr _mpi) {
  if (type_ == aero::interface::npgrasp_standard::npgrasptype::tumbleforce) {
    calibrate();
    if (!waitCalibrate()) {
      ROS_ERROR("failed touch sensor calibration");
      return false;
    }
  }

  execute_(_robot, _mpi);

  // TODO: parameters should depend on object and size
  switch (type_) {
  case aero::interface::npgrasp_standard::npgrasptype::tumblev: {
    if (!executeGraspDefinition // push
        (_robot, aero::grasp_definition::definition({-0.004, -0.03}, 2000, false)))
      return false;
    if (!executeGraspDefinition // pull
        (_robot, aero::grasp_definition::definition({-0.045, 0.0}, 2000, false)))
      return false;
    if (!executeGraspDefinition // release
        (_robot, aero::grasp_definition::definition({0.0, 0.04}, 2000, false)))
      return false;
    if (!executeGraspDefinition // lower for stable grasp
        (_robot, aero::grasp_definition::definition({0.0, -0.02}, 2000, false)))
      return false;
    if (!executeGraspDefinition // grasp
        (_robot, aero::grasp_definition::definition({arm_, 0.0}, false)))
      return false;
    return true;
  }
  case aero::interface::npgrasp_standard::npgrasptype::tumbleforce: {
    if (!executeGraspDefinition
        (_robot, aero::grasp_definition::definition
         (aero::grasp_definition::forcedef
          (aero::grasp_definition::actions::touch, 3.0, 2000, 0.1), true)))
      return false;
    if (!executeGraspDefinition
        (_robot, aero::grasp_definition::definition
         (aero::grasp_definition::forcedef
          (aero::grasp_definition::actions::pull, 0.5, 1000, 0.1), true)))
      return false;
    if (!executeGraspDefinition
        (_robot, aero::grasp_definition::definition
         (aero::grasp_definition::forcedef
          (aero::grasp_definition::actions::release, 0.5, 2000, 0.1), true)))
      return false;
    return true;
  }
  default:
    return false;
  }

  return true;
}

aero::interface::npgrasp_else::npgrasp_else
(ros::NodeHandle &_nh, const aero::interface::npgrasp_else::npgrasptype _type)
  : NonPrehensileGraspInterface(_nh) {
  type_ = _type;

  // reach offset is required to solve collision avoidance
  switch (type_) {
  case aero::interface::npgrasp_else::npgrasptype::bottle: {
    offset_ = aero::Vector3(-0.07, -0.01, 0.0);
    break;
  }
  case aero::interface::npgrasp_else::npgrasptype::carton: {
    offset_ = aero::Vector3(-0.02, 0.0, 0.01);
    break;
  }
  default:
    break;
  }
};

aero::interface::npgrasp_else::~npgrasp_else() {
}

bool aero::interface::npgrasp_else::execute
(aero::interface::AeroMoveitInterface::Ptr _robot,
 aero::interface::AeroMotionPlanningInterface::Ptr _mpi) {
  execute_(_robot, _mpi);

  // TODO: parameters should depend on object position, weight and size
  switch (type_) {
  case aero::interface::npgrasp_else::npgrasptype::bottle: {
    if (!executeGraspDefinition // reach
        (_robot, aero::grasp_definition::definition({0.04, 0.0}, 2000, false)))
      return false;
    // if (!executeGraspDefinition // grasp
    //     (_robot, aero::grasp_definition::definition({arm_, 0.5}, false)))
    //   return false;
    /* *** this is a hack for stepper hand which does really bad in grasping *** */
    _robot->sendGrasp(aero::arm::larm);
    usleep(1000 * 5000);
    _robot->sendGrasp(aero::arm::larm);
    usleep(1000 * 15000);
    /* *** hack ends here *** */
    if (!executeGraspDefinition // pull1 = r+h*sin(0.236482)-r*cos(0.236482)
        (_robot, aero::grasp_definition::definition({-0.05, 0.0}, 2000, false)))
      return false;
    if (!executeGraspDefinition // pull2 = r+h*sin(0.472964)-r*cos(0.472964)
        (_robot, aero::grasp_definition::definition({-0.0995, 0.0995}, 2000, false)))
      return false;
    return true;
  }
  case aero::interface::npgrasp_else::npgrasptype::carton: {
    if (!executeGraspDefinition // reach
        (_robot, aero::grasp_definition::definition({0.031, 0.03} , 2000, false)))
      return false;
    if (!executeGraspDefinition // touch
        (_robot, aero::grasp_definition::definition({0.01, -0.028}, 2000, false)))
      return false;
    if (!executeGraspDefinition // pull = r+h*sin()-r*cos(), h*(1-cos())-r*sin()
        (_robot, aero::grasp_definition::definition({-0.14, -0.01}, 2000, false)))
      return false;
    if (!executeGraspDefinition // release = -touch
        (_robot, aero::grasp_definition::definition({0.01, 0.03}, 2000, false)))
      return false;
    if (!executeGraspDefinition // back hand
        (_robot, aero::grasp_definition::definition({-0.08, 0.0}, 2000, false)))
      return false;
    if (!executeGraspDefinition // open
        (_robot, aero::grasp_definition::definition({arm_, -0.65}, false)))
      return false;
    // if (!executeGraspDefinition // re-reach hand (should match back hand)
    //     (_robot, aero::grasp_definition::definition
    //      ({arm_, aero::Translation(0.0, 0.0, 0.0) * aero::Quaternion(0.707107, 0.707107, 0.0, 0.0)},
    //       {0.08, 0.0}, 3000, false)))
    //   return false;
    if (!executeGraspDefinition // down
        (_robot, aero::grasp_definition::definition({0.01, -0.03}, 2000, false)))
      return false;
    if (!executeGraspDefinition // re-reach
        (_robot, aero::grasp_definition::definition({0.095, -0.035}, 2000, false)))
      return false;
    if (!executeGraspDefinition // grasp
        (_robot, aero::grasp_definition::definition({arm_, 0.0}, false)))
      return false;
    if (!executeGraspDefinition // pull finish 1
        (_robot, aero::grasp_definition::definition({0.0, 0.03}, 2000, false)))
      return false;
    if (!executeGraspDefinition // pull finish 2
        (_robot, aero::grasp_definition::definition({-0.1, 0.05}, 2000, false)))
      return false;
    return true;
  }
  default:
    return false;
  }

  return true;
}

aero::interface::NonPrehensileGraspInterface::NonPrehensileGraspInterface
(ros::NodeHandle &_nh) : nh_(_nh), force_spinner_(1, &force_queue_)  {
  setupTouchSensing(); // does it anyways despite not using a touch sensor
}

aero::interface::NonPrehensileGraspInterface::~NonPrehensileGraspInterface() {
}

bool aero::interface::NonPrehensileGraspInterface::executeGraspDefinition
(aero::interface::AeroMoveitInterface::Ptr _robot, const aero::grasp_definition::definition _def) {
  switch(_def.type) {
  case aero::grasp_definition::types::gripper: {
    if (_def.gripper.first == aero::arm::rarm) {
      _robot->setHand(_def.gripper.first, -_def.gripper.second);
      _robot->sendHand(_def.gripper.first, -_def.gripper.second, 3.0); // hack
    } else {
      _robot->setHand(_def.gripper.first, _def.gripper.second);
      _robot->sendHand(_def.gripper.first, _def.gripper.second, 3.0); // hack
    }
    _robot->waitInterpolation();
    usleep(1000 * 1000);
    return true;
  }

  case aero::grasp_definition::types::lifteroffset: {
    double x, z;
    _robot->getLifter(x, z);
    _robot->setLifter(x + _def.lifteroffset.first, z + _def.lifteroffset.second);
    _robot->sendLifter(x + _def.lifteroffset.first, z + _def.lifteroffset.second, _def.time);
#if defined(USE_GAZEBO_SIM_)
    aero::gazebo::waitInterpolation(_robot, aero::gazebo::controller::lifter);
#else
    _robot->waitInterpolation();
#endif
    return true;
  }

  case aero::grasp_definition::types::transformoffset: {
    // TODO: reverse pose for rarm
    aero::Vector3 p = _robot->getEEFPosition(_def.pose.first, aero::eef::grasp);
    aero::Quaternion q(_def.pose.second.linear());
    aero::Transform goal = aero::Translation(p.x(), p.y(), p.z()) * q;
    if (!_robot->setFromIK(_def.pose.first, aero::ikrange::upperbody, goal, aero::eef::grasp))
      return false;
    double x, z;
    _robot->getLifter(x, z);
    _robot->setLifter(x + _def.lifteroffset.first, z + _def.lifteroffset.second);
    _robot->sendModelAngles(_def.time);
#if defined(USE_GAZEBO_SIM_)
    aero::gazebo::waitInterpolation(_robot, aero::gazebo::controller::lifter);
#else
    _robot->waitInterpolation();
#endif
    return true;
  }

  case aero::grasp_definition::types::force: {
    switch(_def.force.action) {
    case aero::grasp_definition::actions::touch: {
      setBeginForce();
      double lifter_x0, lifter_z0;
      _robot->getLifter(lifter_x0, lifter_z0);
      bool status = touchTillForceDetection
        (_robot, _def.force.force_threshold, _def.force.time,
         lifter_x0, lifter_z0 - _def.force.move_limit, &index_force_z_);
      // TODO: please see tmbTweakTillForce for remaining implementation
      if (_def.save_lifter) {
      }
      setEndForce();

      if (!status)
        ROS_ERROR("failed touch!");
      float fu = readLastForce(1);
      float fmax = readMaxForce(1);
      // saveForceAnalysis(logdir_, "id" + object_id_ + "forcetweak",
      //                   {{"force[N]", _fz},
      //                    {"time[ms]", _t},
      //                    {"fu[N]", fu},
      //                    {"fmax[N]", fmax}});
      details_.status = status;
      details_.fu = fu;
      details_.fmax =fmax;
      // TODO: please see doHeuristic and see where to jump
      return status;
    }
    case aero::grasp_definition::actions::pull: {
      double lifter_x0, lifter_z0;
      _robot->getLifter(lifter_x0, lifter_z0);

      setBeginForce();
      double x = lifter_x0 - _def.force.move_limit;
      double z = lifter_z0;
      printf("maximum goal: %f, %f\n", x, z);
      while (!_robot->setLifter(x, z) && x < lifter_x0) {
        ROS_INFO("could not solve: %f, %f\n", x, z);
        x += 0.01; // adjust max amount
      }
      int time = _def.force.time * static_cast<int>((lifter_x0 - x) / 0.02);
      auto send_time_start = ros::Time::now();
      if (!_robot->sendLifter(x, z, time)) {
        ROS_ERROR("failed pull");
        setEndForce();
        return false;
      }
      force_mutex_.lock();
      float force = index_force_z_;
      force_mutex_.unlock();
      // when force is smaller than threshold, or move finished
      while (1) {
        if (force < _def.force.force_threshold ||
            (ros::Time::now() - send_time_start).toSec() > (time*0.001 + 1.0))
          break;
        force_mutex_.lock();
        force = index_force_z_;
        force_mutex_.unlock();
      }
      _robot->cancelLifter(); // stop current movement
      _robot->setRobotStateToCurrentState();
      _robot->getLifter(saved_lifter_.first, saved_lifter_.second); // save lifter movement
      setEndForce();

      // read thumb force to check tumble slip in-hand
      float fg = readLastForce(0);
      float fmax = readMaxForce(0);
      // saveForceAnalysis(logdir_, "id" + object_id_ + "forceslide",
      //                   {{"force[N]", _fs},
      //                    {"time[ms]", _t},
      //                    {"fg[N]", fg}, {"fmax[N]", fmax}});
      double diff = lifter_x0 - saved_lifter_.first;
      details_.status = true;
      details_.fg = fg;
      details_.fmax = fmax;
      details_.diff = diff;
      // TODO: please see doHeuristic and see where to jump
      return true;
    }
    case aero::grasp_definition::actions::release: {
      double lifter_x0, lifter_z0;
      _robot->getLifter(lifter_x0, lifter_z0);
      setBeginForce();
      bool status = touchTillForceDetection
        (_robot, _def.force.force_threshold, _def.force.time,
         lifter_x0, lifter_z0 + _def.force.move_limit, &thumb_force_z_);
      setEndForce();

      if (!status)
        ROS_ERROR("failed force tweak!");
      float fg = readLastForce(0);
      float fmax = readMaxForce(0);
      // saveForceAnalysis(logdir_, "id" + object_id_ + "forcerevtweak",
      //                   {{"force[N]", _fz},
      //                    {"time[ms]", _t},
      //                    {"fg[N]", fg}, {"fmax[N]", fmax}});
      _robot->setRobotStateToCurrentState();
      double lifter_x1, lifter_z1;
      _robot->getLifter(lifter_x1, lifter_z1);
      double diff = lifter_z1 - lifter_z0;
      details_.status = status;
      details_.fg = fg;
      details_.fmax = fmax;
      details_.diff = diff;
      return status;
    }
    default:
      return false;
    }
  }
  default:
    return false;
  }

  return true;
}

bool aero::interface::NonPrehensileGraspInterface::plan_
(aero::interface::AeroMoveitInterface::Ptr _robot,
 aero::interface::AeroMotionPlanningInterface::Ptr _mpi,
 const aero::arm _arm, aero::Transform _target, const bool _joint) {
  _target = aero::Translation(_target.translation() + offset_) * aero::Quaternion(_target.linear());
  // solve reach trajectory
  _mpi->setCurrentState(_robot);
  planning_interface::MotionPlanRequest req;
  req.group_name = aero::moveGroup(_arm, aero::ikrange::upperbody);
  moveit_msgs::Constraints pose_goalp;
  if (_joint) {
    if (!_robot->setFromIK(_arm, aero::ikrange::upperbody, _target, aero::eef::hand)) {
      ROS_ERROR("failed planning from ik");
      return false;
    }
    pose_goalp =
      _mpi->constructGoalConstraints(_robot, req.group_name, 0.005, 0.005);
  } else {
    pose_goalp =
      _mpi->constructGoalConstraints("base_link", _target, _arm, 0.005, 0.005);
  }
  req.goal_constraints.push_back(pose_goalp);
  _mpi->fillWorkspace(req);
  req.planner_id = "BKPIECEkConfigDefault"; //"RRTstarkConfigDefault";
  req.allowed_planning_time = 10.0;
  _mpi->displayScene();
  double t0;
  bool found = _mpi->plan(req, solution_.joints, t0, 10);
  if (!found) {
    ROS_ERROR("failed planning");
    return false;
  }
  arm_ = _arm;

  return true;
}

bool aero::interface::NonPrehensileGraspInterface::execute_
(aero::interface::AeroMoveitInterface::Ptr _robot,
 aero::interface::AeroMotionPlanningInterface::Ptr _mpi) {
  moveit_msgs::MotionPlanResponse res = solution_.joints;

  _mpi->execute(_robot, res, 200*res.trajectory.joint_trajectory.points.size(),
                aero::ikrange::upperbody);
  ROS_INFO("executing %d seconds",
           static_cast<int>(200*res.trajectory.joint_trajectory.points.size()));
  _mpi->displayTrajectory(res);
  _mpi->setStateToTrajectoryEnd(_robot, res);
#if defined(USE_GAZEBO_SIM_)
  if (arm_ == aero::arm::rarm)
    aero::gazebo::waitInterpolation(_robot, aero::gazebo::controller::rarm);
  else
    aero::gazebo::waitInterpolation(_robot, aero::gazebo::controller::larm);
#else
  _robot->waitInterpolation();
#endif

  return true;
}

void aero::interface::NonPrehensileGraspInterface::calibrate() {
#if !defined(USE_GAZEBO_SIM_)
  sensor_calibrated_ = 0;
  std::thread calibrate([&](){
      std_srvs::Trigger srv;
      if (!calibration_client_.call(srv)) {
        sensor_calibrated_ = -1;
        return;
      }
      if (!srv.response.success) {
        sensor_calibrated_ = -1;
        return;
      }
      calibration_mutex_.lock();
      sensor_calibrated_ = 1;
      calibration_mutex_.unlock();
    });
  calibrate.detach();
#endif
}

bool aero::interface::NonPrehensileGraspInterface::touchTillForceDetection
(aero::interface::AeroMoveitInterface::Ptr _robot, const float _force, const int _t,
 const double _x, const double _z, float* _ref_force) {
  printf("maximum goal: %f, %f\n", _x, _z);
  if (!_robot->setLifter(_x, _z)) {
    ROS_ERROR("In touch solve: cannot touch, I.K. failed.");
    return false;
  }
  auto send_start_time = ros::Time::now();
  _robot->sendLifter(_x, _z, _t);

  force_mutex_.lock();
  float force = *_ref_force;
  force_mutex_.unlock();
  while (1) { // when force is larger than threshold, or move finished
    if (force > _force || (ros::Time::now() - send_start_time).toSec() > (_t * 0.001 + 1.0))
      break;
    force_mutex_.lock();
    force = *_ref_force;
    force_mutex_.unlock();
  }
  _robot->cancelLifter(); // stop current movement
  return true;
}

bool aero::interface::NonPrehensileGraspInterface::waitCalibrate() {
#if !defined(USE_GAZEBO_SIM_)
  calibration_mutex_.lock();
  bool sensor_calibrated = sensor_calibrated_;
  calibration_mutex_.unlock();
  while (sensor_calibrated == 0) {
    calibration_mutex_.lock();
    sensor_calibrated = sensor_calibrated_;
    calibration_mutex_.unlock();
  }
  if (sensor_calibrated_ == -1) { // check for sensor calibration status
    return false;
  }
#endif
  return true;
}

void aero::interface::NonPrehensileGraspInterface::setupTouchSensing() {
#if !defined(USE_GAZEBO_SIM_)
  calibration_client_ =
    nh_.serviceClient<std_srvs::Trigger>("/touchence/calibrate");
#endif

  record_force_ = false;
  force_analysis_ = false;
  record_start_time_ = ros::Time::now();
  force_data_.resize(3);
  force_data4analysis_.resize(3);

  ros::SubscribeOptions force_ops0 =
    ros::SubscribeOptions::create<geometry_msgs::Vector3>
    ("/touchence/force01",
     10,
     boost::bind(&aero::interface::NonPrehensileGraspInterface::Force0Callback, this, _1),
     ros::VoidPtr(),
     &force_queue_);
  force0_sub_ = nh_.subscribe(force_ops0);

  ros::SubscribeOptions force_ops1 =
    ros::SubscribeOptions::create<geometry_msgs::Vector3>
    ("/touchence/force02",
     10,
     boost::bind(&aero::interface::NonPrehensileGraspInterface::Force1Callback, this, _1),
     ros::VoidPtr(),
     &force_queue_);
  force1_sub_ = nh_.subscribe(force_ops1);

  ros::SubscribeOptions force_ops2 =
    ros::SubscribeOptions::create<geometry_msgs::Vector3>
    ("/touchence/force03",
     10,
     boost::bind(&aero::interface::NonPrehensileGraspInterface::Force2Callback, this, _1),
     ros::VoidPtr(),
     &force_queue_);
  force2_sub_ = nh_.subscribe(force_ops2);

  force_spinner_.start();
}

void aero::interface::NonPrehensileGraspInterface::setBeginForce() {
  force_mutex_.lock();
  for (auto it = force_data4analysis_.begin(); // clear all data
       it != force_data4analysis_.end(); ++it)
    it->clear();
  force_analysis_ = true;
  force_mutex_.unlock();
}

void aero::interface::NonPrehensileGraspInterface::setEndForce() {
  force_mutex_.lock();
  force_analysis_ = false;
  force_mutex_.unlock();
}

float aero::interface::NonPrehensileGraspInterface::readLastForce(const int _num) {
  if (_num >= force_data4analysis_.size()
      || force_data4analysis_.at(_num).size() == 0) {
    ROS_WARN("cannot read last force of %d", _num);
    return 0.0;
  }

  force_mutex_.lock();
  float force = force_data4analysis_.at(_num).back().second.z();
  force_mutex_.unlock();

  return force;
}

float aero::interface::NonPrehensileGraspInterface::readMaxForce(const int _num) {
  if (_num >= force_data4analysis_.size()
      || force_data4analysis_.at(_num).size() == 0) {
    ROS_WARN("cannot read last force of %d", _num);
    return 0.0;
  }

  force_mutex_.lock();
  auto dat = std::max_element(force_data4analysis_.at(_num).begin(),
                              force_data4analysis_.at(_num).end(),
                              [&](std::pair<float, aero::Vector3> _a,
                                  std::pair<float, aero::Vector3> _b) {
                                return (_a.second.z() < _b.second.z());
                              });
  float force = dat->second.z();
  force_mutex_.unlock();

  return force;
}

void aero::interface::NonPrehensileGraspInterface::Force0Callback
(const geometry_msgs::Vector3::ConstPtr &_msg) {
  force_mutex_.lock();
  float timestamp = (ros::Time::now() - record_start_time_).toSec();
  if (record_force_)
    force_data_.at(0).push_back({timestamp, {_msg->x, _msg->y, _msg->z}});
  if (force_analysis_)
    force_data4analysis_.at(0).push_back({timestamp, {_msg->x, _msg->y, _msg->z}});
  thumb_force_z_ = _msg->z;
  force_mutex_.unlock();
}

void aero::interface::NonPrehensileGraspInterface::Force1Callback
(const geometry_msgs::Vector3::ConstPtr &_msg) {
  force_mutex_.lock();
  float timestamp = (ros::Time::now() - record_start_time_).toSec();
  if (record_force_)
    force_data_.at(1).push_back({timestamp, {_msg->x, _msg->y, _msg->z}});
  if (force_analysis_)
    force_data4analysis_.at(1).push_back({timestamp, {_msg->x, _msg->y, _msg->z}});
  index_force_z_ = _msg->z;
  force_mutex_.unlock();
}

void aero::interface::NonPrehensileGraspInterface::Force2Callback
(const geometry_msgs::Vector3::ConstPtr &_msg) {
  force_mutex_.lock();
  float timestamp = (ros::Time::now() - record_start_time_).toSec();
  if (record_force_)
    force_data_.at(2).push_back({timestamp, {_msg->x, _msg->y, _msg->z}});
  if (force_analysis_)
    force_data4analysis_.at(2).push_back({timestamp, {_msg->x, _msg->y, _msg->z}});
  force_mutex_.unlock();
}
