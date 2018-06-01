#ifndef AERO_NONOFFICIAL_API_UTIL_POSES_
#define AERO_NONOFFICIAL_API_UTIL_POSES_

#include "aero_nonofficial_api/AeroMotionPlanningInterface.hh"
#include "aero_nonofficial_api/utils.hh"

namespace aero
{
  namespace poses
  {
    /// @brief print the pose of the hand from robot state (used to find poses from prepared state)
    /// @param[in] _rm the robot interface for calculating the hand pose
    /// @param[in] _jm the state to set
    /// @param[in] _mpi the planning interface to display the pose
    /// @param[in] _eef used when finding the relative diffs between eef positions
    void viewPose(aero::interface::AeroMoveitInterface::Ptr _rm,
                  const std::map<aero::joint, double> _jm,
                  aero::interface::AeroMotionPlanningInterface::Ptr _mpi,
                  const aero::eef _eef=aero::eef::grasp) {
      _rm->setRobotStateVariables(_jm);
      _mpi->setCurrentState(_rm);
      _mpi->displayScene();
      aero::Transform trans_r, trans_l;
      _rm->getEndEffectorCoords(aero::arm::rarm, _eef, trans_r);
      _rm->getEndEffectorCoords(aero::arm::larm, _eef, trans_l);
      aero::Transform trans_rh, trans_lh;
      _rm->getEndEffectorCoords(aero::arm::rarm, aero::eef::hand, trans_rh);
      _rm->getEndEffectorCoords(aero::arm::larm, aero::eef::hand, trans_lh);
      aero::Quaternion q_r(trans_r.linear()), q_l(trans_l.linear()),
        q_rh(trans_rh.linear()), q_lh(trans_lh.linear());
      ROS_INFO("right(ref): (%f, %f, %f) (%f, %f, %f, %f)",
          trans_r.translation().x(), trans_r.translation().y(), trans_r.translation().z(),
          q_r.w(), q_r.x(), q_r.y(), q_r.z());
      ROS_INFO("left(ref): (%f, %f, %f) (%f, %f, %f, %f)",
          trans_l.translation().x(), trans_l.translation().y(), trans_l.translation().z(),
          q_l.w(), q_l.x(), q_l.y(), q_l.z());
      ROS_INFO("right(hand): (%f, %f, %f) (%f, %f, %f, %f)",
          trans_rh.translation().x(), trans_rh.translation().y(), trans_rh.translation().z(),
          q_rh.w(), q_rh.x(), q_rh.y(), q_rh.z());
      ROS_INFO("left(hand): (%f, %f, %f) (%f, %f, %f, %f)",
          trans_lh.translation().x(), trans_lh.translation().y(), trans_lh.translation().z(),
          q_lh.w(), q_lh.x(), q_lh.y(), q_lh.z());
    };

    /// @brief print the pose of the hand from robot state (used to find poses from prepared state)
    /// @param[in] _rm the robot interface containing the state to set
    /// @param[in] _mpi the planning interface to display the pose
    /// @param[in] _eef used when finding the relative diffs between eef positions
    void viewPose(aero::interface::AeroMoveitInterface::Ptr _rm,
                  aero::interface::AeroMotionPlanningInterface::Ptr _mpi,
                  const aero::eef _eef=aero::eef::grasp) {
      std::map<aero::joint, double> av;
      _rm->getRobotStateVariables(av);
      viewPose(_rm, av, _mpi, _eef);
    };

    /// @brief util for finding valid lifter range (rarely used unless lifter size is reconfigured)
    /// @param[in] _rm the robot interface to test
    /// @param[in] _resolution solve resolution to find range, small values take time
    /// @param[in] _min_z the z range to test from (finds valid range within this range)
    /// @param[in] _range_x the x range to test (finds valid range within this range)
    void findLifterMinMax(aero::interface::AeroMoveitInterface::Ptr _rm,
                          float _resolution=0.01, float _min_z=-0.301, float _range_x=0.251) {
      int min_z = _min_z / _resolution;
      int min_x = -_range_x / _resolution;
      int max_x = _range_x / _resolution;
      for (int z = min_z; z < 0; ++z) {
        int xlim_min = 0;
        int xlim_max = 0;
        for (int x = min_x; x <= max_x; ++x)
          if (_rm->setLifter(x * _resolution, z * _resolution)) {
            if (xlim_min >= 0)
              xlim_min = x;
            xlim_max = x;
          }
        ROS_INFO("for z=%d, range is [%d, %d]", z, xlim_min, xlim_max);
      }
    }

    /// @brief search lifter position that solves ik under environment collision
    /// @param[in] _rm the robot interface for finding position
    /// @param[in] _mpi the planning interface with environment collision
    /// @param[in] _arm the arm to solve ik
    /// @param[in] _eef the end effector position to solve ik
    /// @param[in] _lx the valid lifter x position
    /// @param[in] _lz the valid lifter z position
    /// @param[in] _resolution the resolution to search a solution
    /// @param[in] _range the list of pre-computed valid lifter ranges
    double last_found_x = 0.0;
    double last_found_z = 0.0;
    bool searchLifterFromIK(aero::interface::AeroMoveitInterface::Ptr _rm,
                            aero::interface::AeroMotionPlanningInterface::Ptr _mpi,
                            const aero::arm _arm, const aero::ikrange _ik_range,
                            const aero::Transform _goal, const aero::eef _eef,
                            double& _lx, double& _lz, const float _resolution=0.01,
                            const std::unordered_map<int, float> _range=aero::utils::range_c) {
      std::map<aero::joint, double> av;
      // int min_z = -0.301 / _resolution;
      int min_z = -0.491 / _resolution;
      _rm->setLifter(last_found_x, last_found_z); // try last answer
      if (_rm->setFromIK(_arm, _ik_range, _goal, _eef))
        if (!_mpi->checkCollision(_rm)) {
          ROS_INFO("found valid from seed (%f, %f)", last_found_x, last_found_z);
          _lx = last_found_x;
          _lz = last_found_z;
#if defined(DEBUG_)
          _mpi->setCurrentState(_rm);
          _mpi->displayScene();
          usleep(3000 * 1000);
#endif
          return true;
        }
      ROS_ERROR("failed to find from seed (%f, %f)", last_found_x, last_found_z);
      _rm->setLifter(0.0, 0.0); // begin full search
      for (int z = 0; z >= min_z; --z) {
        int index = static_cast<int>((z * _resolution - 0.001) * 100); // m -> cm
        int xlim = static_cast<int>(_range.at(index) / _resolution + 0.001);
        for (int x = xlim; x >= -xlim; --x) {
          _rm->setLifter(x * _resolution, z * _resolution);
          if (_rm->setFromIK(_arm, _ik_range, _goal, _eef)) {
#if defined(DEBUG_)
            _mpi->setCurrentState(_rm);
            _mpi->displayScene();
            usleep(3000 * 1000);
#endif
            if (!_mpi->checkCollision(_rm)) {
              ROS_INFO("found valid at (%f, %f)", x * _resolution, z * _resolution);
              _lx = x * _resolution;
              _lz = z * _resolution;
              last_found_x = _lx; // save for speed up
              last_found_z = _lz; // save for speed up
              return true;
            } else {
              ROS_ERROR("found valid at (%f, %f) but collides", x * _resolution, z * _resolution);
            }
          } else {
            ROS_ERROR("failed with [%d](%d, %d)->(%f, %f) in [(%d, %d), (%d, 0)]",
                      index, x, z, x * _resolution, z * _resolution, -xlim, xlim, min_z);
          }
        }
      }
      return false;
    };

    /// @brief utility function to calculate hand poses from index and thumb distance
    /// @param[in] _robot kinematic model
    /// @param[in] _arm arm to use
    /// @param[in] _d distance between fingers = index - thumb > 0
    /// @param[in] _omega hand open angle for hand
    /// @return successful or not
    bool quaternionFromHandParameters(aero::interface::AeroMoveitInterface::Ptr _robot,
                                      const aero::arm _arm, const float _d, const float _omega) {
      std::map<aero::joint, double> av;
      _robot->getRobotStateVariables(av);
      double grasp_r = _robot->getHand(aero::arm::rarm);
      double grasp_l = _robot->getHand(aero::arm::larm);

      // setup seed search pose
      _robot->setPoseVariables(aero::pose::reset_manip);
      _robot->setLifter(0.0, -0.25);
      _robot->setHand(_arm, _omega);
      aero::Translation pos(0.6, -0.03, 1.02); // magic seed parameter
      aero::Quaternion rot0;
      if (_arm == aero::arm::rarm)
        rot0 = aero::Quaternion(0.707107, -0.707107, 0.0, 0.0);
      else
        rot0 = aero::Quaternion(0.707107, 0.707107, 0.0, 0.0);
      if (!_robot->setFromIK(_arm, aero::ikrange::upperbody, pos * rot0, aero::eef::grasp)) {
        ROS_ERROR("could not solve seed pose!");
        return false;
      }

      // initiate variables
      bool found_solution = false;
      float dist = std::numeric_limits<float>::max();
      float index_x = _robot->getEEFPosition(_arm, aero::eef::index).x();
      float thumb_x = _robot->getEEFPosition(_arm, aero::eef::thumb).x();
      float pitch = 0.0;
      printf("start hand pose: %lf, %lf %lf\n", index_x, thumb_x, index_x - thumb_x);

      // evaluation function is different depending on finger position
      // note, finger position differs depending on _omega
      std::function<bool(float,float)> f;
      float thre = 0.0;
      if (index_x > thumb_x) {
        f = [&](float a, float b){ return (a > b); };
      } else {
        f = [&](float a, float b){ return (a < b); };
        thre = _d + 0.1;
      }

      // search desired hand pose by updating hand pitch value
      while (f((index_x - thumb_x), thre)) {
        dist = index_x - thumb_x;
        if (f(_d, dist)) {
          found_solution = true;
          break;
        }
        pitch += 0.01;
        if (!_robot->setFromIK
            (_arm, aero::ikrange::upperbody,
             pos * (aero::Quaternion(cos(-pitch), 0.0, sin(-pitch), 0.0) * rot0), aero::eef::grasp))
          break;
        index_x = _robot->getEEFPosition(_arm, aero::eef::index).x();
        thumb_x = _robot->getEEFPosition(_arm, aero::eef::thumb).x();
        printf("setting hand pose: %lf, %lf %lf\n", index_x, thumb_x, index_x - thumb_x);
      }

      if (found_solution) {
        aero::Transform trans_r, trans_rh;
        _robot->getEndEffectorCoords(_arm, aero::eef::index, trans_r);
        _robot->getEndEffectorCoords(_arm, aero::eef::hand, trans_rh);
        aero::Quaternion q_r(trans_r.linear()), q_rh(trans_rh.linear());
        ROS_INFO("ref: (%f, %f, %f) (%f, %f, %f, %f)",
                 trans_r.translation().x(), trans_r.translation().y(), trans_r.translation().z(),
                 q_r.w(), q_r.x(), q_r.y(), q_r.z());
        ROS_INFO("hand: (%f, %f, %f) (%f, %f, %f, %f)",
                 trans_rh.translation().x(), trans_rh.translation().y(), trans_rh.translation().z(),
                 q_rh.w(), q_rh.x(), q_rh.y(), q_rh.z());
      } else {
        ROS_ERROR("no pose solution found from hand parameters");
      }

#if defined(DEBUG_)
      _robot->sendHand(_arm, _omega);
      _robot->sendModelAngles(3000);
      _robot->waitInterpolation();
#endif

      _robot->setRobotStateVariables(av);
      _robot->setHand(aero::arm::rarm, grasp_r);
      _robot->setHand(aero::arm::larm, grasp_l);

      return found_solution;
    };

    /// @brief utility function to calculate hand poses from farthest index finger reach
    /// @param[in] _robot kinematic model
    /// @param[in] _arm arm to use
    /// @param[in] _omega hand open angle for hand
    /// @return successful or not
    bool quaternionFromHandParameters(aero::interface::AeroMoveitInterface::Ptr _robot,
                                      const aero::arm _arm, const float _omega) {
      std::map<aero::joint, double> av;
      _robot->getRobotStateVariables(av);
      double grasp_r = _robot->getHand(aero::arm::rarm);
      double grasp_l = _robot->getHand(aero::arm::larm);

      // setup seed search pose
      _robot->setPoseVariables(aero::pose::reset_manip);
      _robot->setLifter(0.0, -0.25);
      _robot->setHand(_arm, _omega);
      aero::Translation pos(0.6, -0.03, 1.02); // magic seed parameter
      aero::Quaternion rot0;
      if (_arm == aero::arm::rarm)
        rot0 = aero::Quaternion(0.707107, -0.707107, 0.0, 0.0);
      else
        rot0 = aero::Quaternion(0.707107, 0.707107, 0.0, 0.0);
      if (!_robot->setFromIK(_arm, aero::ikrange::upperbody, pos * rot0, aero::eef::grasp)) {
        ROS_ERROR("could not solve seed pose!");
        return false;
      }

      // initiate variables
      float index_x = _robot->getEEFPosition(_arm, aero::eef::index).x();
      float pitch = 0.0;
      float best_pitch = 0.0;
      float max_reach = std::numeric_limits<float>::min();
      printf("start hand pose: %lf\n", index_x);

      // search desired hand pose by updating hand pitch value
      while (1) {
        if (index_x > max_reach) {
          max_reach = index_x;
          best_pitch = pitch;
        }
        pitch += 0.01;
        if (!_robot->setFromIK
            (_arm, aero::ikrange::upperbody,
             pos * (aero::Quaternion(cos(-pitch), 0.0, sin(-pitch), 0.0) * rot0), aero::eef::grasp))
          break;
        index_x = _robot->getEEFPosition(_arm, aero::eef::index).x();
        printf("setting hand pose: %f, best: %f(%f)\n", index_x, max_reach, best_pitch);
      }

      if (pitch < 0.3) { // not enough search
        ROS_ERROR("no pose solution found from hand parameters");
        return false;
      }

      _robot->setFromIK
        (_arm, aero::ikrange::upperbody,
         pos * (aero::Quaternion(cos(-best_pitch), 0.0, sin(-best_pitch), 0.0) * rot0),
         aero::eef::grasp);

      aero::Transform trans_r, trans_rh;
      _robot->getEndEffectorCoords(_arm, aero::eef::index, trans_r);
      _robot->getEndEffectorCoords(_arm, aero::eef::hand, trans_rh);
      aero::Quaternion q_r(trans_r.linear()), q_rh(trans_rh.linear());
      ROS_INFO("ref: (%f, %f, %f) (%f, %f, %f, %f)",
               trans_r.translation().x(), trans_r.translation().y(), trans_r.translation().z(),
               q_r.w(), q_r.x(), q_r.y(), q_r.z());
      ROS_INFO("hand: (%f, %f, %f) (%f, %f, %f, %f)",
               trans_rh.translation().x(), trans_rh.translation().y(), trans_rh.translation().z(),
               q_rh.w(), q_rh.x(), q_rh.y(), q_rh.z());

#if defined(DEBUG_)
      _robot->sendHand(_arm, _omega);
      _robot->sendModelAngles(3000);
      _robot->waitInterpolation();
#endif

      _robot->setRobotStateVariables(av);
      _robot->setHand(aero::arm::rarm, grasp_r);
      _robot->setHand(aero::arm::larm, grasp_l);

      return true;
    };

  }
}

#endif
