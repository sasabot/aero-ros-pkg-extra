#include "aero_nonofficial_api/AeroMotionPlanningInterface.hh"
#include "aero_nonofficial_api/utils_poses.hh"
#include "aero_nonofficial_api/utils_recognition.hh"

int main(int argc, char **argv)
{
  // ------------------ robot setup ------------------

  // init ros
  ros::init(argc, argv, "bento_example");
  ros::NodeHandle nh;

  // init robot interface
  aero::interface::AeroMoveitInterface::Ptr robot(new aero::interface::AeroMoveitInterface(nh));
  // init planning scene
  aero::interface::AeroMotionPlanningInterface::Ptr planning
    (new aero::interface::AeroMotionPlanningInterface(nh, robot->kinematic_model));
  // init camera
  aero::recognition::setCameraTransform
    (robot, "head_base_link",
     aero::Translation(0.06, 0.04, 0.08) * aero::Quaternion(0.5, -0.5, 0.5, -0.5));
  aero::recognition::initialize(nh);
#if defined(USE_SIM_)
  ros::Publisher object_pub =
    nh.advertise<aero_recognition_msgs::LabeledPoseArray>("/object_3d_projector/output", 1);
#endif

  // define robot start pose
  robot->setPoseVariables(aero::pose::reset_manip);
  robot->setLifter(0.1, -0.2);
  robot->setHand(aero::arm::rarm, 0.55);
  robot->setHand(aero::arm::larm, -0.55);
  robot->sendModelAngles(3000);
  robot->sendHand(aero::arm::rarm, 0.55);
  robot->sendHand(aero::arm::larm, -0.55);
  robot->waitInterpolation();



  // ------------------ environment setup before vision ------------------

  // bento parameters
  const aero::Vector3 bento_bb(0.150, 0.250, 0.05);
  const aero::Vector3 bento_offset(0.0, -bento_bb.y() * 0.5 + 0.01, 0.03);
  const aero::Vector3 container(0.4, 0.0, 0.65);
  const aero::Vector3 grasp_to_hand(0.0, -0.077782, 0.035357); // pre-computed aero::poses::viewPoses
  const aero::Vector3 bento_ref(container + aero::Vector3(0.0, 0.0, 0.05));

  // show objects on rviz
  planning->remove("container");
  planning->remove("bento");
  planning->setCurrentState(robot); // set start state
  planning->setDefaultColor(0.0f, 1.0f, 0.0f);
  planning->processCollisionMesh // add container which is not set from vision in this sample
    ("container", "base_link", aero::Translation(container) * aero::Quaternion(1.0, 0.0, 0.0, 0.0),
     "package://aero_nonofficial_api/models/container.dae");



  // ------------------ environment setup from vision ------------------

  // find object
  aero::recognition::start(bento_ref);
  while (!aero::recognition::object_found) {
    ros::spinOnce();
#if defined(USE_SIM_)
    aero::Transform actual_coords; // map coords
    if (!robot->getCurrentCoords(actual_coords, "/map")) {
      ROS_WARN("failed to transform from /map, setting identical");
      actual_coords = aero::Translation(0.0, 0.0, 0.0) * aero::Quaternion(1.0, 0.0, 0.0, 0.0);
    }
    aero::Quaternion actual_qua(actual_coords.linear());
    aero::Vector3 actual_pos = // base_link coords
      actual_qua.inverse() * (bento_ref + aero::Vector3(0.0, 0.0, bento_bb.z()*0.5));
    aero_recognition_msgs::LabeledPose posemsg;
    tf::poseEigenToMsg
      (aero::Translation(aero::recognition::base2Camera(actual_pos, false))
       * aero::Quaternion(1.0, 0.0, 0.0, 0.0), posemsg.pose);
    aero_recognition_msgs::LabeledPoseArray arrmsg;
    arrmsg.poses.push_back(posemsg);
    object_pub.publish(arrmsg);
#endif
  }
  aero::recognition::finish();

  // show result on rviz
  aero::recognition::object_pos -= aero::Vector3(0.0, 0.0, bento_bb.z()*0.5); // surface -> center
  planning->setDefaultColor(1.0f, 1.0f, 0.0f);
  planning->processCollisionMesh
    ("bento", "base_link", // objects are recognized from base
     aero::Translation(aero::recognition::object_pos) * aero::Quaternion(1.0, 0.0, 0.0, 0.0),
     "package://aero_nonofficial_api/models/bento.dae");
  planning->displayScene();



  // ------------------ solve manipulation ------------------

  // left arm is mirrored, set solve pose for right
  // note, in this example, we assume the bento is in the center of the container
  aero::Transform goal_r =
    aero::Translation(aero::recognition::object_pos + bento_offset + grasp_to_hand)
    * aero::Quaternion(0.923879, 0.382684, 0.0, 0.0);

  // search for a good lifter position
  double lx, lz;
  double ik_resolution = 0.05;
  if (!aero::poses::searchLifterFromIK
      (robot, planning, aero::arm::rarm, aero::ikrange::arm, goal_r, aero::eef::hand,
       lx, lz, ik_resolution, aero::utils::range_c)) {
    ROS_ERROR("failed I.K.");
    return 0;
  }
  robot->setLifter(lx, lz);
  robot->setFromIK(aero::arm::rarm, aero::ikrange::arm, goal_r, aero::eef::hand);

  // plan in joint space
  planning_interface::MotionPlanRequest req;
  moveit_msgs::MotionPlanResponse res;
  req.group_name = aero::moveGroup(aero::arm::rarm, aero::ikrange::arm_lifter);
  moveit_msgs::Constraints pose_goalj =
    planning->constructGoalConstraints(robot, req.group_name, 0.03, 0.03);
  req.goal_constraints.push_back(pose_goalj);
  planning->fillWorkspace(req);
  req.planner_id = "BKPIECEkConfigDefault"; //"RRTstarkConfigDefault";
  req.allowed_planning_time = 10.0;
  double t0; // get calculation time
  int trials = 10; // how times to try solving
  if (!planning->plan(req, res, t0, trials)) {
    ROS_ERROR("failed plan");
    return 0;
  }
  planning->mirrorPlan(res); // copy solution of right arm on left

  // send plan and display
  planning->execute(robot, res, 200*res.trajectory.joint_trajectory.points.size(),
                    aero::ikrange::arm_lifter);
  planning->displayTrajectory(res);
  robot->waitInterpolation();

  ROS_INFO("demo node finished");
  ros::shutdown();

  return 0;
}
