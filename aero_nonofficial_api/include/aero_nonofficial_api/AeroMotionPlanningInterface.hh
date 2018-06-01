#ifndef AERO_MOTION_PLANNING_INTERFACE_
#define AERO_MOTION_PLANNING_INTERFACE_

#include <pluginlib/class_loader.h>
#include <aero_std/AeroMoveitInterface.hh>

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/collision_detection/collision_tools.h>

#include <boost/scoped_ptr.hpp>

#include <visualization_msgs/Marker.h>
#include <eigen_conversions/eigen_msg.h>

namespace aero
{
  namespace interface
  {
    class AeroMotionPlanningInterface
    {
    public: typedef std::shared_ptr<AeroMotionPlanningInterface> Ptr;

    /// @brief constructor
    /// @param[in] _nh ros node handler
    /// @param[in] _rm robot model
    /// @param[in] _plugin planner plugin to use (only OMPL for now)
    public: explicit AeroMotionPlanningInterface
    (ros::NodeHandle &_nh, const robot_model::RobotModelConstPtr _rm, const std::string _plugin="ompl_interface/OMPLPlanner");
    public: ~AeroMotionPlanningInterface();

    /// @brief set start state of plan (note, links in model will be updated)
    /// @param[in] robot interface with kinematic model
    public: void setCurrentState(aero::interface::AeroMoveitInterface::Ptr _robot);

    /// @brief transform aero::Transform to transform in move_group eef (aero::eef::hand)
    /// @param[in] _robot robot interface with kinematic model
    /// @param[in] _group_name name of move_group
    /// @param[in] _arm arm of transform
    /// @param[in] _eef end effector link of transform
    /// @param[in] _mat aero::Transform to transform
    /// @return transformed aero::Transform
    public: aero::Transform getTransformInMoveGroupEEF
    (aero::interface::AeroMoveitInterface::Ptr _robot, const std::string _group_name, const aero::arm _arm, const aero::eef _eef, const aero::Transform _mat);

    /// @brief set color of objects that are going to be pushed
    /// @param[in] _r red
    /// @param[in] _g green
    /// @param[in] _b blue
    /// @param[in] _a alpha
    public: void setDefaultColor(const float _r, const float _g, const float _b, const float _a=1.0f);

    /// @brief add box to scene for collision
    /// @param[in] _id name the box
    /// @param[in] _parent where the box should be attached to e.g. map, base_link
    /// @param[in] _pose position and orientation of box from parent link
    /// @param[in] _scale size of box in meters
    public: void processCollisionBox(const std::string _id, const std::string _parent, const aero::Transform _pose, const aero::Vector3 _scale);

    /// @brief add mesh to scene for collision
    /// @param[in] _id name the mesh
    /// @param[in] _parent where the mesh should be attached to e.g. map, base_link
    /// @param[in] _pose position and orientation of mesh from parent link
    /// @param[in] _resource mesh file name
    /// @param[in] _scale scale the mesh to whatever wanted size
    public: void processCollisionMesh(const std::string _id, const std::string _parent, const aero::Transform _pose, const std::string _resource, const aero::Vector3 _scale=aero::Vector3(1.0, 1.0, 1.0));

    /// @brief remove collision object from scene
    /// @param[in] _id name of object
    public: void remove(const std::string _object);

    // public: void processAttachedCollisionMesh(const std::string _link_name, const std::string _parent, const aero::Transform _pose, const std::string _resource, const std::vector<std::string> _touch_links);

    /// @brief create goal state for request from current robot joint state
    /// @param[in] _robot robot interface with kinematic model
    /// @param[in] _group_name name of move_group
    /// @param[in] _tolerance1 pose tolerance
    /// @param[in] _tolerance2 angle tolerance
    /// @return constraints to push in request      
    public: moveit_msgs::Constraints constructGoalConstraints
    (aero::interface::AeroMoveitInterface::Ptr _robot, std::string _group_name, const double _tolerance1=0.03, const double _tolerance2=0.03);

    /// @brief create path constraint state for request from pose
    /// @param[in] _origin name of group to do manipulation
    /// @param[in] _goal goal pose of goal state
    /// @param[in] _arm arm to solve (note, goal state always refers to aero::eef::hand)
    /// @param[in] _tolerance1 pose tolerance
    /// @param[in] _tolerance2 angle tolerance
    /// @return constraints to push in request
    public: moveit_msgs::Constraints constructGoalConstraints
    (const std::string _origin, const aero::Transform _goal, const aero::arm _arm, const double _tolerance1=0.03, const double _tolerance2=0.03);

    /// @brief solve the motion plan
    /// @param[in] _req motion plan settings
    /// @param[in] _res solved trajectory and status
    /// @return whether successful or not
    public: bool plan
    (planning_interface::MotionPlanRequest& _req, planning_interface::MotionPlanResponse& _res);

    /// @brief search for a solution path _trials times and return calculation time
    /// @param[in] _robot robot interface with kinematic model
    /// @param[in] _res solved trajectory and status
    /// @param[in] _t how much time it took for calculation
    /// @param[in] _trials number of tries to do
    /// @return succeeded or not
    public: bool plan(planning_interface::MotionPlanRequest& _req, moveit_msgs::MotionPlanResponse &_res, double &_t, int _trials=10);

    /// @brief send the motion plan
    /// @param[in] _robot robot interface with kinematic model
    /// @param[in] _res solved trajectory and status
    /// @param[in] _duration time to finish whole trajectory in milliseconds
    /// @param[in] _ikrange this should match the range used for solving motion plan
    /// @param[in] _async false to wait interpolation
    /// @return succeed or not
    public: bool execute(aero::interface::AeroMoveitInterface::Ptr _robot, const planning_interface::MotionPlanResponse _res, const int _duration, const aero::ikrange _ikrange, const bool _async=true);

    /// @brief send the motion plan
    /// @param[in] _robot robot interface with kinematic model
    /// @param[in] _res solved trajectory and status
    /// @param[in] _duration time to finish whole trajectory in milliseconds
    /// @param[in] _ikrange this should match the range used for solving motion plan
    /// @param[in] _async false to wait interpolation
    /// @return succeed or not
    public: bool execute(aero::interface::AeroMoveitInterface::Ptr _robot, const moveit_msgs::MotionPlanResponse _res, const int _duration, const aero::ikrange _ikrange, const bool _async=true);

    /// @brief mirror solved plan on one arm to the other
    /// @param[in] _res solved trajectory and status to mirror from and to
    public: void mirrorPlan(moveit_msgs::MotionPlanResponse& _res);

    /// @brief check collision between robot state and environment
    /// @param[in] _robot robot interface with robot state
    /// @return whether there was collision or not (visual /moveit/collision/contacts)
    public: bool checkCollision(aero::interface::AeroMoveitInterface::Ptr _rm);

    /// @brief check collision for fast-computed approximate paths
    /// @param[in] _robot robot interface with kinematic model
    /// @param[in] _res solved trajectory and status
    /// @return whether there was collision or not (visual /moveit/collision/contacts)
    public: bool checkCollision(const aero::interface::AeroMoveitInterface::Ptr _robot, const planning_interface::MotionPlanResponse _res);

    /// @brief set robot state to end state trajectory of solved trajectory
    /// @param[in] _robot robot interface to change state
    /// @param[in] _goal the response containing solved trajectory
    public: void setStateToTrajectoryEnd(aero::interface::AeroMoveitInterface::Ptr _robot, const moveit_msgs::MotionPlanResponse _goal);

    /// @brief fill in _req with default workspace parameters
    /// @param[in] _req plan request to fill.
    public: inline void fillWorkspace(planning_interface::MotionPlanRequest &_req) {
      _req.workspace_parameters.min_corner.x = -2.0;
      _req.workspace_parameters.min_corner.y = -2.0;
      _req.workspace_parameters.min_corner.z = 0.0;
      _req.workspace_parameters.max_corner.x = 2.0;
      _req.workspace_parameters.max_corner.y = 2.0;
      _req.workspace_parameters.max_corner.z = 10.0;
    };

    /// @brief publish objects added to scene
    public: void displayScene();

    /// @brief display trajectory to rviz
    /// @param[in] _res MotionPlanResponse with solved trajectory
    public: void displayTrajectory(const planning_interface::MotionPlanResponse _res);

    /// @brief display trajectory to rviz
    /// @param[in] _res MotionPlanResponse with solved trajectory
    public: void displayTrajectory(const moveit_msgs::MotionPlanResponse _res);

    public: boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;

    public: planning_scene::PlanningScenePtr planning_scene;

    public: planning_interface::PlannerManagerPtr planner_manager;

    public: moveit_msgs::PlanningScene scene_msg;

    private: moveit_msgs::ObjectColor color_;

    private: ros::Publisher display_publisher_;

    private: ros::Publisher scene_publisher_;

    public: ros::Publisher contact_publisher_;

    public: ros::Publisher path_constraint_publisher_;
    };
  }
}

#endif
