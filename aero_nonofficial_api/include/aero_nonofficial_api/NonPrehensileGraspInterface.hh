#ifndef NON_PREHENSILE_GRASP_INTERFACE_
#define NON_PREHENSILE_GRASP_INTERFACE_

#include <aero_std/AeroMoveitInterface.hh>
#include "aero_nonofficial_api/utils_poses.hh"

#include <thread>

#if defined(USE_GAZEBO_SIM_)
#include "aero_benchmarks/utils_gazebo.hh"
#endif

namespace aero
{
  /// @brief hand angles included trajectory
  struct joint_solution {
    moveit_msgs::MotionPlanResponse joints;
    double grasp_r;
    double grasp_l;
  };

  /// @brief grasp_definitions are simple ways for coding complex grasps
  ///     why not use functions? in order to group and limit the functions that are executable
  namespace grasp_definition {
    // transformoffset = lifteroffset + change hand pose
    enum struct types : int {transform, transformoffset, lifteroffset, gripper, force, readlifter};
    enum struct actions : int {touch, pull, release};

    struct forcedef {
      forcedef() {}
      forcedef(actions _action, float _threshold, int _time, float _move_limit)
      { action = _action; force_threshold = _threshold; time = _time; move_limit = _move_limit; }

      actions action;
      float force_threshold;
      int time;
      float move_limit;
    };

    struct definition {
      definition(const types _type) { type = _type; }
      definition(const std::pair<aero::arm, aero::Transform> _pose, const bool _save_lifter)
      { type = types::transform; pose = _pose; save_lifter = _save_lifter; }
      definition(const std::pair<double, double> _lifteroffset,
                 const int _time, const bool _save_lifter)
      { type = types::lifteroffset;
        lifteroffset = _lifteroffset; time = _time; save_lifter = _save_lifter; }
      definition(const std::pair<aero::arm, aero::Transform> _pose,
                 const std::pair<double, double> _lifteroffset, const int _time,
                 const bool _save_lifter)
      { type = types::transformoffset;
        pose = _pose; lifteroffset = _lifteroffset; time = _time; save_lifter = _save_lifter; }
      definition(const std::pair<aero::arm, double> _gripper, const bool _save_lifter)
      { type = types::gripper; gripper = _gripper; save_lifter = _save_lifter; }
      definition(const forcedef _force, const bool _save_lifter)
      { type = types::force; force = _force; save_lifter = _save_lifter; }

      types type;
      std::pair<aero::arm, aero::Transform> pose;
      std::pair<double, double> lifteroffset; // first: x, second: z
      std::pair<aero::arm, double> gripper;
      forcedef force;
      int time;
      bool save_lifter;
    };
  }



  namespace interface
  {
    /// @brief interface for executing complex grasps, including touch sensing
    ///     has general functions for complex grasps
    ///     actual grasp execution script should be written in child classes
    class NonPrehensileGraspInterface
    {
    public: typedef std::shared_ptr<NonPrehensileGraspInterface> Ptr;

    /// @brief detailed results of grasp execution when force is used
    public: struct details {
      bool status;
      float fu; // index
      float fmax;
      float fg; // thumb
      float diff;
    };

    /// @brief constructor
    /// @param[in] _nh ros node handler
    public: explicit NonPrehensileGraspInterface(ros::NodeHandle &_nh);
    public: ~NonPrehensileGraspInterface();

    /// @brief execute a grasp definition
    /// @param[in] _robot the robot that executes the grasp
    /// @param[in] _def the definition of the grasp
    /// @return whether the execution was successful
    public: bool executeGraspDefinition(aero::interface::AeroMoveitInterface::Ptr _robot,
                                        const aero::grasp_definition::definition _def);

    /// @brief solve collision avoided reaching trajectory
    /// @param[in] _robot robot under manipulation
    /// @param[in] _mpi planner instance
    /// @param[in] _arm manipulating arm
    /// @param[in] _target x=object frontal surface, y=center, z=object top,
    ///     quaternion should be pre-calculated depending on target state at hand coordinate
    /// @param[in] _joint solve in joint space instead of target space
    /// @return whether plan was successful or not
    public: bool plan_(aero::interface::AeroMoveitInterface::Ptr _robot,
                       aero::interface::AeroMotionPlanningInterface::Ptr _mpi,
                       const aero::arm _arm, aero::Transform _target,
                       const bool _joint=false);

    /// @brief execute reaching trajectory (must be called after plan_)
    /// @param[in] _robot robot under manipulation
    /// @param[in] _mpi planner instance
    /// @return always true
    public: bool execute_(aero::interface::AeroMoveitInterface::Ptr _robot,
                          aero::interface::AeroMotionPlanningInterface::Ptr _mpi);

    /// @brief move lifter until force is detected
    /// @param[in] _robot the robot that moves its lifter
    /// @param[in] _force the force threshold to detect
    /// @param[in] _t how fast to move the lifter [ms]
    /// @param[in] _x maximum x position to move [m]
    /// @param[in] _z maximum z position to move [m]
    /// @param[in] _ref_force variable to save the detected force [N]
    /// @return false if maximum movement is not reachable
    public: bool touchTillForceDetection
    (aero::interface::AeroMoveitInterface::Ptr _robot, const float _force, const int _t,
     const double _x, const double _z, float* _ref_force);

    protected: ros::NodeHandle nh_;

    /// @brief initial reaching offset
    protected: aero::Vector3 offset_;

    /// @brief solved reach trajectory
    protected: joint_solution solution_;

    protected: aero::arm arm_;

    protected: details details_;

    protected: std::pair<double, double> saved_lifter_;

    // from here, for touch sensing

    /// @brief setup subscribers etc. for touch sensing
    protected: void setupTouchSensing();

    /// @brief calibrate the touch sensor (detached on thread)
    public: void calibrate();

    /// @brief wait till touch sensor calibration has finished
    public: bool waitCalibrate();

    /// @brief start recording touch sensor values for executing grasp
    protected: void setBeginForce();

    /// @brief stop recording touch sensor values for executing grasp
    protected: void setEndForce();

    /// @brief return the last recorded(most likely current) value before setEndForce
    protected: float readLastForce(int _num);

    /// @brief return the maximum recorded value between setBeginForce and setEndForce
    protected: float readMaxForce(int _num);

    protected: ros::CallbackQueue force_queue_;

    protected: ros::AsyncSpinner force_spinner_;

    protected: ros::ServiceClient calibration_client_;

    protected: std::mutex calibration_mutex_;

    protected: int sensor_calibrated_;

    /// @brief used when saving log for plot
    protected: ros::Time record_start_time_;

    protected: std::mutex force_mutex_;

    protected: float thumb_force_z_;

    protected: float index_force_z_;

    /// @brief record force values to save as data
    protected: bool record_force_;

    /// @brief record force values at specific region for analysis
    protected: bool force_analysis_;

    /// @brief recorded force values to save as data (size > force_data4analysis_)
    protected: std::vector<std::vector<std::pair<float, aero::Vector3>>> force_data_;

    /// @brief recorded force values at specific region for analysis
    protected: std::vector<std::vector<std::pair<float, aero::Vector3>>> force_data4analysis_;

    /// @brief subscriber for force value on the thumb
    protected: ros::Subscriber force0_sub_;

    /// @brief subscriber for force value on one of the index fingers
    protected: ros::Subscriber force1_sub_;

    protected: ros::Subscriber force2_sub_;

    private: void Force0Callback(const geometry_msgs::Vector3::ConstPtr &_msg);

    private: void Force1Callback(const geometry_msgs::Vector3::ConstPtr &_msg);

    private: void Force2Callback(const geometry_msgs::Vector3::ConstPtr &_msg);
    };



    /// @brief grasp execution script for tumbling and sliding objects
    class npgrasp_standard : public NonPrehensileGraspInterface {
    public: typedef std::shared_ptr<npgrasp_standard> ptr;

    public: enum npposetype : int {tumble_or_lowslide, lowslide, tumble_deep, tumble_verydeep};
    public: enum nptranstype : int
      {tumble2lowslide, tumble2deep, deep2verydeep, deep2lowslide, verydeep2lowslide};
    public: enum npgrasptype : int
      {tumblev, tumbleforce, fail, highslidev, highslideforce, lowslidev, lowslideforce};

    public: npgrasp_standard(ros::NodeHandle& _nh,
                             const aero::interface::npgrasp_standard::npgrasptype _type);
    public: ~npgrasp_standard();

    // /// @brief plan tumble or slide with recovery
    // /// @param[in] _targets map should include as much nptype as possible
    // public: void plan(const std::map<npposetype, aero::Transform> _targets);

    public: bool execute(aero::interface::AeroMoveitInterface::Ptr _robot,
                         aero::interface::AeroMotionPlanningInterface::Ptr _mpi);

    protected: npgrasptype type_;

    /// @brief transition between different hand poses
    protected: std::map<nptranstype, std::pair<bool, joint_solution > >
    planned_trajectories_;
    };



    /// @brief grasp execution script for other types of grasp (no recovery)
    class npgrasp_else : public NonPrehensileGraspInterface {
    public: typedef std::shared_ptr<npgrasp_else> ptr;

    public: enum npgrasptype : int {bottle, carton};

    public: npgrasp_else(ros::NodeHandle& _nh,
                         const aero::interface::npgrasp_else::npgrasptype _type);
    public: ~npgrasp_else();

    public: bool execute(aero::interface::AeroMoveitInterface::Ptr _robot,
                         aero::interface::AeroMotionPlanningInterface::Ptr _mpi);

    protected: npgrasptype type_;
    };
  }
}

#endif
