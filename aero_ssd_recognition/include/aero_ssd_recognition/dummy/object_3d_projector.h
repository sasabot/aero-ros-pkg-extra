#ifndef AERO_OBJECT_DETECTOR_DUMMY_OBJECT_3D_PROJECTOR_H_
#define AERO_OBJECT_DETECTOR_DUMMY_OBJECT_3D_PROJECTOR_H_

#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <boost/assign.hpp>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <aero_recognition_msgs/Scored2DBoxArray.h>
#include <aero_recognition_msgs/LabeledPoseArray.h>
#include <geometry_msgs/PoseArray.h>

namespace aero_ssd_recognition
{
  class DummyObject3DProjector: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    DummyObject3DProjector(): DiagnosticNodelet("DummyObject3DProjector") {}
  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void apply(const aero_recognition_msgs::Scored2DBoxArray::ConstPtr& boxes_msg);

    ros::Subscriber sub_box_;
    ros::Publisher pub_;
    ros::Publisher debug_pub_;


    //param
    bool approximate_sync_;

  private:

  };
}

#endif
