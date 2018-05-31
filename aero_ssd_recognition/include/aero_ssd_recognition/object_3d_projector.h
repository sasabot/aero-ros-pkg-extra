#ifndef AERO_OBJECT_DETECTOR_OBJECT_3D_PROJECTOR_H_
#define AERO_OBJECT_DETECTOR_OBJECT_3D_PROJECTOR_H_

#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <boost/assign.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <aero_recognition_msgs/Scored2DBoxArray.h>
#include <dynamic_reconfigure/server.h>
#include <aero_ssd_recognition/Object3DProjectorConfig.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/geometry.h>
#include <image_geometry/pinhole_camera_model.h>

#if defined(KINETIC_)
#include <opencv2/core.hpp>
#elif defined(INDIGO_)
#include <opencv2/core/core.hpp>
#endif

#include <aero_recognition_msgs/LabeledPoseArray.h>
#include <geometry_msgs/PoseArray.h>

namespace aero_ssd_recognition
{
#if defined(RGBA_)
  typedef pcl::PointXYZRGBA pcltype;
#else
  typedef pcl::PointXYZ pcltype;
#endif

  class Object3DProjector: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    typedef aero_ssd_recognition::Object3DProjectorConfig Config;
    typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::PointCloud2,
      aero_recognition_msgs::Scored2DBoxArray,
      sensor_msgs::CameraInfo,
      sensor_msgs::Image > ApproximateSyncPolicy;
    typedef message_filters::sync_policies::ExactTime<
      sensor_msgs::PointCloud2,
      aero_recognition_msgs::Scored2DBoxArray,
      sensor_msgs::CameraInfo,
      sensor_msgs::Image > SyncPolicy;

    enum struct Mode{
      Simple,
      Nearest,
      NearestCenter,
      SimpleForSandwich
    };

    Object3DProjector(): DiagnosticNodelet("Object3DProjector") {}
  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void configCallback(Config &config, uint32_t level);
    virtual void apply(const sensor_msgs::PointCloud2::ConstPtr& points_msg,
                       const aero_recognition_msgs::Scored2DBoxArray::ConstPtr& boxes_msg,
                       const sensor_msgs::CameraInfo::ConstPtr& info_msg,
                       const sensor_msgs::Image::ConstPtr& img_msg);
    virtual void getDistanceSortedPoints(const pcl::PointCloud<pcltype>::Ptr cloud,
                                         std::vector<pcltype>& sorted_points,
                                         aero_recognition_msgs::Scored2DBox& box);
    virtual double distance(const pcltype& a, const pcltype& b);

    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_cloud_;
    message_filters::Subscriber<aero_recognition_msgs::Scored2DBoxArray> sub_box_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> sub_info_;
    message_filters::Subscriber<sensor_msgs::Image> sub_img_;
    ros::Publisher pub_;
    ros::Publisher debug_pub_;
    ros::Publisher image_pub_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;
    boost::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy> > async_;
    boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;

    //param
    bool approximate_sync_;
    int wsize_;
    int step_;
    Mode mode_;

  private:

  };
}

#endif
