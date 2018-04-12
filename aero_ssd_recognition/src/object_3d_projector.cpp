#include <aero_ssd_recognition/object_3d_projector.h>

namespace aero_ssd_recognition
{

  void Object3DProjector::onInit()
  {
    DiagnosticNodelet::onInit();
    pnh_->param("approximate_sync", approximate_sync_, true);
    pub_ = advertise<aero_recognition_msgs::LabeledPoseArray>(*pnh_, "output", 1);
    debug_pub_ = advertise<geometry_msgs::PoseArray>(*pnh_, "debug_output", 1);
    image_pub_ = advertise<sensor_msgs::Image>(*pnh_, "image", 1);
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&Object3DProjector::configCallback, this, _1, _2);
    srv_->setCallback (f);
    onInitPostProcess();
  }

  void Object3DProjector::subscribe()
  {
    sub_cloud_.subscribe(*pnh_, "input_cloud", 10);
    sub_box_.subscribe(*pnh_, "input_boxes", 10);
    sub_info_.subscribe(*pnh_, "input_info", 10);
    sub_img_.subscribe(*pnh_, "input_image", 10);
    ros::V_string names = boost::assign::list_of("~input_cloud")("~input_boxes")("~input_info")("~input_image");
    jsk_topic_tools::warnNoRemap(names);

    if (approximate_sync_) {
      async_ = boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy> >(5000);
      async_->connectInput(sub_cloud_, sub_box_, sub_info_, sub_img_);
      async_->registerCallback(boost::bind(&Object3DProjector::apply, this, _1, _2, _3, _4));
    }
    else {
      sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
      sync_->connectInput(sub_cloud_, sub_box_, sub_info_, sub_img_);
      sync_->registerCallback(boost::bind(&Object3DProjector::apply, this, _1, _2, _3, _4));
    }
  }

  void Object3DProjector::unsubscribe()
  {
    sub_cloud_.unsubscribe();
    sub_box_.unsubscribe();
    sub_info_.unsubscribe();
    sub_img_.unsubscribe();
  }

  void Object3DProjector::configCallback(Config &config, uint32_t level){

    wsize_ = config.window_size;
    step_ = config.search_step;
    if(step_ > wsize_){
      step_ = wsize_;
    }

    if(config.mode == 0 && mode_ != Object3DProjector::Mode::Simple){
      mode_ = Object3DProjector::Mode::Simple;
      ROS_INFO("Set Projection Mode : SIMPLE");
    } else if(config.mode == 1 && mode_ != Object3DProjector::Mode::SimpleForSandwich){
      mode_ = Object3DProjector::Mode::SimpleForSandwich;
      ROS_INFO("Set Projection Mode : SANDWICH");
    } else if(config.mode == 2 && mode_ != Object3DProjector::Mode::Nearest){
      mode_ = Object3DProjector::Mode::Nearest;
      ROS_INFO("Set Projection Mode : NEAREST");
    } else if(config.mode == 3 && mode_ != Object3DProjector::Mode::NearestCenter){
      mode_ = Object3DProjector::Mode::NearestCenter;
      ROS_INFO("Set Projection Mode : NEAREST CENTER");
    }
  }

  void Object3DProjector::apply(const sensor_msgs::PointCloud2::ConstPtr& points_msg,
                                const aero_recognition_msgs::Scored2DBoxArray::ConstPtr& boxes_msg,
                                const sensor_msgs::CameraInfo::ConstPtr& info_msg,
                                const sensor_msgs::Image::ConstPtr& img_msg)
  {
    if(points_msg->height != info_msg->height ||
       points_msg->width != info_msg->width){
      ROS_ERROR("Invalid PointCloud shape!");
      return;
    }

    // setup input data
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::fromROSMsg(*points_msg, *cloud);
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img_msg, img_msg->encoding);
    cv::Mat debug_img = cv_ptr->image;

    image_geometry::PinholeCameraModel model;
    if (!model.fromCameraInfo(info_msg)) {
      return;
    }

    // output messages
    aero_recognition_msgs::LabeledPoseArray poses_out_msg;
    geometry_msgs::PoseArray poses_debug_msg;
    poses_out_msg.header = boxes_msg->header;
    poses_debug_msg.header = boxes_msg->header;

    for(auto box : boxes_msg->boxes){
      ROS_INFO("projection    : %s", box.label.c_str());

      int x_center = (int)(box.x + box.width / 2);
      int y_center = (int)(box.y + box.height / 2);
      bool is_valid = false;

      // 3d pose
      aero_recognition_msgs::LabeledPose pose;
      geometry_msgs::Pose debug_pose;
      pose.label = box.label;
      pose.pose.orientation.w = 1.0;
      debug_pose.orientation.w = 1.0;

      switch(mode_){
        /** Simple mode **/
      case Object3DProjector::Mode::Simple:
        {
          int valid_cnt = 0;
          double x = 0, y = 0, z = 0;
          for(int v = y_center - wsize_; v <= y_center + wsize_; v += step_){
            for(int u = x_center - wsize_; u <= x_center + wsize_; u += step_){
              if(v < box.y || v >= box.y + box.height || u < box.x || u >= box.x + box.width){
                continue;
              }
              auto x_ = cloud->points[v * cloud->width + u].x;
              auto y_ = cloud->points[v * cloud->width + u].y;
              auto z_ = cloud->points[v * cloud->width + u].z;
              if(!std::isnan(x_) && !std::isnan(y_) && !std::isnan(z_)){
                valid_cnt++;
                x += x_;
                y += y_;
                z += z_;
              }
            }
          }
          if(valid_cnt > 0){
            x /= valid_cnt;
            y /= valid_cnt;
            z /= valid_cnt;
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            pose.pose.position.z = z;
            debug_pose.position.x = x;
            debug_pose.position.y = y;
            debug_pose.position.z = z;
            is_valid = true;

          }
        }
        break;

        /** Simple mode with sandwich parameter**/
      case Object3DProjector::Mode::SimpleForSandwich:
        {
          int valid_cnt = 0;
          double x = 0, y = 0, z = 0;
          y_center += (int)(box.height / 6.0);
          for(int v = y_center - wsize_; v <= y_center + wsize_; v += step_){
            for(int u = x_center - wsize_; u <= x_center + wsize_; u += step_){
              if(v < 0 || v >= box.y + box.height || u < 0 || u >= box.x + box.width){
                continue;
              }
              auto x_ = cloud->points[v * cloud->width + u].x;
              auto y_ = cloud->points[v * cloud->width + u].y;
              auto z_ = cloud->points[v * cloud->width + u].z;
              if(!std::isnan(x_) && !std::isnan(y_) && !std::isnan(z_)){
                valid_cnt++;
                x += x_;
                y += y_;
                z += z_;
              }
            }
          }
          if(valid_cnt > 0){
            x /= valid_cnt;
            y /= valid_cnt;
            z /= valid_cnt;
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            pose.pose.position.z = z;
            debug_pose.position.x = x;
            debug_pose.position.y = y;
            debug_pose.position.z = z;
            is_valid = true;

          }
        }
        break;

        /** Nearest mode **/
      case Object3DProjector::Mode::Nearest:
        {

          // search and sort 3d point by distance from camera
          std::vector<pcl::PointXYZRGBA> valid_points;
          Object3DProjector::getDistanceSortedPoints(cloud, valid_points, box);

          if(valid_points.size() > 0){
            double ave_dist = 0.0;
            double ave_dist_square = 0.0;
            for(auto p : valid_points){
              double d = sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
              ave_dist += d / valid_points.size();
              ave_dist_square += d * d / valid_points.size();
            }
            double sigma = sqrt(ave_dist_square - ave_dist * ave_dist);

            for(auto itr = valid_points.begin(); itr != valid_points.end(); ++itr){

              double dist = sqrt(itr->x * itr->x + itr->y * itr->y + itr->z * itr->z);
              if(fabs(dist - ave_dist) < 2 * sigma){
                pose.pose.position.x = itr->x;
                pose.pose.position.y = itr->y;
                pose.pose.position.z = itr->z;
                debug_pose.position.x = itr->x;
                debug_pose.position.y = itr->y;
                debug_pose.position.z = itr->z;
                is_valid = true;

                break;
              }
            }
          }
        }
        break;

        /** Nearest Center mode **/
      case Object3DProjector::Mode::NearestCenter:
        {

          // search and sort 3d point by distance from camera
          std::vector<pcl::PointXYZRGBA> valid_points;
          Object3DProjector::getDistanceSortedPoints(cloud, valid_points, box);

          if(valid_points.size() > 0){
            double ave_dist = 0.0;
            double ave_dist_square = 0.0;
            for(auto p : valid_points){
              double d = sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
              ave_dist += d / valid_points.size();
              ave_dist_square += d * d / valid_points.size();
            }
            double sigma = sqrt(ave_dist_square - ave_dist * ave_dist);

            for(auto itr = valid_points.begin(); itr != valid_points.end(); ++itr){

              // noize removal (temporary)
              double dist = sqrt(itr->x * itr->x + itr->y * itr->y + itr->z * itr->z);
              if(fabs(dist - ave_dist) < 2 * sigma){
                auto ray = model.projectPixelTo3dRay(cv::Point2d(x_center, y_center));
                Eigen::Vector3f direction(ray.x, ray.y, ray.z);
                Eigen::Vector3f nearest_point(itr->x, itr->y, itr->z);
                auto point_3d = direction * direction.dot(nearest_point) / direction.norm() / direction.norm();

                pose.pose.position.x = point_3d.x();
                pose.pose.position.y = point_3d.y();
                pose.pose.position.z = point_3d.z();
                debug_pose.position.x = point_3d.x();
                debug_pose.position.y = point_3d.y();
                debug_pose.position.z = point_3d.z();

                is_valid = true;

                break;
              }
            }
          }
        }
        break;

      default:
        break;
      }

      if(is_valid){
        // Red point for valid 3d projection
        auto point_2d = model.project3dToPixel(cv::Point3d(pose.pose.position.x,
                                                           pose.pose.position.y,
                                                           pose.pose.position.z));
        cv::circle(debug_img, point_2d, 5, cv::Scalar(0, 0, 255) ,-1);
        ROS_INFO("          pos : %lf %lf %lf",
                 pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);

        poses_out_msg.poses.push_back(pose);
        poses_debug_msg.poses.push_back(debug_pose);
      } else {
          // Blue point for invalid 3d projection
          cv::circle(debug_img, cv::Point(x_center, y_center), 5, cv::Scalar(255, 0, 0) ,-1);
          ROS_INFO("        false : no valid point found.");
      }
    }

    if(poses_out_msg.poses.size() > 0){
      pub_.publish(poses_out_msg);
      debug_pub_.publish(poses_debug_msg);
    }
    image_pub_.publish(cv_bridge::CvImage(
                                          boxes_msg->header,
                                          img_msg->encoding,
                                          debug_img).toImageMsg());
  }

  void Object3DProjector::getDistanceSortedPoints(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,
                                                  std::vector<pcl::PointXYZRGBA>& sorted_points,
                                                  aero_recognition_msgs::Scored2DBox& box){
    int valid_cnt = 0;

    sorted_points.reserve(box.width * box.height / step_ / step_);
    for(int v = box.y; v <= box.y + box.height; v += step_){
      for(int u = box.x; u <= box.x + box.width; u += step_){
        pcl::PointXYZRGBA point;
        point.x = cloud->points[v * cloud->width + u].x;
        point.y = cloud->points[v * cloud->width + u].y;
        point.z = cloud->points[v * cloud->width + u].z;
        if(!std::isnan(point.x) && !std::isnan(point.y) && !std::isnan(point.z) && point.z > 0.1){
          sorted_points.push_back(point);
          valid_cnt++;
        }
      }
    }

    if(valid_cnt == 0){
      return;
    }
    sorted_points.resize(valid_cnt);
    std::sort(sorted_points.begin(),
              sorted_points.end(),
              [](const pcl::PointXYZRGBA& a, const pcl::PointXYZRGBA& b)
              { double a_dist = sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
                double b_dist = sqrt(b.x * b.x + b.y * b.y + b.z * b.z);
                return a_dist < b_dist;});
  }

  double Object3DProjector::distance(const pcl::PointXYZRGBA& a, const pcl::PointXYZRGBA& b){
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));
  }

};

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (aero_ssd_recognition::Object3DProjector, nodelet::Nodelet);
