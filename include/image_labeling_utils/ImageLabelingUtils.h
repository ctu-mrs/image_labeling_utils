#pragma once
#ifndef IMAGE_LABELING_UTILS_H
#define IMAGE_LABELING_UTILS_H

/* includes //{ */

/* each ROS nodelet must have these */
#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

/* TF2 related ROS includes */
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>

/* camera image messages */
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

/* long unsigned integer message */
#include <std_msgs/UInt64.h>

/* for loading dynamic parameters while the nodelet is running */
#include <dynamic_reconfigure/server.h>

/* this header file is created during compilation from python script dynparam.cfg */
#include <image_labeling_utils/dynparamConfig.h>

/* some STL includes */
#include <stdlib.h>
#include <stdio.h>
#include <mutex>
// Eigen library for vectors
#include <Eigen/Dense>

/* some OpenCV includes */
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

/* ROS includes for working with OpenCV and images */
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>

/* custom helper functions from our library */
#include <mrs_lib/param_loader.h>
#include <mrs_lib/transformer.h>

/* packages for json reading and writing COCO format */
#include "json/json.h"
#include <iostream>
#include <iostream>


//}


namespace image_labeling_utils
{

/* class ImageLabelingUtils //{ */

class ImageLabelingUtils : public nodelet::Nodelet {

public:
  /* onInit() is called when nodelet is launched (similar to main() in regular node) */
  virtual void onInit();

private:
  /* flags */
  bool is_initialized_ = false;

  bool got_image_       = false;
  bool got_artefact_gt_ = false;
  bool got_camera_info_ = false;

  std::string _uav_name_;

  // | --------------------- MRS transformer -------------------- |

  mrs_lib::Transformer transformer_;

  // | ---------------------- msg callbacks --------------------- |

  void                        callbackImage(const sensor_msgs::ImageConstPtr& msg);
  image_transport::Subscriber sub_image_;
  ros::Time                   time_last_image_;

  void                               callbackCameraInfo(const sensor_msgs::CameraInfoConstPtr& msg);
  ros::Subscriber                    sub_camera_info_;
  image_geometry::PinholeCameraModel camera_model_;
  ros::Time                          time_last_camera_info_;
  std::mutex                         mutex_counters_;

  void                       callbackArtefactGt(const geometry_msgs::PoseStampedConstPtr& msg);
  ros::Subscriber            sub_artefact_gt_;
  geometry_msgs::PoseStamped artefact_pose;
  ros::Time                  time_last_artefact_gt_;
  Eigen::Vector3d            artefact_vector_;

  // | --------------------- timer callbacks -------------------- |

  void       callbackTimerCheckSubscribers(const ros::TimerEvent& te);
  ros::Timer timer_check_subscribers_;
  int        _rate_timer_check_subscribers_;

  // | ----------------------- publishers ----------------------- |
  image_transport::Publisher pub_with_boundings;
  int                        _rate_timer_publish_;


  // | ------------- variables for point projection ------------- |
  std::string                                 world_frame_id_;
  std::string                                 camera_frame_id_;
  tf2_ros::Buffer                             tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_ptr_;

  // | --------------- object params for labeling --------------- |
  float           _obj_width_;
  float           _obj_height_;
  float           _obj_offset_x_;
  float           _obj_offset_y_;
  float           _obj_offset_z_;
  Eigen::MatrixXd _objects_;
  int             _object_id_;
  std::string     _object_str_;
  bool            _labeling_on_;

  // | --------------- labeling files destinations -------------- |
  std::string img_saving_path_;
  std::string json_saving_path_;
  std::string dataset_name_;

  // | -------------------- JSON instruments -------------------- |
  Json::Reader              reader;
  Json::Value               root;
  Json::StreamWriterBuilder builder;
  Json::StyledWriter        styledWriter;
  std::ofstream             file_id;

  // | ------------------- dynamic reconfigure ------------------ |

  typedef image_labeling_utils::dynparamConfig                              Config;
  typedef dynamic_reconfigure::Server<image_labeling_utils::dynparamConfig> ReconfigureServer;
  boost::recursive_mutex                                                    mutex_dynamic_reconfigure_;
  boost::shared_ptr<ReconfigureServer>                                      reconfigure_server_;
  void                                                                      callbackDynamicReconfigure(Config& config, uint32_t level);
  image_labeling_utils::dynparamConfig                                      last_drs_config_;


  cv::Mat projectWorldPointToImage(cv::InputArray image, const ros::Time& image_stamp);

  void saveFrame(cv::InputArray image, cv::Point2d left_top, cv::Point2d right_bot);
  long frame_count_;

  Json::Value prepareStructure();

  std::string base64_encode(std::vector <uchar> in);
};

//}

}  // namespace image_labeling_utils
#endif
