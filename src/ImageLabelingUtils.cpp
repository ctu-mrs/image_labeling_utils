#include <image_labeling_utils/ImageLabelingUtils.h>

/* every nodelet must include macros which export the class as a nodelet plugin */
#include <pluginlib/class_list_macros.h>


namespace image_labeling_utils
{

  /* onInit() //{ */
  
  void ImageLabelingUtils::onInit() {
    got_image_       = false;
    got_artefact_gt_ = false;
  
    ros::NodeHandle nh("~");
    ros::Time::waitForValid();
  
    mrs_lib::ParamLoader param_loader(nh, "ImageLabelingUtils");
  
    param_loader.loadParam("uav_name", _uav_name_);
    param_loader.loadParam("rate/publish", _rate_timer_publish_);
    param_loader.loadParam("rate/check_subscribers", _rate_timer_check_subscribers_);
  
  
    if (!param_loader.loadedSuccessfully()) {
      ROS_ERROR("[ImageLabelingUtils]: failed to load non-optional parameters!");
      ros::shutdown();
    }
  
    // | --------------------- tf transformer --------------------- |
  
    transformer_ = mrs_lib::Transformer("ImageLabelingUtils",_uav_name_);
  
  
    /* initialize the image transport, needs node handle */
    image_transport::ImageTransport it(nh);
  
  
    tf_listener_ptr_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_);
  
  
    // | ----------------- initialize subscribers ----------------- |
    sub_image_       = it.subscribe("image_in", 1, &ImageLabelingUtils::callbackImage, this);
    sub_camera_info_ = nh.subscribe("camera_info_in", 1, &ImageLabelingUtils::callbackCameraInfo, this, ros::TransportHints().tcpNoDelay());
    sub_artefact_gt_ = nh.subscribe("artefact_pose", 1, &ImageLabelingUtils::callbackArtefactGt, this, ros::TransportHints().tcpNoDelay());
  
    // | ------------------ initialize publishers ----------------- |
    pub_with_boundings = it.advertise("artefact_boundings", 1);
  
    timer_check_subscribers_ = nh.createTimer(ros::Rate(_rate_timer_check_subscribers_), &ImageLabelingUtils::callbackTimerCheckSubscribers, this);
  
    is_initialized_ = true;
  
  }
  
  //}

/* callbackCameraInfo() method //{ */
void ImageLabelingUtils::callbackCameraInfo(const sensor_msgs::CameraInfoConstPtr& msg) {
  if (!is_initialized_)
    return;

  got_camera_info_       = true;
  time_last_camera_info_ = ros::Time::now();

  // update the camera model using the latest camera info message
  camera_model_.fromCameraInfo(*msg);
}
//}

/* callbackImage() method //{ */
void ImageLabelingUtils::callbackImage(const sensor_msgs::ImageConstPtr& msg) {
  const std::string color_encoding     = "bgr8";
  const std::string grayscale_encoding = "mono8";

  if (!is_initialized_)
    return;

  /* update the checks-related variables (in a thread-safe manner) */
  {
    std::scoped_lock lock(mutex_counters_);
    got_image_ = true;
    time_last_image_ = ros::Time::now();
  }

  const cv_bridge::CvImageConstPtr bridge_image_ptr = cv_bridge::toCvShare(msg, color_encoding);
  const std_msgs::Header           msg_header       = msg->header;


  // | ----------- Project a world point to the image ----------- |

  const auto projection_image = ImageLabelingUtils::projectWorldPointToImage(bridge_image_ptr->image, msg_header.stamp);


  cv_bridge::CvImage bridge_image_out;
  bridge_image_out.header = msg_header;
  bridge_image_out.image = projection_image;
  bridge_image_out.encoding = color_encoding;

  sensor_msgs::ImageConstPtr out_msg = bridge_image_out.toImageMsg();
  pub_with_boundings.publish(out_msg);



}
//}

/* callbackArtefactGt() //{ */


void ImageLabelingUtils::callbackArtefactGt(const geometry_msgs::PoseStampedConstPtr& msg) {

  if (!is_initialized_)
    return;

  /* update the checks-related variables (in a thread-safe manner) */
  {
    std::scoped_lock lock(mutex_counters_);
    got_artefact_gt_ = true;
    time_last_artefact_gt_ = ros::Time::now();
    artefact_pose = *msg;
    artefact_vector_ = Eigen::Vector3d(artefact_pose.pose.position.x,artefact_pose.pose.position.y,artefact_pose.pose.position.z);
  }

}

//}

/* projectWorldPointToImage() method //{ */
cv::Mat ImageLabelingUtils::projectWorldPointToImage(cv::InputArray image, const ros::Time& image_stamp) {
  // cv::InputArray indicates that the variable should not be modified, but we want
  // to draw into the image. Therefore we need to copy it.
  cv::Mat projected_point;
  image.copyTo(projected_point);

  // If no camera info was received yet, we cannot do the backprojection, alert the user and return.
  if (!got_camera_info_) {
    ROS_WARN_THROTTLE(1.0, "[ImageLabelingUtils]: No camera info received yet, cannot backproject point to image");
    return projected_point;
  }
  geometry_msgs::PoseStamped pt3d_world;
  {
    std::scoped_lock lock(mutex_counters_);
    pt3d_world = artefact_pose;
  }

  geometry_msgs::PoseStamped pt3d_world_left_bot;
  geometry_msgs::PoseStamped pt3d_world_right_top;

  pt3d_world_left_bot = pt3d_world;
  pt3d_world_left_bot.header.stamp = ros::Time();
  pt3d_world_left_bot.pose.position.x -= 0.35;
  pt3d_world_left_bot.pose.position.z -= 0.35;




  pt3d_world_right_top = pt3d_world;
  pt3d_world_right_top.header.stamp = ros::Time();
  pt3d_world_right_top.pose.position.x += 0.35;
  pt3d_world_right_top.pose.position.z += 0.35;

  // | --------- transform the point to the camera frame -------- |

  std::string camera_frame = camera_model_.tfFrame();

  auto ret = transformer_.transformSingle("MARBLE_QAV500/base_link/camera_front_optical", pt3d_world);
  auto ret_l = transformer_.transformSingle("MARBLE_QAV500/base_link/camera_front_optical", pt3d_world_left_bot);
  auto ret_r = transformer_.transformSingle("MARBLE_QAV500/base_link/camera_front_optical", pt3d_world_right_top);

  geometry_msgs::PoseStamped pt3d_cam;
  geometry_msgs::PoseStamped pt3d_cam_l;
  geometry_msgs::PoseStamped pt3d_cam_r;

  if (ret && ret_l && ret_r) {
    pt3d_cam = ret.value();
    pt3d_cam_l = ret_l.value();
    pt3d_cam_r = ret_r.value();
  } else {
    ROS_WARN_THROTTLE(1.0, "[ImageLabelingUtils]: Failed to tranform point from world to camera frame, cannot backproject point to image");
    return projected_point;
  }

  // | ----------- backproject the point from 3D to 2D ---------- |
  ROS_INFO("[]: pos %f %f %f",pt3d_cam.pose.position.x, pt3d_cam.pose.position.y, pt3d_cam.pose.position.z );

  const cv::Point3d pt3d(pt3d_cam.pose.position.x, pt3d_cam.pose.position.y, pt3d_cam.pose.position.z);

  const cv::Point3d pt3d_l(pt3d_cam_l.pose.position.x, pt3d_cam_l.pose.position.y, pt3d_cam_l.pose.position.z);

  const cv::Point3d pt3d_r(pt3d_cam_r.pose.position.x, pt3d_cam_r.pose.position.y, pt3d_cam_r.pose.position.z);
  
  const cv::Point2d pt2d = camera_model_.project3dToPixel(pt3d);  // this is now in rectified image coordinates

  const cv::Point2d pt2d_l = camera_model_.project3dToPixel(pt3d_l);  // this is now in rectified image coordinates
  const cv::Point2d pt2d_r = camera_model_.project3dToPixel(pt3d_r);  // this is now in rectified image coordinates
  // | ----------- unrectify the 2D point coordinates ----------- |

  // This is done to correct for camera distortion. IT has no effect in simulation, where the camera model
  // is an ideal pinhole camera, but usually has a BIG effect when using real cameras, so don't forget this part!
  const cv::Point2d pt2d_unrec = camera_model_.unrectifyPoint(pt2d);  // this is now in unrectified image coordinates
  const cv::Point2d pt2d_unrec_l = camera_model_.unrectifyPoint(pt2d_l);  // this is now in unrectified image coordinates
  const cv::Point2d pt2d_unrec_r = camera_model_.unrectifyPoint(pt2d_r);  // this is now in unrectified image coordinates

  // | --------------- draw the point to the image -------------- |

  // The point will be drawn as a filled circle with the coordinates as text in the image
  const int        pt_radius = 5;      // pixels
  const cv::Scalar color(255, 0, 0);   // red or blue color, depending on the pixel ordering (BGR or RGB)
  const int        pt_thickness = -1;  // pixels, -1 means filled
  cv::Rect bounding(pt2d.x, pt2d.y, 10, 10);
  /* cv::circle(projected_point, pt2d, pt_radius, color, pt_thickness); */
  cv::rectangle(projected_point, pt2d_l,pt2d_r, color);
  ROS_INFO("[]: unref %f %f ", pt2d_l.x,pt2d_r.y);

  // Draw the text with the coordinates to the image
  const std::string coord_txt = "[" + std::to_string(artefact_pose.pose.position.x) + "," + std::to_string(artefact_pose.pose.position.y) + "," + std::to_string(artefact_pose.pose.position.z) + "]";
  const cv::Point2d txt_pos(pt2d_unrec.x, pt2d_unrec.y);  // offset the text a bit to avoid overlap with the circle
  const int         txt_font       = cv::FONT_HERSHEY_PLAIN;      // some default OpenCV font
  const double      txt_font_scale = 1.0;
  cv::putText(projected_point, coord_txt, txt_pos, txt_font, txt_font_scale, color);
  ROS_INFO("[]: shit");

  return projected_point;
}
//}

/* callbackTimerCheckSubscribers() method //{ */
void ImageLabelingUtils::callbackTimerCheckSubscribers([[maybe_unused]] const ros::TimerEvent& te) {

  if (!is_initialized_)
    return;

  if (!got_image_) {
    ROS_WARN_THROTTLE(1.0, "Not received camera image since node launch.");
  }

  if (!got_camera_info_) {
    ROS_WARN_THROTTLE(1.0, "Not received camera info msg since node launch.");
  }
  if (!got_artefact_gt_) {
    ROS_WARN_THROTTLE(1.0, "Not received artefact  msg since node launch.");
  }
}
//}
//

Eigen::Vector3f project(float px_x, float px_y, const image_geometry::PinholeCameraModel& camera_model)
{
  cv::Point2f det_pt(px_x, px_y);
  if (!camera_model.distortionCoeffs().empty())
    det_pt = camera_model.rectifyPoint(det_pt);  // do not forget to rectify the points! (not necessary for Realsense)
  cv::Point3f cv_vec = camera_model.projectPixelTo3dRay(det_pt);
  return Eigen::Vector3f(cv_vec.x, cv_vec.y, cv_vec.z).normalized();
}

}  // namespace image_labeling_utils

PLUGINLIB_EXPORT_CLASS(image_labeling_utils::ImageLabelingUtils, nodelet::Nodelet);
