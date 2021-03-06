#include <image_labeling_utils/ImageLabelingUtils.h>
#include <pluginlib/class_list_macros.h>


namespace image_labeling_utils
{

/* onInit() //{ */

void ImageLabelingUtils::onInit() {
  got_image_       = false;
  got_artefact_gt_ = false;
  _labeling_on_    = false;
  frame_count_     = 0;

  ros::NodeHandle nh("~");
  ros::Time::waitForValid();

  mrs_lib::ParamLoader param_loader(nh, "ImageLabelingUtils");

  param_loader.loadParam("uav_name", _uav_name_);
  param_loader.loadParam("camera_frame_id", camera_frame_id_);
  param_loader.loadParam("rate/publish", _rate_timer_publish_);
  param_loader.loadParam("rate/check_subscribers", _rate_timer_check_subscribers_);
  param_loader.loadParam("object_id", _object_id_);
  param_loader.loadParam("object_str", _object_str_);
  param_loader.loadParam("dataset_name", dataset_name_);
  param_loader.loadParam("json_dir", json_saving_path_);
  param_loader.loadParam("img_dir", img_saving_path_);
  param_loader.loadMatrixDynamic("objects", _objects_, -1, 6);
  dataset_name_ += "_" + std::to_string(ros::Time::now().toSec());

  if (_object_id_ > _objects_.rows() || _object_id_ < 0) {
    ROS_ERROR("[ImageLabelingUtils]: The object id is below zero or out of the size of the objects matrix!");
    ros::shutdown();
  } else {
    _obj_height_   = _objects_(_object_id_, 0);
    _obj_width_    = _objects_(_object_id_, 1);
    _obj_width_y_  = _objects_(_object_id_, 2);
    _obj_offset_x_ = _objects_(_object_id_, 3);
    _obj_offset_y_ = _objects_(_object_id_, 4);
    _obj_offset_z_ = _objects_(_object_id_, 5);
  }

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[ImageLabelingUtils]: failed to load non-optional parameters!");
    ros::shutdown();
  }

  // | --------------------- tf transformer --------------------- |

  transformer_ = mrs_lib::Transformer("ImageLabelingUtils", _uav_name_);


  /* initialize the image transport, needs node handle */
  image_transport::ImageTransport it(nh);


  tf_listener_ptr_ = std::make_unique<tf2_ros::TransformListener>(tf_buffer_);

  srv_save_csv_ = nh.advertiseService("save_csv", &ImageLabelingUtils::serviceSaveCsv, this);


  // | ----------------- initialize subscribers ----------------- |
  sub_image_       = it.subscribe("image_in", 1, &ImageLabelingUtils::callbackImage, this);
  sub_camera_info_ = nh.subscribe("camera_info_in", 1, &ImageLabelingUtils::callbackCameraInfo, this, ros::TransportHints().tcpNoDelay());
  sub_artefact_gt_ = nh.subscribe("artefact_pose", 1, &ImageLabelingUtils::callbackArtefactGt, this, ros::TransportHints().tcpNoDelay());

  // | ------------------ initialize publishers ----------------- |
  pub_with_boundings = it.advertise("artefact_boundings", 1);

  timer_check_subscribers_ = nh.createTimer(ros::Rate(_rate_timer_check_subscribers_), &ImageLabelingUtils::callbackTimerCheckSubscribers, this);

  // | ---------- initialize dynamic reconfigure server --------- |

  reconfigure_server_.reset(new ReconfigureServer(mutex_dynamic_reconfigure_, nh));
  ReconfigureServer::CallbackType f = boost::bind(&ImageLabelingUtils::callbackDynamicReconfigure, this, _1, _2);
  reconfigure_server_->setCallback(f);

  {
    std::scoped_lock loc(mutex_dynamic_reconfigure_);
    last_drs_config_.obj_height   = _obj_height_;
    last_drs_config_.obj_width    = _obj_width_;
    last_drs_config_.obj_width_y  = _obj_width_y_;
    last_drs_config_.obj_offset_x = _obj_offset_x_;
    last_drs_config_.obj_offset_y = _obj_offset_y_;
    last_drs_config_.obj_offset_z = _obj_offset_z_;
    last_drs_config_.labeling_on  = _labeling_on_;
  }
  reconfigure_server_->updateConfig(last_drs_config_);


  ROS_INFO("[ImageLabelingUtils]: Initialization was successfull");

  boost::filesystem::create_directories(img_saving_path_ + "/" + dataset_name_ + "/");
  boost::filesystem::create_directories(json_saving_path_ + "/" + dataset_name_ + "/");
  img_saving_path_ = img_saving_path_ + "/" + dataset_name_ + "/";
  json_saving_path_ += "/" + dataset_name_ + "/";
  path_name_      = "../../images/" + dataset_name_ + "/";
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
    got_image_       = true;
    time_last_image_ = ros::Time::now();
  }

  const cv_bridge::CvImageConstPtr bridge_image_ptr = cv_bridge::toCvShare(msg, color_encoding);
  const std_msgs::Header           msg_header       = msg->header;


  // | ----------- Project a world point to the image ----------- |

  const auto projection_image = ImageLabelingUtils::projectWorldPointToImage(bridge_image_ptr->image, msg_header.stamp);


  cv_bridge::CvImage bridge_image_out;
  bridge_image_out.header   = msg_header;
  bridge_image_out.image    = projection_image;
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
    got_artefact_gt_       = true;
    time_last_artefact_gt_ = ros::Time::now();
    artefact_pose          = *msg;
    artefact_vector_       = Eigen::Vector3d(artefact_pose.pose.position.x, artefact_pose.pose.position.y, artefact_pose.pose.position.z);
  }
}

//}

/* projectWorldPointToImage() method //{ */
cv::Mat ImageLabelingUtils::projectWorldPointToImage(cv::InputArray image, const ros::Time& image_stamp) {
  cv::Mat projected_point;
  image.copyTo(projected_point);

  if (!got_camera_info_) {
    ROS_WARN_THROTTLE(1.0, "[ImageLabelingUtils]: No camera info received yet, cannot backproject point to image");
    return projected_point;
  }
  geometry_msgs::PoseStamped pt3d_world;
  {
    std::scoped_lock lock(mutex_counters_);
    pt3d_world = artefact_pose;
  }
  pt3d_world.pose.position.x -= _obj_offset_x_;
  pt3d_world.pose.position.y -= _obj_offset_y_;
  pt3d_world.pose.position.z -= _obj_offset_z_;

  geometry_msgs::PoseStamped pt3d_world_left_top;
  geometry_msgs::PoseStamped pt3d_world_right_bot;

  pt3d_world_left_top              = pt3d_world;
  pt3d_world_left_top.header.stamp = ros::Time();
  pt3d_world_left_top.pose.position.x -= _obj_width_;
  pt3d_world_left_top.pose.position.y -= _obj_width_y_;
  pt3d_world_left_top.pose.position.z -= _obj_height_;

  pt3d_world_right_bot              = pt3d_world;
  pt3d_world_right_bot.header.stamp = ros::Time();
  pt3d_world_right_bot.pose.position.x += _obj_width_;
  pt3d_world_right_bot.pose.position.y += _obj_width_y_;
  pt3d_world_right_bot.pose.position.z += _obj_height_;

  // | --------- transform the point to the camera frame -------- |

  std::string camera_frame = camera_model_.tfFrame();

  auto ret   = transformer_.transformSingle(camera_frame_id_, pt3d_world);
  auto ret_l = transformer_.transformSingle(camera_frame_id_, pt3d_world_left_top);
  auto ret_r = transformer_.transformSingle(camera_frame_id_, pt3d_world_right_bot);

  geometry_msgs::PoseStamped pt3d_cam;
  geometry_msgs::PoseStamped pt3d_cam_l;
  geometry_msgs::PoseStamped pt3d_cam_r;

  if (ret && ret_l && ret_r) {
    pt3d_cam   = ret.value();
    pt3d_cam_l = ret_l.value();
    pt3d_cam_r = ret_r.value();
  } else {
    ROS_WARN_THROTTLE(1.0, "[ImageLabelingUtils]: Failed to tranform point from world to camera frame, cannot backproject point to image");
    return projected_point;
  }

  // | ----------- backproject the point from 3D to 2D ---------- |
  /* ROS_INFO("[]: pos %f %f %f", pt3d_cam.pose.position.x, pt3d_cam.pose.position.y, pt3d_cam.pose.position.z); */
  /* ROS_INFO("[]: width %f height %f offset %f", _obj_width_, _obj_height_, _obj_offset_); */

  const cv::Point3d pt3d(pt3d_cam.pose.position.x, pt3d_cam.pose.position.y, pt3d_cam.pose.position.z);

  const cv::Point3d pt3d_l(pt3d_cam_l.pose.position.x, pt3d_cam_l.pose.position.y, pt3d_cam_l.pose.position.z);

  const cv::Point3d pt3d_r(pt3d_cam_r.pose.position.x, pt3d_cam_r.pose.position.y, pt3d_cam_r.pose.position.z);

  const cv::Point2d pt2d = camera_model_.project3dToPixel(pt3d);

  const cv::Point2d pt2d_l = camera_model_.project3dToPixel(pt3d_l);
  const cv::Point2d pt2d_r = camera_model_.project3dToPixel(pt3d_r);
  // | ----------- unrectify the 2D point coordinates ----------- |

  const cv::Point2d pt2d_unrec   = camera_model_.unrectifyPoint(pt2d);
  const cv::Point2d pt2d_unrec_l = camera_model_.unrectifyPoint(pt2d_l);
  const cv::Point2d pt2d_unrec_r = camera_model_.unrectifyPoint(pt2d_r);

  // | --------------- draw the point to the image -------------- |

  const cv::Scalar color(255, 0, 0);
  cv::Rect         bounding(pt2d.x, pt2d.y, 10, 10);
  ROS_INFO_THROTTLE(1.0, "[ImageLabelingUtils]: Received image, label is set ");

  cv::rectangle(projected_point, pt2d_l, pt2d_r, color);

  saveFrame(image, pt2d_l, pt2d_r);

  // Draw the text with the coordinates to the image
  const std::string coord_txt = "[" + std::to_string(artefact_pose.pose.position.x) + "," + std::to_string(artefact_pose.pose.position.y) + "," +
                                std::to_string(artefact_pose.pose.position.z) + "]";

  const cv::Point2d txt_pos(pt2d_unrec.x + 20, pt2d_unrec.y + 20);
  const int         txt_font       = cv::FONT_HERSHEY_PLAIN;
  const double      txt_font_scale = 1.0;
  cv::putText(projected_point, coord_txt, txt_pos, txt_font, txt_font_scale, color);

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

/* //{ callbackDynamicReconfigure() */
void ImageLabelingUtils::callbackDynamicReconfigure([[maybe_unused]] Config& config, [[maybe_unused]] uint32_t level) {

  if (!is_initialized_)
    return;
  {
    std::scoped_lock loc(mutex_dynamic_reconfigure_);

    if (!config.labeling_on && _labeling_on_) {
      ROS_INFO_THROTTLE(1.0, "[ImageLabelingUtils]: Saving csv due to labeling has been stopped");
      saveCsv();
    }
    ROS_INFO("[ImageLabelingUtils]: triggered dynamic reconfigure ");
    _obj_height_   = config.obj_height;
    _obj_width_    = config.obj_width;
    _obj_width_y_  = config.obj_width_y;
    _obj_offset_x_ = config.obj_offset_x;
    _obj_offset_y_ = config.obj_offset_y;
    _obj_offset_z_ = config.obj_offset_z;
    _labeling_on_  = config.labeling_on;
  }
}
//}

/* saveFrame() //{ */

void ImageLabelingUtils::saveFrame(cv::InputArray image, cv::Point2d left_top, cv::Point2d right_bot) {

  {
    std::scoped_lock loc(mutex_dynamic_reconfigure_);

    if (!_labeling_on_) {
      return;
    }

    std::string   name_  = std::to_string(ros::Time::now().toSec()) + "_" + std::to_string(frame_count_);
    Json::Value   frame_ = prepareStructure();
    std::ofstream outfile(json_saving_path_ + name_ + ".json");


    // checking if the label fits into the image, so we won't label it as as correct one
    if ( (left_top.x < image.cols() && right_bot.x < image.cols()) &&  (left_top.y < image.rows() && right_bot.y < image.rows())) {
      Json::Value shape_;
      shape_["label"]  = _object_str_;
      shape_["points"] = Json::arrayValue;
      if (_object_str_ != "none") {
        ROS_INFO("[ImageLabelingUtils]: The object is labeled as %s",_object_str_.c_str());
        Json::Value l;
        l[0] = left_top.x;
        l[1] = left_top.y;
        shape_["points"].append(l);

        Json::Value r;
        r[0] = right_bot.x;
        r[1] = right_bot.y;
        shape_["points"].append(r);
      }
      shape_["shape_type"] = "rectangle";
      shape_["flags"]      = Json::objectValue;
      frame_["shapes"].append(shape_);
    }

    frame_["imagePath"]   = path_name_ + name_ + ".png";
    frame_["imageHeight"] = image.rows();
    frame_["imageWidth"]  = image.cols();
    bool cv_res_          = cv::imwrite(img_saving_path_ + name_ + ".png", image);

    _csv_ += path_name_ + name_ + ".png," + std::to_string((int)right_bot.x) + "," + std::to_string((int)right_bot.y) + "," + std::to_string((int)left_top.x) +
             "," + std::to_string((int)left_top.y) + "," + _object_str_ + "\n";

    if (!cv_res_ && outfile.fail()) {
      ROS_ERROR_THROTTLE(1.0, "[ImageLabelingUtils]: Image or label file couldn't be saved, error, check the paths");
    } else {
      ROS_INFO_THROTTLE(1.0, "[ImageLabelingUtils]: Image and label have been saved");
    }

    outfile << styledWriter.write(frame_);
    outfile.flush();
    outfile.close();
    frame_count_++;
  }
}

//}

/* base64_encode() //{ */

std::string ImageLabelingUtils::base64_encode(std::vector<uchar> in) {
  std::string out;

  int val = 0, valb = -6;
  for (uchar c : in) {
    val = (val << 8) + c;
    valb += 8;
    while (valb >= 0) {
      out.push_back(in[(val >> valb) & 0x3F]);
      valb -= 6;
    }
  }
  if (valb > -6)
    out.push_back(in[((val << 8) >> (valb + 8)) & 0x3F]);
  while (out.size() % 4)
    out.push_back('=');
  return out;
}

//}

/* prepareStructure() //{ */

Json::Value ImageLabelingUtils::prepareStructure() {
  Json::Value structure_;
  structure_["version"]   = "0.0.1";
  structure_["flags"]     = Json::objectValue;
  structure_["shapes"]    = Json::arrayValue;
  structure_["imageData"] = Json::nullValue;

  return structure_;
}

//}

/* serviceSaveCsv() //{ */

bool ImageLabelingUtils::serviceSaveCsv([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp) {

  if (saveCsv()) {
    ROS_ERROR("[ERROR]: Service saving csv has failed, check path");
    resp.success = false;
    return true;
  }
  resp.success = true;
  ROS_INFO_THROTTLE(1.0, "[ImageLabelingUtils]: CSV have been saved");
  return true;
}

//}

/* saveCsv() //{ */

bool ImageLabelingUtils::saveCsv() {
  std::ofstream outfile_csv(json_saving_path_ + dataset_name_ + ".csv");
  if (outfile_csv.fail()) {
    ROS_ERROR("[ERROR]: Saving has failed, check path");
    return false;
  }
  outfile_csv << _csv_;

  outfile_csv.flush();
  outfile_csv.close();
  return true;
}

//}


}  // namespace image_labeling_utils

PLUGINLIB_EXPORT_CLASS(image_labeling_utils::ImageLabelingUtils, nodelet::Nodelet);
