#pragma once

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "bonxai_map/pcl_utils.hpp"
#include "bonxai_map/probabilistic_map.hpp"
#include "bonxai/bonxai.hpp"

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>

#include <message_filters/subscriber.h>
#include <nodelet/nodelet.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <std_srvs/Empty.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>

namespace bonxai_server
{

using sensor_msgs::PointCloud2;

class BonxaiServer : public nodelet::Nodelet
{
public:
  using PointT = pcl::PointXYZ;
  using PointCloudT = pcl::PointCloud<PointT>;
  using BonxaiT = Bonxai::ProbabilisticMap;

  /**
   * @brief Initializes nodelet when nodelet is loaded.
   *
   * Overrides nodelet::Nodelet::onInit().
   */
  virtual void onInit() override;

  bool resetSrv(std_srvs::Empty::Request& req,
                std_srvs::Empty::Response& resp);

  virtual void insertCloudCallback(const PointCloud2& cloud);

protected:
  virtual void publishAll(const ros::Time& rostime);


  /**
   * @brief Loads ROS parameters from parameter server.
   */
  void loadParameters();

/**
   * @brief Loads requested ROS parameter from parameter server.
   *
   * @param[in]   key      parameter name
   * @param[out]  value    variable where to store the retrieved parameter
   *
   * @return  true         if parameter was successfully retrieved
   * @return  false        if parameter was not found
   */
  bool loadParameter(const std::string& key, std::string& value);

  /**
   * @brief Loads requested ROS parameter from parameter server, allows default
   * value.
   *
   * @param[in]   key            parameter name
   * @param[out]  value          variable where to store the retrieved parameter
   * @param[in]   default_value  default value
   *
   * @return  true         if parameter was successfully retrieved
   * @return  false        if parameter was not found or default was used
   */
  bool loadParameter(const std::string& key, std::string& value,
                     const std::string& default_value);

  /**
   * @brief Loads requested ROS parameter from parameter server.
   *
   * @tparam  T            type (one of int, double, bool)
   *
   * @param[in]   key      parameter name
   * @param[out]  value    variable where to store the retrieved parameter
   *
   * @return  true         if parameter was successfully retrieved
   * @return  false        if parameter was not found
   */
  template <typename T>
  bool loadParameter(const std::string& key, T& value);

  /**
   * @brief Loads requested ROS parameter from parameter server, allows default
   * value.
   *
   * @tparam  T            type (one of int, double, bool)
   *
   * @param[in]   key            parameter name
   * @param[out]  value          variable where to store the retrieved parameter
   * @param[in]   default_value  default value
   *
   * @return  true         if parameter was successfully retrieved
   * @return  false        if parameter was not found or default was used
   */
  template <typename T>
  bool loadParameter(const std::string& key, T& value, const T& default_value);

  // TODO: Dynamic reconfigure...
  // OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  // rcl_interfaces::msg::SetParametersResult
  // onParameter(const std::vector<rclcpp::Parameter>& parameters);

  ros::Publisher point_cloud_pub_;
  std::shared_ptr<message_filters::Subscriber<PointCloud2>> point_cloud_sub_;
  std::shared_ptr<tf2_ros::MessageFilter<PointCloud2>> tf_point_cloud_sub_;
  // rclcpp::Service<BBoxSrv>::SharedPtr clear_bbox_srv_;
  ros::ServiceServer reset_srv_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

  std::unique_ptr<BonxaiT> bonxai_;
  std::vector<Bonxai::CoordT> key_ray_;

  double max_range_;
  std::string world_frame_id_;  // the map frame
  std::string base_frame_id_;   // base of the robot for ground plane filtering

  bool latched_topics_;

  double res_;

  double occupancy_min_z_;
  double occupancy_max_z_;

  bool publish_2d_map_;
  bool map_origin_changed;
  // octomap::OcTreeKey padded_min_key_;
  unsigned multires_2d_scale_;
  bool project_complete_map_;

  double sensor_model_hit_;
  double sensor_model_miss_;
  double sensor_model_min_;
  double sensor_model_max_;

    /**
   * @brief ROS node handle
   */
  ros::NodeHandle node_handle_;

  /**
   * @brief Private ROS node handle
   */
  ros::NodeHandle private_node_handle_;
};

template <typename T>
bool BonxaiServer::loadParameter(const std::string& key, T& value) {
  const bool found = private_node_handle_.getParam(key, value);
  if (found)
    NODELET_DEBUG("Retrieved parameter '%s' = '%s'", key.c_str(),
                  std::to_string(value).c_str());
  return found;
}


template <typename T>
bool BonxaiServer::loadParameter(const std::string& key, T& value,
                               const T& default_value) {
  const bool found = private_node_handle_.param<T>(key, value, default_value);
  if (!found) {
    if (private_node_handle_.hasParam(key))
      NODELET_ERROR("Parameter '%s' has wrong data type", key.c_str());
    NODELET_WARN("Parameter '%s' not set, defaulting to '%s'", key.c_str(),
                 std::to_string(default_value).c_str());
  }
  if (found)
    NODELET_DEBUG("Retrieved parameter '%s' = '%s'", key.c_str(),
                  std::to_string(value).c_str());
  return found;
}

}  // namespace bonxai_server
