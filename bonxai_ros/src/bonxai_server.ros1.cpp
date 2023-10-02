#include "bonxai_server.ros1.hpp"

namespace bonxai_server
{
void BonxaiServer::onInit()
{
  // get nodehandles
  node_handle_ = this->getNodeHandle();
  private_node_handle_ = this->getPrivateNodeHandle();

  // Load the parameters.
  loadParameters();

  // initialize bonxai object & params
  NODELET_INFO("Voxel resolution %f", res_);
  bonxai_ = std::make_unique<BonxaiT>(res_);
  BonxaiT::Options options = { bonxai_->logods(sensor_model_miss_),
                              bonxai_->logods(sensor_model_hit_),
                              bonxai_->logods(sensor_model_min_),
                              bonxai_->logods(sensor_model_max_) };
  bonxai_->setOptions(options);

  if (latched_topics_)
  {
    NODELET_INFO("Publishing latched (single publish will take longer, "
                "all topics are prepared)");
  }
  else
  {
    NODELET_INFO("Publishing non-latched (topics are only prepared as needed, "
                "will only be re-published on map change");
  }

  // initialize publishers
  point_cloud_pub_ = private_node_handle_.advertise<PointCloud2>(
      "bonxai_point_cloud_centers", 1, latched_topics_);


  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>();
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

  point_cloud_sub_ = std::make_shared<message_filters::Subscriber<PointCloud2>>(
      private_node_handle_, "cloud_in", 5);
  tf_point_cloud_sub_ = std::make_shared<tf2_ros::MessageFilter<PointCloud2>>(
      *point_cloud_sub_,
      *tf2_buffer_,
      world_frame_id_,
      5,
      private_node_handle_);

  point_cloud_sub_->registerCallback(&BonxaiServer::insertCloudCallback, this);

  reset_srv_ = private_node_handle_.advertiseService(
    "reset", &BonxaiServer::resetSrv, this);

  // TODO: Dynamic reconfigure.
}

void BonxaiServer::loadParameters()
{
  loadParameter("frame_id", world_frame_id_, "map");
  loadParameter("base_frame_id", base_frame_id_, "base_footprint");
  loadParameter("resolution", res_, 0.1);

  loadParameter("occupancy_min_z", occupancy_min_z_, -100.0);
  loadParameter("occupancy_max_z", occupancy_max_z_, 100.0);

  loadParameter("sensor_model/max_range", max_range_, -1.0);
  loadParameter("sensor_model/hit", sensor_model_hit_, 0.7);
  loadParameter("sensor_model/miss", sensor_model_miss_, 0.4);
  loadParameter("sensor_model/min", sensor_model_min_, 0.12);
  loadParameter("sensor_model/max", sensor_model_max_, 0.97);

  loadParameter("latch", latched_topics_, true);
}

bool BonxaiServer::loadParameter(const std::string& key, std::string& value) {
  const bool found = private_node_handle_.getParam(key, value);
  if (found)
    NODELET_DEBUG("Retrieved parameter '%s' = '%s'", key.c_str(),
                  value.c_str());
  return found;
}

bool BonxaiServer::loadParameter(const std::string& key, std::string& value,
                               const std::string& default_value) {
  const bool found =
    private_node_handle_.param<std::string>(key, value, default_value);
  if (!found) {
    if (private_node_handle_.hasParam(key))
      NODELET_ERROR("Parameter '%s' has wrong data type", key.c_str());
    NODELET_WARN("Parameter '%s' not set, defaulting to '%s'", key.c_str(),
                 default_value.c_str());
  }
  if (found)
    NODELET_DEBUG("Retrieved parameter '%s' = '%s'", key.c_str(),
                  value.c_str());
  return found;
}

void BonxaiServer::insertCloudCallback(const PointCloud2& cloud)
{
  const auto start_time = ros::Time::now();

  PointCloudT pc;  // input cloud for filtering and ground-detection
  pcl::fromROSMsg(cloud, pc);

  // Sensor In Global Frames Coordinates
  geometry_msgs::TransformStamped sensor_to_world_transform_stamped;
  try
  {
    sensor_to_world_transform_stamped =
        tf2_buffer_->lookupTransform(world_frame_id_,
                                     cloud.header.frame_id,
                                     cloud.header.stamp,
                                     ros::Duration(1.0));
  }
  catch (const tf2::TransformException& ex)
  {
    NODELET_WARN("%s", ex.what());
    return;
  }

  Eigen::Matrix4f sensor_to_world =
      tf2::transformToEigen(sensor_to_world_transform_stamped.transform)
          .matrix()
          .cast<float>();

  // Transforming Points to Global Reference Frame
  pcl::transformPointCloud(pc, pc, sensor_to_world);

  // Getting the Translation from the sensor to the Global Reference Frame
  const auto& t = sensor_to_world_transform_stamped.transform.translation;

  const PointT sensor_to_world_vec3(t.x, t.y, t.z);

  bonxai_->insertPointCloud(pc.points, sensor_to_world_vec3, max_range_);

  const auto total_elapsed = ros::Time::now() - start_time;
  NODELET_DEBUG("Pointcloud insertion in Bonxai done, %f sec)", total_elapsed.toSec());

  publishAll(cloud.header.stamp);
}

// rcl_interfaces::msg::SetParametersResult
// BonxaiServer::onParameter(const std::vector<rclcpp::Parameter>& parameters)
// {
//   update_param(parameters, "occupancy_min_z", occupancy_min_z_);
//   update_param(parameters, "occupancy_max_z", occupancy_max_z_);

//   double sensor_model_min{ get_parameter("sensor_model.min").as_double() };
//   update_param(parameters, "sensor_model.min", sensor_model_min);
//   double sensor_model_max{ get_parameter("sensor_model.max").as_double() };
//   update_param(parameters, "sensor_model.max", sensor_model_max);
//   double sensor_model_hit{ get_parameter("sensor_model.hit").as_double() };
//   update_param(parameters, "sensor_model.hit", sensor_model_hit);
//   double sensor_model_miss{ get_parameter("sensor_model.miss").as_double() };
//   update_param(parameters, "sensor_model.miss", sensor_model_miss);

//   BonxaiT::Options options = { bonxai_->logods(sensor_model_miss),
//                                bonxai_->logods(sensor_model_hit),
//                                bonxai_->logods(sensor_model_min),
//                                bonxai_->logods(sensor_model_max) };

//   bonxai_->setOptions(options);

//   publishAll(now());

//   rcl_interfaces::msg::SetParametersResult result;
//   result.successful = true;
//   result.reason = "success";
//   return result;
// }

void BonxaiServer::publishAll(const ros::Time& rostime)
{
  thread_local std::vector<Eigen::Vector3d> bonxai_result;
  bonxai_result.clear();
  bonxai_->getOccupiedVoxels(bonxai_result);

  if (bonxai_result.size() <= 1)
  {
    NODELET_WARN("Nothing to publish, bonxai is empty");
    return;
  }

  const bool publish_point_cloud =
      (latched_topics_ ||
       point_cloud_pub_.getNumSubscribers()  > 0);

  // init pointcloud for occupied space:
  if (publish_point_cloud)
  {
    thread_local pcl::PointCloud<PointT> pcl_cloud;
    pcl_cloud.clear();

    for (const auto& voxel : bonxai_result)
    {
      if(voxel.z() >= occupancy_min_z_ && voxel.z() <= occupancy_max_z_)
      {
        pcl_cloud.push_back(PointT(voxel.x(), voxel.y(), voxel.z()));
      }
    }
    PointCloud2 cloud;
    pcl::toROSMsg(pcl_cloud, cloud);

    cloud.header.frame_id = world_frame_id_;
    cloud.header.stamp = rostime;
    point_cloud_pub_.publish(cloud);
    NODELET_DEBUG("Published occupancy grid with %ld voxels", pcl_cloud.points.size());
  }
}

bool BonxaiServer::resetSrv(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  const auto rostime = ros::Time::now();
  bonxai_ = std::make_unique<BonxaiT>(res_);

  NODELET_INFO("Cleared Bonxai");
  publishAll(rostime);

  return true;
}

}  // namespace bonxai_server

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(bonxai_server::BonxaiServer, nodelet::Nodelet)
