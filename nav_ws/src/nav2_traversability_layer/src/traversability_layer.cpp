#include "nav2_traversability_layer/traversability_layer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"

namespace nav2_traversability_layer
{

namespace
{
// Convert 2D cell coordinates to a flattened array index.
int index2d(const int x, const int y, const int width)
{
  return y * width + x;
}

// Clamp a floating-point value into the [0, 1] range.
double clamp01(const double value)
{
  return std::max(0.0, std::min(1.0, value));
}
}  // namespace

// Initialize members that are not handled by the base class constructor.
TraversabilityLayer::TraversabilityLayer()
{
  costmap_ = nullptr;
}

// Register all configurable parameters for the traversability layer.
void TraversabilityLayer::declareParameters()
{
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("pointcloud_topic", rclcpp::ParameterValue(std::string("/cloud_registered_body_1")));
  declareParameter("target_frame", rclcpp::ParameterValue(std::string("")));
  declareParameter("transform_tolerance", rclcpp::ParameterValue(0.05));
  declareParameter("max_cloud_age", rclcpp::ParameterValue(0.5));
  declareParameter("analysis_z_min", rclcpp::ParameterValue(-1.0));
  declareParameter("analysis_z_max", rclcpp::ParameterValue(2.0));
  declareParameter("min_points_per_cell", rclcpp::ParameterValue(4));
  declareParameter("ground_percentile", rclcpp::ParameterValue(0.2));
  declareParameter("ground_estimation_method", rclcpp::ParameterValue(std::string("upper_densest")));
  declareParameter("ground_cluster_tolerance", rclcpp::ParameterValue(0.05));
  declareParameter("ground_cluster_percentile", rclcpp::ParameterValue(0.50));
  declareParameter("ground_layer_max_gap", rclcpp::ParameterValue(0.30));
  declareParameter("ground_layer_count_ratio", rclcpp::ParameterValue(0.55));
  declareParameter("max_step_height", rclcpp::ParameterValue(0.24));
  declareParameter("robot_body_height", rclcpp::ParameterValue(1.2));
  declareParameter("obstacle_min_height", rclcpp::ParameterValue(0.06));
  declareParameter("obstacle_ratio_threshold", rclcpp::ParameterValue(0.18));
  declareParameter("max_slope_traversable", rclcpp::ParameterValue(30.0));
  declareParameter("slope_cost_start", rclcpp::ParameterValue(20.0));
  declareParameter("height_cost_start", rclcpp::ParameterValue(0.05));
  declareParameter("slope_compensation_enabled", rclcpp::ParameterValue(true));
  declareParameter("slope_fit_radius", rclcpp::ParameterValue(2));
  declareParameter("min_slope_fit_neighbors", rclcpp::ParameterValue(5));
  declareParameter("allow_low_step_slope_bypass", rclcpp::ParameterValue(true));
  declareParameter("low_step_slope_bypass_height", rclcpp::ParameterValue(0.0));
  declareParameter("flat_step_threshold", rclcpp::ParameterValue(0.02));
  declareParameter("hard_obstacle_height", rclcpp::ParameterValue(0.35));
  declareParameter("hard_obstacle_ratio_threshold", rclcpp::ParameterValue(0.30));
  declareParameter("hard_slope_limit", rclcpp::ParameterValue(50.0));
  declareParameter("hard_step_height", rclcpp::ParameterValue(0.35));
  declareParameter("soft_cost_max", rclcpp::ParameterValue(200));
  declareParameter("unknown_is_lethal", rclcpp::ParameterValue(false));
  declareParameter("reset_each_update", rclcpp::ParameterValue(true));
  declareParameter("transform_to_costmap_frame", rclcpp::ParameterValue(true));
  declareParameter("decay_enabled", rclcpp::ParameterValue(false));
  declareParameter("decay_rate_per_second", rclcpp::ParameterValue(0.0));
  declareParameter("decay_minimum_cost", rclcpp::ParameterValue(1));
}

// Read parameters from the ROS node and normalize the cached values.
void TraversabilityLayer::readParameters()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("TraversabilityLayer node expired");
  }

  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".pointcloud_topic", pointcloud_topic_);
  node->get_parameter(name_ + ".target_frame", target_frame_);
  node->get_parameter(name_ + ".transform_tolerance", transform_tolerance_);
  node->get_parameter(name_ + ".max_cloud_age", max_cloud_age_);
  node->get_parameter(name_ + ".analysis_z_min", analysis_z_min_);
  node->get_parameter(name_ + ".analysis_z_max", analysis_z_max_);
  node->get_parameter(name_ + ".min_points_per_cell", min_points_per_cell_);
  node->get_parameter(name_ + ".ground_percentile", ground_percentile_);
  node->get_parameter(name_ + ".ground_estimation_method", ground_estimation_method_);
  node->get_parameter(name_ + ".ground_cluster_tolerance", ground_cluster_tolerance_);
  node->get_parameter(name_ + ".ground_cluster_percentile", ground_cluster_percentile_);
  node->get_parameter(name_ + ".ground_layer_max_gap", ground_layer_max_gap_);
  node->get_parameter(name_ + ".ground_layer_count_ratio", ground_layer_count_ratio_);
  node->get_parameter(name_ + ".max_step_height", max_step_height_);
  node->get_parameter(name_ + ".robot_body_height", robot_body_height_);
  node->get_parameter(name_ + ".obstacle_min_height", obstacle_min_height_);
  node->get_parameter(name_ + ".obstacle_ratio_threshold", obstacle_ratio_threshold_);
  node->get_parameter(name_ + ".max_slope_traversable", max_slope_traversable_deg_);
  node->get_parameter(name_ + ".slope_cost_start", slope_cost_start_deg_);
  node->get_parameter(name_ + ".height_cost_start", height_cost_start_);
  node->get_parameter(name_ + ".slope_compensation_enabled", slope_compensation_enabled_);
  node->get_parameter(name_ + ".slope_fit_radius", slope_fit_radius_);
  node->get_parameter(name_ + ".min_slope_fit_neighbors", min_slope_fit_neighbors_);
  node->get_parameter(name_ + ".allow_low_step_slope_bypass", allow_low_step_slope_bypass_);
  node->get_parameter(name_ + ".low_step_slope_bypass_height", low_step_slope_bypass_height_);
  node->get_parameter(name_ + ".flat_step_threshold", flat_step_threshold_);
  node->get_parameter(name_ + ".hard_obstacle_height", hard_obstacle_height_);
  node->get_parameter(name_ + ".hard_obstacle_ratio_threshold", hard_obstacle_ratio_threshold_);
  node->get_parameter(name_ + ".hard_slope_limit", hard_slope_limit_deg_);
  node->get_parameter(name_ + ".hard_step_height", hard_step_height_);
  node->get_parameter(name_ + ".soft_cost_max", soft_cost_max_);
  node->get_parameter(name_ + ".unknown_is_lethal", unknown_is_lethal_);
  node->get_parameter(name_ + ".reset_each_update", reset_each_update_);
  node->get_parameter(name_ + ".transform_to_costmap_frame", transform_to_costmap_frame_);
  node->get_parameter(name_ + ".decay_enabled", decay_enabled_);
  node->get_parameter(name_ + ".decay_rate_per_second", decay_rate_per_second_);
  node->get_parameter(name_ + ".decay_minimum_cost", decay_minimum_cost_);

  if (target_frame_.empty()) {
    target_frame_ = layered_costmap_->getGlobalFrameID();
  }
  ground_percentile_ = clamp01(ground_percentile_);
  ground_cluster_percentile_ = clamp01(ground_cluster_percentile_);
  ground_cluster_tolerance_ = std::max(0.01, ground_cluster_tolerance_);
  ground_layer_max_gap_ = std::max(0.0, ground_layer_max_gap_);
  ground_layer_count_ratio_ = clamp01(ground_layer_count_ratio_);
  if (low_step_slope_bypass_height_ <= 0.0) {
    low_step_slope_bypass_height_ = max_step_height_;
  }
  slope_fit_radius_ = std::max(1, slope_fit_radius_);
  min_slope_fit_neighbors_ = std::max(3, min_slope_fit_neighbors_);
  soft_cost_max_ = std::max(1, std::min(252, soft_cost_max_));
  decay_rate_per_second_ = std::max(0.0, decay_rate_per_second_);
  decay_minimum_cost_ = std::max(1, std::min(soft_cost_max_, decay_minimum_cost_));
  if (reset_each_update_) {
    decay_enabled_ = false;
  }
}

// Initialize the plugin, allocate storage, and subscribe to terrain input.
void TraversabilityLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("TraversabilityLayer node expired");
  }

  declareParameters();
  readParameters();
  matchSize();
  resetLayerCostmap();
  last_decay_stamp_ = clock_->now();
  current_ = false;

  pointcloud_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    pointcloud_topic_, rclcpp::SensorDataQoS(),
    std::bind(&TraversabilityLayer::pointCloudCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    logger_,
    "TraversabilityLayer [%s] subscribed to [%s], target_frame=[%s], reset_each_update=%s, decay_enabled=%s, decay_rate_per_second=%.3f",
    name_.c_str(), pointcloud_topic_.c_str(), target_frame_.c_str(),
    reset_each_update_ ? "true" : "false",
    decay_enabled_ ? "true" : "false", decay_rate_per_second_);
}

// Match the internal layer dimensions to the parent costmap.
void TraversabilityLayer::matchSize()
{
  CostmapLayer::matchSize();
}

// Reset every cell in this layer back to NO_INFORMATION.
void TraversabilityLayer::resetLayerCostmap()
{
  if (costmap_ == nullptr) {
    return;
  }
  resetMap(0, 0, getSizeInCellsX(), getSizeInCellsY());
  const unsigned int size = getSizeInCellsX() * getSizeInCellsY();
  std::fill(costmap_, costmap_ + size, nav2_costmap_2d::NO_INFORMATION);
}

// Clear all layer state so the next update starts from a clean slate.
void TraversabilityLayer::reset()
{
  resetLayerCostmap();
  has_last_bounds_ = false;
  last_decay_stamp_ = clock_->now();
  current_ = false;
}

// Report that this layer can be cleared by the layered costmap.
bool TraversabilityLayer::isClearable()
{
  return true;
}

// Cache the most recent point cloud without doing heavy work in the callback.
void TraversabilityLayer::pointCloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(cloud_mutex_);
  latest_cloud_ = msg;
  // Build message stamps with the layer clock type so ROS time and system time are never mixed.
  latest_cloud_stamp_ = rclcpp::Time(msg->header.stamp, clock_->get_clock_type());
  current_ = false;
}

// Estimate a representative ground height using percentile or z-layer clustering.
double TraversabilityLayer::estimateGroundZ(std::vector<double> & z_values) const
{
  if (z_values.empty()) {
    return 0.0;
  }

  if (ground_estimation_method_ == "percentile") {
    const size_t idx = static_cast<size_t>(
      std::floor(ground_percentile_ * static_cast<double>(z_values.size() - 1)));
    std::nth_element(z_values.begin(), z_values.begin() + idx, z_values.end());
    return z_values[idx];
  }

  std::sort(z_values.begin(), z_values.end());
  std::vector<ZCluster> clusters;
  ZCluster current;
  current.begin = 0;
  current.min_z = z_values.front();

  for (size_t i = 1; i < z_values.size(); ++i) {
    if (z_values[i] - z_values[i - 1] > ground_cluster_tolerance_) {
      current.end = i;
      current.max_z = z_values[i - 1];
      current.count = current.end - current.begin;
      current.center_z = 0.5 * (current.min_z + current.max_z);
      clusters.push_back(current);

      current = ZCluster{};
      current.begin = i;
      current.min_z = z_values[i];
    }
  }

  current.end = z_values.size();
  current.max_z = z_values.back();
  current.count = current.end - current.begin;
  current.center_z = 0.5 * (current.min_z + current.max_z);
  clusters.push_back(current);

  std::vector<ZCluster>::const_iterator best_it = clusters.begin();
  if (ground_estimation_method_ == "densest") {
    best_it = std::max_element(
      clusters.begin(), clusters.end(),
      [](const ZCluster & lhs, const ZCluster & rhs) {
        return lhs.count < rhs.count;
      });
  } else if (ground_estimation_method_ == "upper_densest") {
    const size_t max_count = std::max_element(
      clusters.begin(), clusters.end(),
      [](const ZCluster & lhs, const ZCluster & rhs) {
        return lhs.count < rhs.count;
      })->count;
    const double lowest_center = clusters.front().center_z;
    const size_t min_count = static_cast<size_t>(
      std::ceil(static_cast<double>(max_count) * ground_layer_count_ratio_));

    best_it = clusters.begin();
    for (std::vector<ZCluster>::const_iterator it = clusters.begin(); it != clusters.end(); ++it) {
      const bool close_to_low_layer = (it->center_z - lowest_center) <= ground_layer_max_gap_;
      if (it->count >= min_count && close_to_low_layer && it->center_z >= best_it->center_z) {
        best_it = it;
      }
    }
  } else {
    const size_t idx = static_cast<size_t>(
      std::floor(ground_percentile_ * static_cast<double>(z_values.size() - 1)));
    return z_values[idx];
  }

  const size_t cluster_size = best_it->end - best_it->begin;
  const size_t idx = static_cast<size_t>(
    std::floor(ground_cluster_percentile_ * static_cast<double>(cluster_size - 1)));
  return z_values[best_it->begin + idx];
}

// Map terrain statistics to either free/soft/lethal cost semantics.
unsigned char TraversabilityLayer::computeCost(const CellStats & cell) const
{
  if (!cell.valid) {
    return unknown_is_lethal_ ? nav2_costmap_2d::LETHAL_OBSTACLE : nav2_costmap_2d::NO_INFORMATION;
  }

  const double slope_deg = std::atan(cell.slope) * 180.0 / M_PI;
  if (cell.obstacle_height >= hard_obstacle_height_ ||
    cell.obstacle_ratio >= hard_obstacle_ratio_threshold_ ||
    cell.height_diff >= hard_step_height_ ||
    slope_deg >= hard_slope_limit_deg_)
  {
    return nav2_costmap_2d::LETHAL_OBSTACLE;
  }

  if (cell.obstacle_height > max_step_height_ &&
    cell.obstacle_ratio >= obstacle_ratio_threshold_)
  {
    return nav2_costmap_2d::LETHAL_OBSTACLE;
  }

  if (cell.height_diff > max_step_height_) {
    return nav2_costmap_2d::LETHAL_OBSTACLE;
  }

  const bool low_step_bypasses_slope =
    allow_low_step_slope_bypass_ && cell.height_diff > flat_step_threshold_ &&
    cell.height_diff <= low_step_slope_bypass_height_;
  if (slope_deg > max_slope_traversable_deg_ && !low_step_bypasses_slope) {
    return nav2_costmap_2d::LETHAL_OBSTACLE;
  }

  double cost_ratio = 0.0;
  if (cell.height_diff > height_cost_start_) {
    const double denom = std::max(1e-3, max_step_height_ - height_cost_start_);
    cost_ratio = std::max(cost_ratio, (cell.height_diff - height_cost_start_) / denom);
  }
  if (cell.obstacle_height > height_cost_start_) {
    const double denom = std::max(1e-3, max_step_height_ - height_cost_start_);
    cost_ratio = std::max(
      cost_ratio,
      ((cell.obstacle_height - height_cost_start_) / denom) *
      std::max(0.1, cell.obstacle_ratio));
  }
  if (slope_deg > slope_cost_start_deg_) {
    const double denom = std::max(1e-3, max_slope_traversable_deg_ - slope_cost_start_deg_);
    cost_ratio = std::max(cost_ratio, (slope_deg - slope_cost_start_deg_) / denom);
  }

  const int cost = static_cast<int>(std::round(clamp01(cost_ratio) * soft_cost_max_));
  return static_cast<unsigned char>(std::max(0, std::min(soft_cost_max_, cost)));
}

// Age out previously accumulated traversability costs for stale dynamic obstacles.
bool TraversabilityLayer::decayStoredCosts(
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!decay_enabled_ || reset_each_update_ || costmap_ == nullptr) {
    last_decay_stamp_ = clock_->now();
    return false;
  }

  const rclcpp::Time now = clock_->now();
  if (last_decay_stamp_.nanoseconds() == 0) {
    last_decay_stamp_ = now;
    return false;
  }

  const double dt = (now - last_decay_stamp_).seconds();
  last_decay_stamp_ = now;
  if (dt <= 0.0 || decay_rate_per_second_ <= 0.0) {
    return false;
  }

  const int decay_delta = std::max(1, static_cast<int>(std::round(decay_rate_per_second_ * dt)));
  const unsigned int size = getSizeInCellsX() * getSizeInCellsY();
  bool changed = false;

  for (unsigned int idx = 0; idx < size; ++idx) {
    unsigned char & cost = costmap_[idx];
    if (cost == nav2_costmap_2d::NO_INFORMATION || cost == nav2_costmap_2d::FREE_SPACE) {
      continue;
    }
    if (cost == nav2_costmap_2d::LETHAL_OBSTACLE) {
      const int decayed_cost = nav2_costmap_2d::LETHAL_OBSTACLE - decay_delta;
      cost = decayed_cost <= decay_minimum_cost_ ?
        nav2_costmap_2d::NO_INFORMATION :
        static_cast<unsigned char>(decayed_cost);
      changed = true;
      continue;
    }

    const int decayed_cost = static_cast<int>(cost) - decay_delta;
    cost = decayed_cost <= decay_minimum_cost_ ?
      nav2_costmap_2d::NO_INFORMATION :
      static_cast<unsigned char>(decayed_cost);
    changed = true;
  }

  if (changed) {
    includeFullLayerBounds(min_x, min_y, max_x, max_y);
  }
  return changed;
}

// Reuse the previous update bounds so late clears also propagate to the master grid.
void TraversabilityLayer::includePreviousBounds(
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!has_last_bounds_) {
    return;
  }
  *min_x = std::min(*min_x, last_min_x_);
  *min_y = std::min(*min_y, last_min_y_);
  *max_x = std::max(*max_x, last_max_x_);
  *max_y = std::max(*max_y, last_max_y_);
}

// Cache the latest observed bounds for the next update cycle.
void TraversabilityLayer::rememberBounds(double min_x, double min_y, double max_x, double max_y)
{
  has_last_bounds_ = true;
  last_min_x_ = min_x;
  last_min_y_ = min_y;
  last_max_x_ = max_x;
  last_max_y_ = max_y;
}

// Expand the update region to the entire layer so decayed cells can be recombined.
void TraversabilityLayer::includeFullLayerBounds(
  double * min_x, double * min_y, double * max_x, double * max_y) const
{
  if (getSizeInCellsX() == 0 || getSizeInCellsY() == 0) {
    return;
  }

  double world_min_x = 0.0;
  double world_min_y = 0.0;
  double world_max_x = 0.0;
  double world_max_y = 0.0;
  mapToWorld(0, 0, world_min_x, world_min_y);
  mapToWorld(getSizeInCellsX() - 1, getSizeInCellsY() - 1, world_max_x, world_max_y);

  *min_x = std::min(*min_x, world_min_x);
  *min_y = std::min(*min_y, world_min_y);
  *max_x = std::max(*max_x, world_max_x);
  *max_y = std::max(*max_y, world_max_y);
}

// Transform the latest point cloud into per-cell terrain statistics and costs.
bool TraversabilityLayer::processLatestCloud(
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud;
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    cloud = latest_cloud_;
  }

  decayStoredCosts(min_x, min_y, max_x, max_y);

  if (!cloud) {
    current_ = false;
    return false;
  }

  // Use the same clock type as clock_->now(); subtracting/comparing rclcpp::Time values with
  // different clock types throws at runtime when use_sim_time is enabled.
  const rclcpp::Time cloud_stamp(cloud->header.stamp, clock_->get_clock_type());
  if (cloud_stamp.nanoseconds() == last_processed_stamp_.nanoseconds()) {
    includePreviousBounds(min_x, min_y, max_x, max_y);
    current_ = true;
    return false;
  }

  const double cloud_age = (clock_->now() - cloud_stamp).seconds();
  if (cloud_age > max_cloud_age_) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "TraversabilityLayer [%s] ignored stale cloud age %.3fs",
      name_.c_str(), cloud_age);
    current_ = false;
    return false;
  }

  geometry_msgs::msg::TransformStamped transform;
  const std::string source_frame = cloud->header.frame_id;
  const std::string target_frame = transform_to_costmap_frame_ ?
    layered_costmap_->getGlobalFrameID() : target_frame_;

  try {
    transform = tf_->lookupTransform(
      target_frame, source_frame, cloud->header.stamp,
      tf2::durationFromSec(transform_tolerance_));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 2000,
      "TraversabilityLayer [%s] TF failed from [%s] to [%s]: %s",
      name_.c_str(), source_frame.c_str(), target_frame.c_str(), ex.what());
    current_ = false;
    return false;
  }

  if (reset_each_update_) {
    resetLayerCostmap();
  }

  const int width = static_cast<int>(getSizeInCellsX());
  const int height = static_cast<int>(getSizeInCellsY());
  std::vector<std::vector<double>> column_z(static_cast<size_t>(width * height));

  double obs_min_x = std::numeric_limits<double>::max();
  double obs_min_y = std::numeric_limits<double>::max();
  double obs_max_x = -std::numeric_limits<double>::max();
  double obs_max_y = -std::numeric_limits<double>::max();
  bool has_observation = false;

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud, "z");
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) {
      continue;
    }

    geometry_msgs::msg::PointStamped point_in;
    geometry_msgs::msg::PointStamped point_out;
    point_in.header = cloud->header;
    point_in.point.x = *iter_x;
    point_in.point.y = *iter_y;
    point_in.point.z = *iter_z;
    tf2::doTransform(point_in, point_out, transform);

    if (point_out.point.z < analysis_z_min_ || point_out.point.z > analysis_z_max_) {
      continue;
    }

    unsigned int mx = 0;
    unsigned int my = 0;
    if (!worldToMap(point_out.point.x, point_out.point.y, mx, my)) {
      continue;
    }

    column_z[static_cast<size_t>(index2d(static_cast<int>(mx), static_cast<int>(my), width))]
      .push_back(point_out.point.z);
    obs_min_x = std::min(obs_min_x, point_out.point.x);
    obs_min_y = std::min(obs_min_y, point_out.point.y);
    obs_max_x = std::max(obs_max_x, point_out.point.x);
    obs_max_y = std::max(obs_max_y, point_out.point.y);
    has_observation = true;
  }

  if (!has_observation) {
    current_ = false;
    return false;
  }

  std::vector<CellStats> cells(static_cast<size_t>(width * height));
  for (size_t idx = 0; idx < cells.size(); ++idx) {
    auto & z_values = column_z[idx];
    if (static_cast<int>(z_values.size()) < min_points_per_cell_) {
      continue;
    }

    auto z_for_ground = z_values;
    const double ground_z = estimateGroundZ(z_for_ground);
    int obstacle_points = 0;
    int observed_layers = 0;
    double max_obstacle_z = ground_z;

    for (const auto z : z_values) {
      if (z <= ground_z + obstacle_min_height_ || z > ground_z + robot_body_height_) {
        continue;
      }
      ++observed_layers;
      if (z > ground_z + max_step_height_) {
        ++obstacle_points;
      }
      max_obstacle_z = std::max(max_obstacle_z, z);
    }

    cells[idx].ground_z = ground_z;
    cells[idx].obstacle_height = std::max(0.0, max_obstacle_z - ground_z);
    cells[idx].obstacle_ratio = observed_layers > 0 ?
      static_cast<double>(obstacle_points) / static_cast<double>(observed_layers) : 0.0;
    cells[idx].valid = true;
  }

  const double resolution = getResolution();
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      auto & cell = cells[static_cast<size_t>(index2d(x, y, width))];
      if (!cell.valid) {
        continue;
      }

      double slope_x = 0.0;
      double slope_y = 0.0;
      double residual_height_diff = 0.0;
      int slope_neighbors = 0;

      if (slope_compensation_enabled_) {
        double sxx = 0.0;
        double syy = 0.0;
        double sxy = 0.0;
        double sxz = 0.0;
        double syz = 0.0;

        for (int dy = -slope_fit_radius_; dy <= slope_fit_radius_; ++dy) {
          for (int dx = -slope_fit_radius_; dx <= slope_fit_radius_; ++dx) {
            if (dx == 0 && dy == 0) {
              continue;
            }
            const int nx = x + dx;
            const int ny = y + dy;
            if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
              continue;
            }
            const auto & neighbor = cells[static_cast<size_t>(index2d(nx, ny, width))];
            if (!neighbor.valid) {
              continue;
            }

            const double local_x = static_cast<double>(dx) * resolution;
            const double local_y = static_cast<double>(dy) * resolution;
            const double dz = neighbor.ground_z - cell.ground_z;
            const double weight = 1.0 / std::max(1.0, static_cast<double>(dx * dx + dy * dy));
            sxx += weight * local_x * local_x;
            syy += weight * local_y * local_y;
            sxy += weight * local_x * local_y;
            sxz += weight * local_x * dz;
            syz += weight * local_y * dz;
            ++slope_neighbors;
          }
        }

        const double det = sxx * syy - sxy * sxy;
        if (slope_neighbors >= min_slope_fit_neighbors_ && std::abs(det) > 1e-9) {
          slope_x = (sxz * syy - syz * sxy) / det;
          slope_y = (syz * sxx - sxz * sxy) / det;

          for (int dy = -slope_fit_radius_; dy <= slope_fit_radius_; ++dy) {
            for (int dx = -slope_fit_radius_; dx <= slope_fit_radius_; ++dx) {
              if (dx == 0 && dy == 0) {
                continue;
              }
              const int nx = x + dx;
              const int ny = y + dy;
              if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
                continue;
              }
              const auto & neighbor = cells[static_cast<size_t>(index2d(nx, ny, width))];
              if (!neighbor.valid) {
                continue;
              }

              const double local_x = static_cast<double>(dx) * resolution;
              const double local_y = static_cast<double>(dy) * resolution;
              const double expected_z = cell.ground_z + slope_x * local_x + slope_y * local_y;
              residual_height_diff =
                std::max(residual_height_diff, std::abs(neighbor.ground_z - expected_z));
            }
          }
        } else {
          slope_neighbors = 0;
        }
      }

      if (!slope_compensation_enabled_ || slope_neighbors < min_slope_fit_neighbors_) {
        double z_xp = cell.ground_z;
        double z_xm = cell.ground_z;
        double z_yp = cell.ground_z;
        double z_ym = cell.ground_z;
        bool has_xp = false;
        bool has_xm = false;
        bool has_yp = false;
        bool has_ym = false;

        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            if (dx == 0 && dy == 0) {
              continue;
            }
            const int nx = x + dx;
            const int ny = y + dy;
            if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
              continue;
            }
            const auto & neighbor = cells[static_cast<size_t>(index2d(nx, ny, width))];
            if (!neighbor.valid) {
              continue;
            }
            residual_height_diff =
              std::max(residual_height_diff, std::abs(neighbor.ground_z - cell.ground_z));
            if (dx == 1 && dy == 0) {
              z_xp = neighbor.ground_z;
              has_xp = true;
            } else if (dx == -1 && dy == 0) {
              z_xm = neighbor.ground_z;
              has_xm = true;
            } else if (dx == 0 && dy == 1) {
              z_yp = neighbor.ground_z;
              has_yp = true;
            } else if (dx == 0 && dy == -1) {
              z_ym = neighbor.ground_z;
              has_ym = true;
            }
          }
        }

        if (has_xp && has_xm) {
          slope_x = (z_xp - z_xm) / (2.0 * resolution);
        } else if (has_xp) {
          slope_x = (z_xp - cell.ground_z) / resolution;
        } else if (has_xm) {
          slope_x = (cell.ground_z - z_xm) / resolution;
        }

        if (has_yp && has_ym) {
          slope_y = (z_yp - z_ym) / (2.0 * resolution);
        } else if (has_yp) {
          slope_y = (z_yp - cell.ground_z) / resolution;
        } else if (has_ym) {
          slope_y = (cell.ground_z - z_ym) / resolution;
        }
      }

      cell.height_diff = residual_height_diff;
      cell.slope = std::sqrt(slope_x * slope_x + slope_y * slope_y);
    }
  }

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const auto & cell = cells[static_cast<size_t>(index2d(x, y, width))];
      if (!cell.valid) {
        continue;
      }
      const unsigned char cost = computeCost(cell);
      const unsigned int idx = getIndex(static_cast<unsigned int>(x), static_cast<unsigned int>(y));
      costmap_[idx] = std::max(costmap_[idx], cost);
    }
  }

  last_processed_stamp_ = cloud_stamp;
  current_ = true;

  *min_x = std::min(*min_x, obs_min_x);
  *min_y = std::min(*min_y, obs_min_y);
  *max_x = std::max(*max_x, obs_max_x);
  *max_y = std::max(*max_y, obs_max_y);
  includePreviousBounds(min_x, min_y, max_x, max_y);
  rememberBounds(obs_min_x, obs_min_y, obs_max_x, obs_max_y);
  return true;
}

// Ask the layer to refresh its observed region before master costmap composition.
void TraversabilityLayer::updateBounds(
  double, double, double, double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) {
    return;
  }

  processLatestCloud(min_x, min_y, max_x, max_y);
  useExtraBounds(min_x, min_y, max_x, max_y);
}

// Combine this layer with the master costmap using max-cost semantics.
void TraversabilityLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }
  updateWithMax(master_grid, min_i, min_j, max_i, max_j);
}

}  // namespace nav2_traversability_layer

PLUGINLIB_EXPORT_CLASS(
  nav2_traversability_layer::TraversabilityLayer,
  nav2_costmap_2d::Layer)
