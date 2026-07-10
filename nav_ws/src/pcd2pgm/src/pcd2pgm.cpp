// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "pcd2pgm/pcd2pgm.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <string>
#include <utility>

#include "pcl/common/transforms.h"
#include "pcl/filters/radius_outlier_removal.h"
#include "pcl/io/pcd_io.h"
#include "pcl/io/ply_io.h"
#include "pcl_conversions/pcl_conversions.h"

namespace pcd2pgm
{
namespace
{
struct CellColumn
{
  float ground_z{std::numeric_limits<float>::quiet_NaN()};
  float obstacle_height{0.0f};
  float obstacle_ratio{0.0f};
  float height_diff{0.0f};
  float slope_magnitude{0.0f};
  bool valid{false};
  bool interpolated{false};
};

struct ZCluster
{
  size_t begin{0};
  size_t end{0};
  float min_z{0.0f};
  float max_z{0.0f};
  float center_z{0.0f};
  size_t count{0};
};

int index2d(int x, int y, int width) { return x + y * width; }

bool isInside(int x, int y, int width, int height)
{
  return x >= 0 && x < width && y >= 0 && y < height;
}

std::string getLowercaseExtension(const std::string & path)
{
  const auto dot_pos = path.find_last_of('.');
  if (dot_pos == std::string::npos) {
    return "";
  }

  std::string extension = path.substr(dot_pos);
  std::transform(extension.begin(), extension.end(), extension.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return extension;
}
}  // namespace

Pcd2PgmNode::Pcd2PgmNode(const rclcpp::NodeOptions & options) : Node("pcd2pgm", options)
{
  declareParameters();
  getParameters();

  rclcpp::QoS map_qos(10);
  map_qos.transient_local();
  map_qos.reliable();
  map_qos.keep_last(1);

  pcd_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic_name_, map_qos);
  pcd_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pcd_cloud", 10);

  if (!loadPointCloud()) {
    return;
  }

  RCLCPP_INFO(get_logger(), "Initial point cloud size: %lu", pcd_cloud_->points.size());

  applyTransform();

  if (terrain_mode_) {
    RCLCPP_INFO(
      get_logger(),
      "Terrain mode enabled: max_step_height=%.2f m, robot_body_height=%.2f m, max_slope=%.1f deg",
      max_step_height_, robot_body_height_, max_slope_traversable_);
    auto analysis_cloud = filterAnalysisCloud();
    setTerrainMapTopicMsg(analysis_cloud, map_topic_msg_);
  } else {
    passThroughFilter(thre_z_min_, thre_z_max_, flag_pass_through_);
    radiusOutlierFilter(cloud_after_pass_through_, thre_radius_, thres_point_count_);
    setMapTopicMsg(cloud_after_radius_, map_topic_msg_);
  }

  passThroughFilter(thre_z_min_, thre_z_max_, flag_pass_through_);
  radiusOutlierFilter(cloud_after_pass_through_, thre_radius_, thres_point_count_);

  timer_ =
    create_wall_timer(std::chrono::seconds(1), std::bind(&Pcd2PgmNode::publishCallback, this));
}

void Pcd2PgmNode::publishCallback()
{
  sensor_msgs::msg::PointCloud2 output;
  pcl::toROSMsg(*cloud_after_radius_, output);
  output.header.frame_id = "map";
  pcd_publisher_->publish(output);
  map_publisher_->publish(map_topic_msg_);
}

void Pcd2PgmNode::declareParameters()
{
  declare_parameter("input_file", "");
  declare_parameter("thre_z_min", 0.5);
  declare_parameter("thre_z_max", 2.0);
  declare_parameter("flag_pass_through", false);
  declare_parameter("terrain_mode", false);
  declare_parameter("max_step_height", 0.20);
  declare_parameter("robot_body_height", 0.60);
  declare_parameter("analysis_z_min", -1.0);
  declare_parameter("analysis_z_max", 2.0);
  declare_parameter("ground_percentile", 0.05);
  declare_parameter("ground_estimation_method", "percentile");
  declare_parameter("ground_cluster_tolerance", 0.06);
  declare_parameter("ground_cluster_percentile", 0.50);
  declare_parameter("ground_layer_max_gap", 0.35);
  declare_parameter("ground_layer_count_ratio", 0.60);
  declare_parameter("obstacle_percentile", 0.95);
  declare_parameter("flat_step_threshold", 0.02);
  declare_parameter("step_cost_max", 95);
  declare_parameter("min_points_per_cell", 3);
  declare_parameter("obstacle_min_height", 0.05);
  declare_parameter("obstacle_ratio_threshold", 0.45);
  declare_parameter("max_slope_traversable", 45.0);
  declare_parameter("slope_cost_start", 12.0);
  declare_parameter("height_cost_start", 0.03);
  declare_parameter("slope_cost_scale", 1.0);
  declare_parameter("height_cost_scale", 1.0);
  declare_parameter("interp_search_radius", 3);
  declare_parameter("min_interp_neighbors", 2);
  declare_parameter("obstacle_inflation_radius", 1);
  declare_parameter("thre_radius", 0.5);
  declare_parameter("map_resolution", 0.05);
  declare_parameter("thres_point_count", 10);
  declare_parameter("map_topic_name", "map");
  declare_parameter(
    "odom_to_lidar_odom", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
}

void Pcd2PgmNode::getParameters()
{
  get_parameter("input_file", input_file_);
  get_parameter("thre_z_min", thre_z_min_);
  get_parameter("thre_z_max", thre_z_max_);
  get_parameter("flag_pass_through", flag_pass_through_);
  get_parameter("terrain_mode", terrain_mode_);
  get_parameter("max_step_height", max_step_height_);
  get_parameter("robot_body_height", robot_body_height_);
  get_parameter("analysis_z_min", analysis_z_min_);
  get_parameter("analysis_z_max", analysis_z_max_);
  get_parameter("ground_percentile", ground_percentile_);
  get_parameter("ground_estimation_method", ground_estimation_method_);
  get_parameter("ground_cluster_tolerance", ground_cluster_tolerance_);
  get_parameter("ground_cluster_percentile", ground_cluster_percentile_);
  get_parameter("ground_layer_max_gap", ground_layer_max_gap_);
  get_parameter("ground_layer_count_ratio", ground_layer_count_ratio_);
  get_parameter("obstacle_percentile", obstacle_percentile_);
  get_parameter("flat_step_threshold", flat_step_threshold_);
  get_parameter("step_cost_max", step_cost_max_);
  get_parameter("min_points_per_cell", min_points_per_cell_);
  get_parameter("obstacle_min_height", obstacle_min_height_);
  get_parameter("obstacle_ratio_threshold", obstacle_ratio_threshold_);
  get_parameter("max_slope_traversable", max_slope_traversable_);
  get_parameter("slope_cost_start", slope_cost_start_);
  get_parameter("height_cost_start", height_cost_start_);
  get_parameter("slope_cost_scale", slope_cost_scale_);
  get_parameter("height_cost_scale", height_cost_scale_);
  get_parameter("interp_search_radius", interp_search_radius_);
  get_parameter("min_interp_neighbors", min_interp_neighbors_);
  get_parameter("obstacle_inflation_radius", obstacle_inflation_radius_);
  get_parameter("thre_radius", thre_radius_);
  get_parameter("map_resolution", map_resolution_);
  get_parameter("thres_point_count", thres_point_count_);
  get_parameter("map_topic_name", map_topic_name_);
  get_parameter("odom_to_lidar_odom", odom_to_lidar_odom_);

  step_cost_max_ = clampOccupancy(step_cost_max_);
  min_points_per_cell_ = std::max(1, min_points_per_cell_);
  ground_cluster_tolerance_ = std::max(0.01f, ground_cluster_tolerance_);
  ground_layer_max_gap_ = std::max(0.0f, ground_layer_max_gap_);
  ground_layer_count_ratio_ = std::max(0.0f, std::min(ground_layer_count_ratio_, 1.0f));
  interp_search_radius_ = std::max(0, interp_search_radius_);
  min_interp_neighbors_ = std::max(1, min_interp_neighbors_);
  obstacle_inflation_radius_ = std::max(0, obstacle_inflation_radius_);
}

bool Pcd2PgmNode::loadPointCloud()
{
  if (input_file_.empty()) {
    RCLCPP_ERROR(get_logger(), "No input point cloud file set. Use input_file.");
    return false;
  }

  const std::string extension = getLowercaseExtension(input_file_);
  int load_result = -1;
  if (extension == ".pcd") {
    load_result = pcl::io::loadPCDFile<pcl::PointXYZ>(input_file_, *pcd_cloud_);
  } else if (extension == ".ply") {
    load_result = pcl::io::loadPLYFile<pcl::PointXYZ>(input_file_, *pcd_cloud_);
  } else {
    RCLCPP_ERROR(
      get_logger(), "Unsupported point cloud file extension '%s'. Supported: .pcd, .ply",
      extension.c_str());
    return false;
  }

  if (load_result == -1) {
    RCLCPP_ERROR(get_logger(), "Couldn't read point cloud file: %s", input_file_.c_str());
    return false;
  }

  RCLCPP_INFO(get_logger(), "Loaded %s point cloud: %s", extension.c_str(), input_file_.c_str());
  return true;
}

void Pcd2PgmNode::passThroughFilter(double thre_low, double thre_high, bool flag_in)
{
  auto filtered_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::PassThrough<pcl::PointXYZ> passthrough;
  passthrough.setInputCloud(pcd_cloud_);
  passthrough.setFilterFieldName("z");
  passthrough.setFilterLimits(thre_low, thre_high);
  passthrough.setNegative(flag_in);
  passthrough.filter(*filtered_cloud);

  cloud_after_pass_through_ = filtered_cloud;
  RCLCPP_INFO(
    get_logger(), "After PassThrough filtering: %lu points",
    cloud_after_pass_through_->points.size());
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Pcd2PgmNode::filterAnalysisCloud()
{
  auto filtered_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::PassThrough<pcl::PointXYZ> passthrough;
  passthrough.setInputCloud(pcd_cloud_);
  passthrough.setFilterFieldName("z");
  passthrough.setFilterLimits(analysis_z_min_, analysis_z_max_);
  passthrough.setNegative(false);
  passthrough.filter(*filtered_cloud);

  RCLCPP_INFO(
    get_logger(), "Analysis cloud size (z=[%.2f, %.2f]): %lu points", analysis_z_min_,
    analysis_z_max_, filtered_cloud->points.size());
  return filtered_cloud;
}

float Pcd2PgmNode::computePercentile(std::vector<float> & values, float percentile)
{
  if (values.empty()) {
    return std::numeric_limits<float>::quiet_NaN();
  }

  percentile = std::max(0.0f, std::min(percentile, 1.0f));
  std::sort(values.begin(), values.end());
  const size_t idx = static_cast<size_t>(percentile * static_cast<float>(values.size() - 1));
  return values[idx];
}

float Pcd2PgmNode::estimateGroundZ(std::vector<float> & values) const
{
  if (ground_estimation_method_ == "percentile") {
    return computePercentile(values, ground_percentile_);
  }

  if (values.empty()) {
    return std::numeric_limits<float>::quiet_NaN();
  }

  std::sort(values.begin(), values.end());
  std::vector<ZCluster> clusters;
  ZCluster current;
  current.begin = 0;
  current.min_z = values.front();

  for (size_t i = 1; i < values.size(); ++i) {
    if (values[i] - values[i - 1] > ground_cluster_tolerance_) {
      current.end = i;
      current.max_z = values[i - 1];
      current.count = current.end - current.begin;
      current.center_z = 0.5f * (current.min_z + current.max_z);
      clusters.push_back(current);

      current = ZCluster{};
      current.begin = i;
      current.min_z = values[i];
    }
  }

  current.end = values.size();
  current.max_z = values.back();
  current.count = current.end - current.begin;
  current.center_z = 0.5f * (current.min_z + current.max_z);
  clusters.push_back(current);

  auto best_it = clusters.begin();
  if (ground_estimation_method_ == "densest") {
    best_it = std::max_element(clusters.begin(), clusters.end(), [](const auto & lhs, const auto & rhs) {
      return lhs.count < rhs.count;
    });
  } else if (ground_estimation_method_ == "upper_densest") {
    const size_t max_count = std::max_element(
      clusters.begin(), clusters.end(), [](const auto & lhs, const auto & rhs) {
        return lhs.count < rhs.count;
      })->count;
    const float lowest_center = clusters.front().center_z;
    const size_t min_count = static_cast<size_t>(
      std::ceil(static_cast<float>(max_count) * ground_layer_count_ratio_));

    best_it = clusters.begin();
    for (auto it = clusters.begin(); it != clusters.end(); ++it) {
      const bool close_to_low_layer = (it->center_z - lowest_center) <= ground_layer_max_gap_;
      if (it->count >= min_count && close_to_low_layer && it->center_z >= best_it->center_z) {
        best_it = it;
      }
    }
  } else {
    return computePercentile(values, ground_percentile_);
  }

  const size_t cluster_size = best_it->end - best_it->begin;
  const float percentile = std::max(0.0f, std::min(ground_cluster_percentile_, 1.0f));
  const size_t offset = static_cast<size_t>(percentile * static_cast<float>(cluster_size - 1));
  return values[best_it->begin + offset];
}

int Pcd2PgmNode::clampOccupancy(int value)
{
  return std::max(0, std::min(value, 100));
}

int Pcd2PgmNode::computeTerrainOccupancy(
  float obstacle_height, float obstacle_ratio, float height_diff, float slope) const
{
  constexpr float kPi = 3.14159265358979323846f;
  const float slope_angle_deg = std::atan(slope) * 180.0f / kPi;

  if (slope_angle_deg > max_slope_traversable_) {
    return 100;
  }

  if (height_diff > max_step_height_) {
    return 100;
  }

  if (
    obstacle_height > max_step_height_ &&
    obstacle_ratio >= obstacle_ratio_threshold_)
  {
    return 100;
  }

  int cost = 0;
  if (height_diff > height_cost_start_) {
    const float range = std::max(1e-3f, max_step_height_ - height_cost_start_);
    const float ratio = std::min(1.0f, (height_diff - height_cost_start_) / range);
    cost = std::max(cost, static_cast<int>(std::round(ratio * step_cost_max_ * height_cost_scale_)));
  }

  if (obstacle_height > height_cost_start_) {
    const float range = std::max(1e-3f, max_step_height_ - height_cost_start_);
    const float ratio = std::min(1.0f, (obstacle_height - height_cost_start_) / range);
    const float density = std::min(1.0f, obstacle_ratio / std::max(1e-3f, obstacle_ratio_threshold_));
    cost = std::max(
      cost, static_cast<int>(std::round(ratio * density * step_cost_max_ * height_cost_scale_)));
  }

  if (slope_angle_deg > slope_cost_start_) {
    const float range = std::max(1e-3f, max_slope_traversable_ - slope_cost_start_);
    const float ratio = std::min(1.0f, (slope_angle_deg - slope_cost_start_) / range);
    cost = std::max(cost, static_cast<int>(std::round(ratio * step_cost_max_ * slope_cost_scale_)));
  }

  return clampOccupancy(std::min(cost, 99));
}

void Pcd2PgmNode::radiusOutlierFilter(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & input_cloud, double radius, int thre_count)
{
  auto filtered_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> radius_outlier;
  radius_outlier.setInputCloud(input_cloud);
  radius_outlier.setRadiusSearch(radius);
  radius_outlier.setMinNeighborsInRadius(thre_count);
  radius_outlier.filter(*filtered_cloud);

  cloud_after_radius_ = filtered_cloud;
  RCLCPP_INFO(
    get_logger(), "After RadiusOutlier filtering: %lu points", cloud_after_radius_->points.size());
}

void Pcd2PgmNode::setMapTopicMsg(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, nav_msgs::msg::OccupancyGrid & msg)
{
  msg.header.stamp = now();
  msg.header.frame_id = "map";

  msg.info.map_load_time = now();
  msg.info.resolution = map_resolution_;

  double x_min = std::numeric_limits<double>::max();
  double x_max = std::numeric_limits<double>::lowest();
  double y_min = std::numeric_limits<double>::max();
  double y_max = std::numeric_limits<double>::lowest();

  if (cloud->points.empty()) {
    RCLCPP_WARN(get_logger(), "Point cloud is empty!");
    return;
  }

  for (const auto & point : cloud->points) {
    x_min = std::min(x_min, static_cast<double>(point.x));
    x_max = std::max(x_max, static_cast<double>(point.x));
    y_min = std::min(y_min, static_cast<double>(point.y));
    y_max = std::max(y_max, static_cast<double>(point.y));
  }

  msg.info.origin.position.x = x_min;
  msg.info.origin.position.y = y_min;
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.x = 0.0;
  msg.info.origin.orientation.y = 0.0;
  msg.info.origin.orientation.z = 0.0;
  msg.info.origin.orientation.w = 1.0;

  msg.info.width = std::ceil((x_max - x_min) / map_resolution_);
  msg.info.height = std::ceil((y_max - y_min) / map_resolution_);
  msg.data.assign(msg.info.width * msg.info.height, 0);

  for (const auto & point : cloud->points) {
    int i = std::floor((point.x - x_min) / map_resolution_);
    int j = std::floor((point.y - y_min) / map_resolution_);

    if (i >= 0 && i < static_cast<int>(msg.info.width) && j >= 0 &&
        j < static_cast<int>(msg.info.height)) {
      msg.data[index2d(i, j, msg.info.width)] = 100;
    }
  }

  RCLCPP_INFO(get_logger(), "Map data size: %lu", msg.data.size());
}

void Pcd2PgmNode::setTerrainMapTopicMsg(
  const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, nav_msgs::msg::OccupancyGrid & msg)
{
  msg.header.stamp = now();
  msg.header.frame_id = "map";
  msg.info.map_load_time = now();
  msg.info.resolution = map_resolution_;

  if (cloud->points.empty()) {
    RCLCPP_WARN(get_logger(), "Analysis point cloud is empty!");
    return;
  }

  double x_min = std::numeric_limits<double>::max();
  double x_max = std::numeric_limits<double>::lowest();
  double y_min = std::numeric_limits<double>::max();
  double y_max = std::numeric_limits<double>::lowest();

  for (const auto & point : cloud->points) {
    x_min = std::min(x_min, static_cast<double>(point.x));
    x_max = std::max(x_max, static_cast<double>(point.x));
    y_min = std::min(y_min, static_cast<double>(point.y));
    y_max = std::max(y_max, static_cast<double>(point.y));
  }

  msg.info.origin.position.x = x_min;
  msg.info.origin.position.y = y_min;
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.w = 1.0;

  const int width = static_cast<int>(std::ceil((x_max - x_min) / map_resolution_));
  const int height = static_cast<int>(std::ceil((y_max - y_min) / map_resolution_));
  msg.info.width = width;
  msg.info.height = height;
  msg.data.assign(static_cast<size_t>(width * height), -1);

  std::vector<std::vector<float>> column_z(static_cast<size_t>(width * height));
  for (const auto & point : cloud->points) {
    const int i = static_cast<int>(std::floor((point.x - x_min) / map_resolution_));
    const int j = static_cast<int>(std::floor((point.y - y_min) / map_resolution_));
    if (!isInside(i, j, width, height)) {
      continue;
    }
    column_z[static_cast<size_t>(index2d(i, j, width))].push_back(point.z);
  }

  std::vector<CellColumn> cells(static_cast<size_t>(width * height));
  for (size_t idx = 0; idx < cells.size(); ++idx) {
    auto & z_values = column_z[idx];
    if (static_cast<int>(z_values.size()) < min_points_per_cell_) {
      continue;
    }

    const float ground_z = estimateGroundZ(z_values);
    int obstacle_points = 0;
    int observed_layers = 0;
    float max_obstacle_z = ground_z;

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
    cells[idx].obstacle_height = std::max(0.0f, max_obstacle_z - ground_z);
    cells[idx].obstacle_ratio =
      observed_layers > 0 ? static_cast<float>(obstacle_points) / static_cast<float>(observed_layers) : 0.0f;
    cells[idx].valid = true;

  }

  if (interp_search_radius_ > 0) {
    auto interpolated = cells;
    for (int j = 0; j < height; ++j) {
      for (int i = 0; i < width; ++i) {
        const size_t idx = static_cast<size_t>(index2d(i, j, width));
        if (cells[idx].valid) {
          continue;
        }

        float sum_w = 0.0f;
        float sum_z = 0.0f;
        int neighbor_count = 0;
        for (int dy = -interp_search_radius_; dy <= interp_search_radius_; ++dy) {
          for (int dx = -interp_search_radius_; dx <= interp_search_radius_; ++dx) {
            if (dx == 0 && dy == 0) {
              continue;
            }
            const int nx = i + dx;
            const int ny = j + dy;
            if (!isInside(nx, ny, width, height)) {
              continue;
            }

            const auto & neighbor = cells[static_cast<size_t>(index2d(nx, ny, width))];
            if (!neighbor.valid) {
              continue;
            }

            const float dist = std::sqrt(static_cast<float>(dx * dx + dy * dy));
            const float weight = 1.0f / std::max(1.0f, dist * dist);
            sum_w += weight;
            sum_z += weight * neighbor.ground_z;
            ++neighbor_count;
          }
        }

        if (neighbor_count >= min_interp_neighbors_ && sum_w > 1e-6f) {
          interpolated[idx].ground_z = sum_z / sum_w;
          interpolated[idx].valid = true;
          interpolated[idx].interpolated = true;
        }
      }
    }
    cells = std::move(interpolated);
  }

  for (int j = 0; j < height; ++j) {
    for (int i = 0; i < width; ++i) {
      const size_t idx = static_cast<size_t>(index2d(i, j, width));
      auto & cell = cells[idx];
      if (!cell.valid) {
        continue;
      }

      float z_xp = cell.ground_z;
      float z_xm = cell.ground_z;
      float z_yp = cell.ground_z;
      float z_ym = cell.ground_z;
      bool has_xp = false;
      bool has_xm = false;
      bool has_yp = false;
      bool has_ym = false;
      float max_diff = 0.0f;

      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          if (dx == 0 && dy == 0) {
            continue;
          }
          const int nx = i + dx;
          const int ny = j + dy;
          if (!isInside(nx, ny, width, height)) {
            continue;
          }
          const auto & neighbor = cells[static_cast<size_t>(index2d(nx, ny, width))];
          if (!neighbor.valid) {
            continue;
          }
          max_diff = std::max(max_diff, std::abs(neighbor.ground_z - cell.ground_z));

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

      float slope_x = 0.0f;
      float slope_y = 0.0f;
      if (has_xp && has_xm) {
        slope_x = (z_xp - z_xm) / (2.0f * map_resolution_);
      } else if (has_xp) {
        slope_x = (z_xp - cell.ground_z) / map_resolution_;
      } else if (has_xm) {
        slope_x = (cell.ground_z - z_xm) / map_resolution_;
      }

      if (has_yp && has_ym) {
        slope_y = (z_yp - z_ym) / (2.0f * map_resolution_);
      } else if (has_yp) {
        slope_y = (z_yp - cell.ground_z) / map_resolution_;
      } else if (has_ym) {
        slope_y = (cell.ground_z - z_ym) / map_resolution_;
      }

      cell.height_diff = max_diff;
      cell.slope_magnitude = std::sqrt(slope_x * slope_x + slope_y * slope_y);
    }
  }

  for (int j = 0; j < height; ++j) {
    for (int i = 0; i < width; ++i) {
      const size_t idx = static_cast<size_t>(index2d(i, j, width));
      const auto & cell = cells[idx];
      if (!cell.valid) {
        continue;
      }

      int occupancy = computeTerrainOccupancy(
        cell.obstacle_height, cell.obstacle_ratio, cell.height_diff, cell.slope_magnitude);
      if (cell.interpolated) {
        occupancy = std::min(occupancy, std::max(0, step_cost_max_ / 3));
      }
      msg.data[idx] = static_cast<int8_t>(occupancy);
    }
  }

  if (obstacle_inflation_radius_ > 0) {
    auto inflated = msg.data;
    for (int j = 0; j < height; ++j) {
      for (int i = 0; i < width; ++i) {
        const size_t idx = static_cast<size_t>(index2d(i, j, width));
        if (msg.data[idx] < 100) {
          continue;
        }
        for (int dy = -obstacle_inflation_radius_; dy <= obstacle_inflation_radius_; ++dy) {
          for (int dx = -obstacle_inflation_radius_; dx <= obstacle_inflation_radius_; ++dx) {
            const int nx = i + dx;
            const int ny = j + dy;
            if (!isInside(nx, ny, width, height)) {
              continue;
            }
            const float dist = std::sqrt(static_cast<float>(dx * dx + dy * dy));
            if (dist > static_cast<float>(obstacle_inflation_radius_)) {
              continue;
            }
            const size_t nidx = static_cast<size_t>(index2d(nx, ny, width));
            if (inflated[nidx] < 0 || inflated[nidx] >= 100) {
              continue;
            }
            const float ratio = 1.0f - dist / static_cast<float>(obstacle_inflation_radius_ + 1);
            inflated[nidx] = static_cast<int8_t>(
              std::max<int>(inflated[nidx], clampOccupancy(static_cast<int>(std::round(ratio * step_cost_max_)))));
          }
        }
      }
    }
    msg.data = std::move(inflated);
  }

  int lethal_cells = 0;
  int cost_cells = 0;
  int free_cells = 0;
  int unknown_cells = 0;
  for (const auto value : msg.data) {
    if (value < 0) {
      ++unknown_cells;
    } else if (value >= 100) {
      ++lethal_cells;
    } else if (value > 0) {
      ++cost_cells;
    } else {
      ++free_cells;
    }
  }

  RCLCPP_INFO(
    get_logger(),
    "Terrain map: %dx%d, free=%d, cost=%d, lethal=%d, unknown=%d", width, height, free_cells,
    cost_cells, lethal_cells, unknown_cells);
}

void Pcd2PgmNode::applyTransform()
{
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();

  transform.translation() << odom_to_lidar_odom_[0], odom_to_lidar_odom_[1], odom_to_lidar_odom_[2];
  transform.rotate(Eigen::AngleAxisf(odom_to_lidar_odom_[3], Eigen::Vector3f::UnitX()));
  transform.rotate(Eigen::AngleAxisf(odom_to_lidar_odom_[4], Eigen::Vector3f::UnitY()));
  transform.rotate(Eigen::AngleAxisf(odom_to_lidar_odom_[5], Eigen::Vector3f::UnitZ()));

  pcl::transformPointCloud(*pcd_cloud_, *pcd_cloud_, transform.inverse());
}

}  // namespace pcd2pgm

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pcd2pgm::Pcd2PgmNode)
