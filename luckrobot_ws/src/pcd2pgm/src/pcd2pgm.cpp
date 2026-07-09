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
#include <cmath>
#include <limits>
#include <utility>

#include "pcl/common/transforms.h"
#include "pcl/filters/radius_outlier_removal.h"
#include "pcl/io/pcd_io.h"
#include "pcl_conversions/pcl_conversions.h"

namespace pcd2pgm
{
namespace
{
struct CellColumn
{
  float ground_z{std::numeric_limits<float>::quiet_NaN()};
  float obstacle_top{std::numeric_limits<float>::quiet_NaN()};
  bool valid{false};
};

int index2d(int x, int y, int width) { return x + y * width; }

bool isInside(int x, int y, int width, int height)
{
  return x >= 0 && x < width && y >= 0 && y < height;
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

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_, *pcd_cloud_) == -1) {
    RCLCPP_ERROR(get_logger(), "Couldn't read file: %s", pcd_file_.c_str());
    return;
  }

  RCLCPP_INFO(get_logger(), "Initial point cloud size: %lu", pcd_cloud_->points.size());

  applyTransform();

  if (terrain_mode_) {
    RCLCPP_INFO(
      get_logger(),
      "Terrain mode enabled: max_step_height=%.2f m, robot_body_height=%.2f m",
      max_step_height_, robot_body_height_);
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
  declare_parameter("pcd_file", "");
  declare_parameter("thre_z_min", 0.5);
  declare_parameter("thre_z_max", 2.0);
  declare_parameter("flag_pass_through", false);
  declare_parameter("terrain_mode", false);
  declare_parameter("max_step_height", 0.20);
  declare_parameter("robot_body_height", 0.60);
  declare_parameter("analysis_z_min", -1.0);
  declare_parameter("analysis_z_max", 2.0);
  declare_parameter("ground_percentile", 0.05);
  declare_parameter("obstacle_percentile", 0.95);
  declare_parameter("flat_step_threshold", 0.02);
  declare_parameter("step_cost_max", 120);
  declare_parameter("min_points_per_cell", 3);
  declare_parameter("thre_radius", 0.5);
  declare_parameter("map_resolution", 0.05);
  declare_parameter("thres_point_count", 10);
  declare_parameter("map_topic_name", "map");
  declare_parameter(
    "odom_to_lidar_odom", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
}

void Pcd2PgmNode::getParameters()
{
  get_parameter("pcd_file", pcd_file_);
  get_parameter("thre_z_min", thre_z_min_);
  get_parameter("thre_z_max", thre_z_max_);
  get_parameter("flag_pass_through", flag_pass_through_);
  get_parameter("terrain_mode", terrain_mode_);
  get_parameter("max_step_height", max_step_height_);
  get_parameter("robot_body_height", robot_body_height_);
  get_parameter("analysis_z_min", analysis_z_min_);
  get_parameter("analysis_z_max", analysis_z_max_);
  get_parameter("ground_percentile", ground_percentile_);
  get_parameter("obstacle_percentile", obstacle_percentile_);
  get_parameter("flat_step_threshold", flat_step_threshold_);
  get_parameter("step_cost_max", step_cost_max_);
  get_parameter("min_points_per_cell", min_points_per_cell_);
  get_parameter("thre_radius", thre_radius_);
  get_parameter("map_resolution", map_resolution_);
  get_parameter("thres_point_count", thres_point_count_);
  get_parameter("map_topic_name", map_topic_name_);
  get_parameter("odom_to_lidar_odom", odom_to_lidar_odom_);
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

    cells[idx].ground_z = computePercentile(z_values, ground_percentile_);
    cells[idx].obstacle_top = computePercentile(z_values, obstacle_percentile_);
    cells[idx].valid = true;
  }

  const std::vector<std::pair<int, int>> neighbors = {
    {1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

  int lethal_cells = 0;
  int step_cells = 0;
  int free_cells = 0;

  for (int j = 0; j < height; ++j) {
    for (int i = 0; i < width; ++i) {
      const size_t idx = static_cast<size_t>(index2d(i, j, width));
      const auto & cell = cells[idx];
      if (!cell.valid) {
        continue;
      }

      const float vertical_span = cell.obstacle_top - cell.ground_z;
      if (vertical_span > robot_body_height_) {
        msg.data[idx] = 254;
        ++lethal_cells;
        continue;
      }

      float max_step = 0.0f;
      for (const auto & offset : neighbors) {
        const int nx = i + offset.first;
        const int ny = j + offset.second;
        if (!isInside(nx, ny, width, height)) {
          continue;
        }

        const auto & neighbor = cells[static_cast<size_t>(index2d(nx, ny, width))];
        if (!neighbor.valid) {
          continue;
        }

        max_step = std::max(max_step, std::abs(cell.ground_z - neighbor.ground_z));
      }

      if (max_step > max_step_height_) {
        msg.data[idx] = 254;
        ++lethal_cells;
      } else if (max_step > flat_step_threshold_) {
        const int cost = static_cast<int>(
          std::round((max_step / max_step_height_) * static_cast<float>(step_cost_max_)));
        msg.data[idx] = static_cast<int8_t>(std::min(cost, 253));
        ++step_cells;
      } else {
        msg.data[idx] = 0;
        ++free_cells;
      }
    }
  }

  RCLCPP_INFO(
    get_logger(),
    "Terrain map: %dx%d, free=%d, step=%d, lethal=%d, unknown=%ld", width, height, free_cells,
    step_cells, lethal_cells,
    static_cast<long>(msg.data.size()) - free_cells - step_cells - lethal_cells);
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
