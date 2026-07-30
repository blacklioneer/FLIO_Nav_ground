#ifndef NAV2_TRAVERSABILITY_LAYER__TRAVERSABILITY_LAYER_HPP_
#define NAV2_TRAVERSABILITY_LAYER__TRAVERSABILITY_LAYER_HPP_

#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace nav2_traversability_layer
{

class TraversabilityLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  // Construct the traversability layer plugin.
  TraversabilityLayer();
  ~TraversabilityLayer() override = default;

  // Initialize plugin state, parameters, TF, and subscriptions.
  void onInitialize() override;
  // Expand update bounds and process the latest terrain observation.
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  // Merge this layer's costs into the master costmap.
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;
  // Resize the internal costmap when the parent costmap changes size.
  void matchSize() override;
  // Clear cached costs and mark the layer as not current.
  void reset() override;
  // Advertise that this layer supports clearing.
  bool isClearable() override;

private:
  struct CellStats
  {
    bool valid{false};
    double ground_z{0.0};
    double obstacle_height{0.0};
    double obstacle_ratio{0.0};
    double height_diff{0.0};
    double slope{0.0};
  };

  struct ZCluster
  {
    size_t begin{0};
    size_t end{0};
    double min_z{0.0};
    double max_z{0.0};
    double center_z{0.0};
    size_t count{0};
  };

  // Store the newest point cloud for the next costmap update cycle.
  void pointCloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  // Convert the latest point cloud into traversability costs for this layer.
  bool processLatestCloud(double * min_x, double * min_y, double * max_x, double * max_y);
  // Estimate the local ground height from the per-cell z distribution.
  double estimateGroundZ(std::vector<double> & z_values) const;
  // Convert per-cell terrain statistics into a nav2 cost value.
  unsigned char computeCost(const CellStats & cell) const;
  // Declare all ROS parameters supported by this layer.
  void declareParameters();
  // Read ROS parameters into cached member variables.
  void readParameters();
  // Reset the full layer costmap back to NO_INFORMATION.
  void resetLayerCostmap();
  // Decay previously accumulated costs to clear stale dynamic obstacles.
  bool decayStoredCosts(double * min_x, double * min_y, double * max_x, double * max_y);
  // Expand the update region with the previous cycle's bounds.
  void includePreviousBounds(double * min_x, double * min_y, double * max_x, double * max_y);
  // Cache the latest update bounds for reuse on subsequent cycles.
  void rememberBounds(double min_x, double min_y, double max_x, double max_y);
  // Expand the update region to cover the full layer costmap.
  void includeFullLayerBounds(double * min_x, double * min_y, double * max_x, double * max_y) const;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr latest_cloud_;
  rclcpp::Time latest_cloud_stamp_;
  rclcpp::Time last_processed_stamp_;
  std::mutex cloud_mutex_;

  std::string pointcloud_topic_;
  std::string target_frame_;
  double transform_tolerance_{0.05};
  double max_cloud_age_{0.5};
  double analysis_z_min_{-1.0};
  double analysis_z_max_{2.0};
  int min_points_per_cell_{4};
  double ground_percentile_{0.2};
  std::string ground_estimation_method_{"upper_densest"};
  double ground_cluster_tolerance_{0.05};
  double ground_cluster_percentile_{0.50};
  double ground_layer_max_gap_{0.30};
  double ground_layer_count_ratio_{0.55};
  double max_step_height_{0.24};
  double robot_body_height_{1.2};
  double obstacle_min_height_{0.06};
  double obstacle_ratio_threshold_{0.18};
  double max_slope_traversable_deg_{30.0};
  double slope_cost_start_deg_{20.0};
  double height_cost_start_{0.05};
  bool slope_compensation_enabled_{true};
  int slope_fit_radius_{2};
  int min_slope_fit_neighbors_{5};
  bool allow_low_step_slope_bypass_{true};
  double low_step_slope_bypass_height_{0.0};
  double flat_step_threshold_{0.02};
  double hard_obstacle_height_{0.35};
  double hard_obstacle_ratio_threshold_{0.30};
  double hard_slope_limit_deg_{50.0};
  double hard_step_height_{0.35};
  int soft_cost_max_{200};
  bool unknown_is_lethal_{false};
  bool reset_each_update_{true};
  bool transform_to_costmap_frame_{true};
  bool decay_enabled_{false};
  double decay_rate_per_second_{0.0};
  int decay_minimum_cost_{1};

  bool has_last_bounds_{false};
  double last_min_x_{0.0};
  double last_min_y_{0.0};
  double last_max_x_{0.0};
  double last_max_y_{0.0};
  rclcpp::Time last_decay_stamp_;
};

}  // namespace nav2_traversability_layer

#endif  // NAV2_TRAVERSABILITY_LAYER__TRAVERSABILITY_LAYER_HPP_
