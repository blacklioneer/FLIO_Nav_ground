#ifndef NAV2_CUSTOM_PLANNER__NAV2_CUSTOM_PLANNER_HPP_
#define NAV2_CUSTOM_PLANNER__NAV2_CUSTOM_PLANNER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <queue>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav_msgs/msg/path.hpp"

namespace nav2_custom_planner {

struct AStarNode {
  unsigned int x, y;
  double g_cost, h_cost, f_cost;
  std::shared_ptr<AStarNode> parent;

  AStarNode(unsigned int x, unsigned int y, double g, double h, std::shared_ptr<AStarNode> p = nullptr)
      : x(x), y(y), g_cost(g), h_cost(h), f_cost(g + h), parent(p) {}
};

struct CompareNode {
  bool operator()(const std::shared_ptr<AStarNode>& a, const std::shared_ptr<AStarNode>& b) const {
    return a->f_cost > b->f_cost;
  }
};

class CustomPlanner : public nav2_core::GlobalPlanner {
public:
  CustomPlanner() = default;
  ~CustomPlanner() = default;
  void configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
      std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  nav_msgs::msg::Path
  createPlan(const geometry_msgs::msg::PoseStamped &start,
             const geometry_msgs::msg::PoseStamped &goal) override;

private:
  std::shared_ptr<tf2_ros::Buffer> tf_;
  nav2_util::LifecycleNode::SharedPtr node_;
  nav2_costmap_2d::Costmap2D *costmap_;
  std::string global_frame_, name_;
  double interpolation_resolution_{0.1};
  double node_cost_upper_bound_{252.0};
  double sight_cost_upper_bound_{180.0};
  double cost_penalty_gain_{100.0};
  double turn_penalty_{3.0};

  // Compute the Euclidean heuristic used by A*.
  double getHeuristic(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2);
  // Reject cells that are outside the map, unknown, or effectively lethal.
  bool isNodeValid(unsigned int x, unsigned int y);
  // Reject cells that are too expensive for line-of-sight smoothing.
  bool isSightValid(unsigned int x, unsigned int y);
  // Convert a costmap cell cost into a traversability penalty for A* expansion.
  double getTraversalPenalty(unsigned int x, unsigned int y) const;
  // Return an extra penalty when the expansion direction changes.
  double getTurnPenalty(
    const std::shared_ptr<AStarNode> & current_node,
    unsigned int next_x, unsigned int next_y) const;
  // Provide a hex-like neighborhood to reduce Manhattan-style path artifacts.
  std::vector<std::pair<int, int>> getHexNeighbors(unsigned int y);
  // Check whether two cells can be connected directly during path smoothing.
  bool hasLineOfSight(unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1);
};

} // namespace nav2_custom_planner

#endif // NAV2_CUSTOM_PLANNER__NAV2_CUSTOM_PLANNER_HPP_
