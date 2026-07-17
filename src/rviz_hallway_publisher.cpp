#include "rviz_hallway_publisher.hpp"

namespace ai_agent_dbt {

RvizPublisher::RvizPublisher()
  : Node("rviz_hallway_publisher")
  , qos_transient_local_20_(20)
{
  declare_parameters();
  parameter_callback_handle_ = this->add_on_set_parameters_callback(std::bind(
      &RvizPublisher::parameters_callback, this, std::placeholders::_1));

  qos_transient_local_20_.transient_local();
  hallway_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "rviz/hallway", qos_transient_local_20_);

  // Pull the toggle state live
  bool load_arena = this->get_parameter("load_stadium_arena").as_bool();

  if (load_arena) {
    RCLCPP_INFO(this->get_logger(), "LOADING OBSTACLE & PATH PLANNING TESTING ARENA...");
    create_and_publish_stadium_arena();
  } else {
    RCLCPP_INFO(this->get_logger(), "LOADING SINGLE-AGENT MAZE CORRIDORS...");
    create_and_publish_hallway();
    publish_model();
  }
}

void RvizPublisher::declare_parameters() {
  // Arena Configuration Parameters
  this->declare_parameter("load_stadium_arena", true); // Set to false to return to original maze mode
  this->declare_parameter("arena_size", 120.0);
  this->declare_parameter("arena_height", 15.0);
  this->declare_parameter("arena_wall_width", 2.0);
  this->declare_parameter("arena_color", std::vector<double>{0.0, 0.5, 1.0, 0.3}); // Translucent Blue Stadium

  // Original Maze Parameters
  this->declare_parameter("cosmo_file", "resource/cosmo.dae");
  this->declare_parameter("hallway_color", std::vector<double>{0.0, 1.0, 0.0, 0.4});
  this->declare_parameter("hallway_width", 10.0);
  this->declare_parameter("hallway_height", 10.0);
  this->declare_parameter("wall_width", 2.0);
  this->declare_parameter("hallway_waypoints_x", std::vector<double>{0.0, 35.0, 35.0, 80.0});
  this->declare_parameter("hallway_waypoints_y", std::vector<double>{0.0, 0.0, 40.0, 40.0});
  this->declare_parameter("hallway_waypoints_z", std::vector<double>{0.0, 0.0, 0.0, 0.0});
  this->declare_parameter("model_file", "resource/maeserstatue_small.stl");
  this->declare_parameter("model_scale", 0.15);
  this->declare_parameter("model_z_offset", 2.5);
}

rcl_interfaces::msg::SetParametersResult RvizPublisher::parameters_callback(
    const std::vector<rclcpp::Parameter> &parameters) {
  (void)parameters;
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

void RvizPublisher::create_and_publish_stadium_arena() {
  double size = this->get_parameter("arena_size").as_double();
  double height = this->get_parameter("arena_height").as_double();
  double thickness = this->get_parameter("arena_wall_width").as_double();
  std::vector<double> color = this->get_parameter("arena_color").as_double_array();

  // Helper lambda for walls and obstacles (Cubes)
  auto make_cube = [&](int id, double cx, double cy, double cz, double sx, double sy, double sz, float r, float g, float b, float a) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = this->now();
    marker.ns = "hallway"; 
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = cx;
    marker.pose.position.y = cy;
    marker.pose.position.z = cz;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = sx;
    marker.scale.y = sy;
    marker.scale.z = sz;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
    return marker;
  };

  // Helper lambda for targets/waypoints (Spheres)
  auto make_sphere = [&](int id, double cx, double cy, double cz, double diameter, float r, float g, float b) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = this->now();
    marker.ns = "hallway";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = cx;
    marker.pose.position.y = cy;
    marker.pose.position.z = cz;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = diameter;
    marker.scale.y = diameter;
    marker.scale.z = diameter;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 0.8f; 
    return marker;
  };

  double half_size = size / 2.0;
  double half_height = height / 2.0;

  // 1. Outer Arena Bounds Enclosure (Translucent Blue)
  hallway_pub_->publish(make_cube(501, 0.0, half_size + (thickness/2.0), half_height, size + (thickness * 2.0), thickness, height, color[0], color[1], color[2], color[3])); // North Wall
  hallway_pub_->publish(make_cube(502, 0.0, -half_size - (thickness/2.0), half_height, size + (thickness * 2.0), thickness, height, color[0], color[1], color[2], color[3])); // South Wall
  hallway_pub_->publish(make_cube(503, half_size + (thickness/2.0), 0.0, half_height, thickness, size, height, color[0], color[1], color[2], color[3])); // East Wall
  hallway_pub_->publish(make_cube(504, -half_size - (thickness/2.0), 0.0, half_height, thickness, size, height, color[0], color[1], color[2], color[3])); // West Wall
  hallway_pub_->publish(make_cube(505, 0.0, 0.0, height, size, size, 0.5, color[0], color[1], color[2], 0.10f)); // Ceiling Cap

  // 2. STATIC OBSTACLES: 3 massive center pillar structures (Solid Dark Grey)
  // Each obstacle pillar is 15m x 15m wide and reaches full stadium height
  hallway_pub_->publish(make_cube(601, -30.0,  0.0, half_height, 15.0, 15.0, height, 0.2f, 0.2f, 0.2f, 1.0f)); // Left Pillar
  hallway_pub_->publish(make_cube(602,   0.0,  -30.0, half_height, 15.0, 15.0, height, 0.2f, 0.2f, 0.2f, 1.0f)); // Center Pillar
  hallway_pub_->publish(make_cube(603,  30.0,  0.0, half_height, 15.0, 15.0, height, 0.2f, 0.2f, 0.2f, 1.0f)); // Right Pillar

  // 3. CHECKPOINT GOALS: 3 bright path markers floating at standard drone altitude (Z = 5.0m)
  hallway_pub_->publish(make_sphere(701, -45.0, -35.0, 5.0, 4.0, 1.0f, 0.0f, 0.0f)); // Checkpoint A (Red)
  hallway_pub_->publish(make_sphere(702,  15.0,  35.0, 5.0, 4.0, 1.0f, 0.5f, 0.0f)); // Checkpoint B (Orange)
  hallway_pub_->publish(make_sphere(703,  45.0, -35.0, 5.0, 4.0, 0.0f, 1.0f, 0.0f)); // Checkpoint C (Green)
}

void RvizPublisher::create_and_publish_hallway() {
  double hw = this->get_parameter("hallway_width").as_double(); 
  double ww = this->get_parameter("wall_width").as_double();    
  double offset = (hw + ww) / 2.0; 

  auto make_custom_wall = [&](int wall_id, double cx, double cy, double sx, double sy) {
    visualization_msgs::msg::Marker hallway = create_default_hallway();
    hallway.id = wall_id;
    hallway.pose.position.x = cx;
    hallway.pose.position.y = cy;
    hallway.pose.position.z = 5.0; 
    hallway.scale.x = sx;
    hallway.scale.y = sy;
    return hallway;
  };

  double r_start = -3.0;
  double r_end = 35.0 - offset; 
  double r_cx = (r_start + r_end) / 2.0; 
  double r_len = r_end - r_start; 

  hallway_pub_->publish(make_custom_wall(100, r_cx, 0.0 + offset, r_len, ww)); 
  hallway_pub_->publish(make_custom_wall(101, r_cx, 0.0 - offset, r_len, ww)); 
  hallway_pub_->publish(make_custom_wall(102, r_start - (ww/2.0), 0.0, ww, hw + (ww * 2.0))); 

  double spine_x = 35.0;
  double top_w_len = (40.0 + offset) - offset; 
  double top_w_cy = offset + (top_w_len / 2.0);
  hallway_pub_->publish(make_custom_wall(200, spine_x - offset, top_w_cy, ww, top_w_len)); 

  double bot_w_len = -offset - (-40.0 - offset); 
  double bot_w_cy = -40.0 - offset + (bot_w_len / 2.0);
  hallway_pub_->publish(make_custom_wall(201, spine_x - offset, bot_w_cy, ww, bot_w_len)); 
  
  double east_spine_len = (40.0 - offset) - (-40.0 - offset); 
  double east_spine_cy = (-40.0 - offset) + (east_spine_len / 2.0); 
  hallway_pub_->publish(make_custom_wall(202, spine_x + offset, east_spine_cy, ww, east_spine_len)); 
  
  hallway_pub_->publish(make_custom_wall(203, spine_x, -40.0 - offset - (ww/2.0), hw + (ww * 2.0), ww));

  double g_start = spine_x + offset; 
  double g_end = 80.0;
  double g_len = g_end - g_start;
  double g_cx = (g_start + g_end) / 2.0;

  hallway_pub_->publish(make_custom_wall(300, g_cx, 40.0 - offset, g_len, ww)); 
  double g_north_start = spine_x - offset - (ww/2.0);
  double g_north_len = g_end - g_north_start;
  double g_north_cx = (g_north_start + g_end) / 2.0;
  hallway_pub_->publish(make_custom_wall(301, g_north_cx, 40.0 + offset, g_north_len, ww)); 
  hallway_pub_->publish(make_custom_wall(302, g_end + (ww/2.0), 40.0, ww, hw + (ww * 2.0))); 
}

visualization_msgs::msg::Marker RvizPublisher::create_default_hallway() {
  std::vector<double> color = this->get_parameter("hallway_color").as_double_array();
  double hallway_height = this->get_parameter("hallway_height").as_double();

  visualization_msgs::msg::Marker hallway;
  hallway.header.frame_id = "world";
  hallway.ns = "hallway";
  hallway.type = visualization_msgs::msg::Marker::CUBE;
  hallway.action = visualization_msgs::msg::Marker::ADD;
  hallway.pose.orientation.w = 1.0;
  hallway.color.r = static_cast<float>(color[0]);
  hallway.color.g = static_cast<float>(color[1]);
  hallway.color.b = static_cast<float>(color[2]);
  hallway.color.a = static_cast<float>(color[3]);
  hallway.scale.z = hallway_height;
  return hallway;
}

void RvizPublisher::add_back_wall() {}

Eigen::Vector2d RvizPublisher::move_second_up_or_down(Eigen::Vector2d line_dir, Eigen::Vector2d normal, double width) {
  return line_dir.dot(normal) * line_dir * width;
}

void RvizPublisher::publish_model() {
  visualization_msgs::msg::Marker model;
  model.header.frame_id = "world";
  model.ns = "stl";
  model.id = 99; 
  model.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
  model.mesh_resource = "package://ai_agent_dbt/" + this->get_parameter("model_file").as_string();
  model.action = visualization_msgs::msg::Marker::ADD;
  model.pose.position.x = 77.0; 
  model.pose.position.y = 40.0; 
  model.pose.position.z = this->get_parameter("model_z_offset").as_double();
  model.pose.orientation.w = 1.0;
  model.scale.x = this->get_parameter("model_scale").as_double();
  model.scale.y = this->get_parameter("model_scale").as_double();
  model.scale.z = this->get_parameter("model_scale").as_double();
  model.color.r = 0.67f; model.color.g = 0.67f; model.color.b = 0.67f; model.color.a = 1.0;
  hallway_pub_->publish(model);
}

} // namespace ai_agent_dbt

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ai_agent_dbt::RvizPublisher>();
  rclcpp::spin(node);
  return 0;
}
