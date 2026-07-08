#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <vector>

namespace onboarding_project
{

class CustomMazePublisher : public rclcpp::Node
{
public:
  CustomMazePublisher() : Node("custom_maze_publisher")
  {
    this->declare_parameter("hallway_width", 10.0);
    this->declare_parameter("hallway_height", 10.0);
    this->declare_parameter("wall_width", 2.0);

    maze_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "rviz/custom_maze", rclcpp::QoS(10).transient_local());

    publish_maze();
  }

private:
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr maze_pub_;

  visualization_msgs::msg::Marker create_wall(int id, double cx, double cy, double sx, double sy)
  {
    double hallway_height = this->get_parameter("hallway_height").as_double();
    
    visualization_msgs::msg::Marker wall;
    wall.header.frame_id = "world";
    wall.header.stamp = this->now();
    wall.ns = "custom_walls";
    wall.id = id;
    wall.type = visualization_msgs::msg::Marker::CUBE;
    wall.action = visualization_msgs::msg::Marker::ADD;
    
    wall.pose.position.x = cx;
    wall.pose.position.y = cy;
    wall.pose.position.z = hallway_height / 2.0;
    wall.pose.orientation.w = 1.0;

    wall.scale.x = sx;
    wall.scale.y = sy;
    wall.scale.z = hallway_height;

    wall.color.r = 0.0f; wall.color.g = 1.0f; wall.color.b = 0.0f; wall.color.a = 0.4f;
    return wall;
  }

  void publish_maze()
  {
    visualization_msgs::msg::MarkerArray marker_array;
    double hw = this->get_parameter("hallway_width").as_double();
    double ww = this->get_parameter("wall_width").as_double();
    int id = 0;

    double offset = (hw + ww) / 2.0; 

    // 1. Entryway Runway (West to East)
    double r_start = -3.0;
    double r_end = 35.0 - offset;
    double r_cx = (r_start + r_end) / 2.0;
    marker_array.markers.push_back(create_wall(id++, r_cx, 0.0 + offset, r_end - r_start, ww));
    marker_array.markers.push_back(create_wall(id++, r_cx, 0.0 - offset, r_end - r_start, ww));
    marker_array.markers.push_back(create_wall(id++, r_start - (ww/2.0), 0.0, ww, hw + (ww * 2.0)));

    // 2. Spine Vertical Crossbar (North to South)
    double spine_x = 35.0;
    double h_len = 40.0 - offset;
    marker_array.markers.push_back(create_wall(id++, spine_x - offset, 40.0 - (h_len/2.0), ww, h_len));
    marker_array.markers.push_back(create_wall(id++, spine_x - offset, -40.0 + (h_len/2.0), ww, h_len));
    marker_array.markers.push_back(create_wall(id++, spine_x + offset, 0.0, ww, 80.0)); 
    marker_array.markers.push_back(create_wall(id++, spine_x, 40.0 + (ww/2.0), hw + (ww * 2.0), ww));

    // 3. Goal Corridor (Eastbound off the bottom Spine)
    double g_start = 35.0 + offset;
    double g_end = 80.0;
    double g_cx = (g_start + g_end) / 2.0;
    marker_array.markers.push_back(create_wall(id++, g_cx, -40.0 + offset, g_end - g_start, ww));
    marker_array.markers.push_back(create_wall(id++, g_cx, -40.0 - offset, g_end - g_start, ww));
    marker_array.markers.push_back(create_wall(id++, g_end + (ww/2.0), -40.0, ww, hw + (ww * 2.0)));

    // 4. Goal Model Marker
    visualization_msgs::msg::Marker model;
    model.header.frame_id = "world";
    model.ns = "goal_statue";
    model.id = id++;
    model.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    model.mesh_resource = "package://onboarding_project/resource/maeserstatue_small.stl";
    model.action = visualization_msgs::msg::Marker::ADD;
    model.pose.position.x = g_end - 3.0;
    model.pose.position.y = -40.0;
    model.pose.position.z = 2.5;
    model.pose.orientation.w = 1.0;
    model.scale.x = 0.15; model.scale.y = 0.15; model.scale.z = 0.15;
    model.color.r = 0.7f; model.color.g = 0.7f; model.color.b = 0.7f; model.color.a = 1.0f;
    marker_array.markers.push_back(model);

    maze_pub_->publish(marker_array);
  }
};

} // namespace onboarding_project

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<onboarding_project::CustomMazePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
