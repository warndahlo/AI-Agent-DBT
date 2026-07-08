#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <vector>

namespace onboarding_project {

class RvizArenaPublisher : public rclcpp::Node {
public:
  RvizArenaPublisher() : Node("rviz_arena_publisher") {
    // Declare arena parameters (Defaulting to a robust 120m x 120m x 15m stadium)
    this->declare_parameter("arena_size", 120.0);
    this->declare_parameter("arena_height", 15.0);
    this->declare_parameter("wall_thickness", 2.0);
    this->declare_parameter("stadium_color", std::vector<double>{0.0, 0.5, 1.0, 0.3}); // Transparent Blue

    // Transient Local QoS ensures RViz receives the static stadium instantly upon opening
    auto qos = rclcpp::QoS(20).transient_local();
    arena_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("rviz/arena", qos);

    // Build and broadcast the stadium perimeter
    publish_arena_bounds();
  }

private:
  void publish_arena_bounds() {
    double size = this->get_parameter("arena_size").as_double();
    double height = this->get_parameter("arena_height").as_double();
    double thickness = this->get_parameter("wall_thickness").as_double();
    std::vector<double> color = this->get_parameter("stadium_color").as_double_array();

    auto make_wall = [&](int id, double cx, double cy, double cz, double sx, double sy, double sz) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "world";
      marker.header.stamp = this->now();
      marker.ns = "stadium";
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

      marker.color.r = static_cast<float>(color[0]);
      marker.color.g = static_cast<float>(color[1]);
      marker.color.b = static_cast<float>(color[2]);
      marker.color.a = static_cast<float>(color[3]);
      return marker;
    };

    double half_size = size / 2.0;
    double half_height = height / 2.0;

    // 1. North Wall (Y = +half_size)
    arena_pub_->publish(make_wall(1, 0.0, half_size + (thickness/2.0), half_height, size + (thickness * 2.0), thickness, height));

    // 2. South Wall (Y = -half_size)
    arena_pub_->publish(make_wall(2, 0.0, -half_size - (thickness/2.0), half_height, size + (thickness * 2.0), thickness, height));

    // 3. East Wall (X = +half_size)
    arena_pub_->publish(make_wall(3, half_size + (thickness/2.0), 0.0, half_height, thickness, size, height));

    // 4. West Wall (X = -half_size)
    arena_pub_->publish(make_wall(4, -half_size - (thickness/2.0), 0.0, half_height, thickness, size, height));

    // 5. Roof / Ceiling Cap (Suspended at Z = height)
    // Made slightly thinner (0.5m) and more transparent so you can still look down into the match in RViz
    visualization_msgs::msg::Marker roof = make_wall(5, 0.0, 0.0, height, size, size, 0.5);
    roof.color.a = 0.15f; 
    arena_pub_->publish(roof);
  }

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr arena_pub_;
};

} // namespace onboarding_project

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<onboarding_project::RvizArenaPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
