#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancygrid.hpp>

rclcpp::Node::SharedPtr nodeh;

void callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
	RCLCPP_INFO(nodeh->get_logger(), "size: %d", msg->data.size());
}

int main(int argc, char **argv) {

	rclcpp::init(argc,argv);
	nodeh = rclcpp::Node::make_shared("test");
	auto sub = nodeh->create_subscription<nav_msgs::msg::OccupancyGrid>
		("/map", 1000, &callback);
		
	rclcpp::spin(nodeh);
	rclcpp::shutdown();

}
