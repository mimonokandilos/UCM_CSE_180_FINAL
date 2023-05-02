//START added section

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/occupancy_grid_info.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.h"




//DONE added section

//OG
//#include <rclcpp/rclcpp.hpp> 
#include <navigation/navigation.hpp>
#include <iostream>
//OG


//START added section
void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  // Convert the scan to a point cloud
  sensor_msgs::msg::PointCloud2 scan_cloud;
  tf2_sensor_msgs::convert(*msg, scan_cloud, "laser_link");

  // Check each point in the scan
  for (int i = 0; i < msg->ranges.size(); i++) {
    float range = msg->ranges[i];
    if (std::isfinite(range)) {
      // Get the position of this point in the laser frame
      float angle = msg->angle_min + i * msg->angle_increment;
      float x = range * std::cos(angle);
      float y = range * std::sin(angle);
      float z = 0.0;
      tf2::Vector3 pos(x, y, z);

      // Check whether this point is within the map
      bool inside_map = false;
      for (int j = 0; j < point_cloud.height * point_cloud.width; j++) {
        float *data = reinterpret_cast<float*>(&point_cloud.data[j * point_cloud.point_step]);
        tf2::Vector3 map_pos(data[0], data[1], data[2]);
        float dist_sq = (map_pos - pos).length2();
        if (dist_sq < 0.01) { // 1cm tolerance
          inside_map = true;
          break;
        }
      }

      // If this point is not within the map, print a warning
      if (!inside_map) {
        RCLCPP_WARN(node->get_logger(), "Scan point %d is outside of the map", i);
      }
    }
  }
}





int main(int argc, char **argv)
{


  rclcpp::init(argc,argv); // initialize ROS 
  Navigator navigator(true,false); // create node with debug info but not verbose
  
  auto node = rclcpp::Node::make_shared("scan_listener");
  auto sub = node->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, scanCallback);

  rclcpp::spin(node);
  nav_msgs::msg::OccupancyGrid map;
  sensor_msgs::msg::PointCloud2 point_cloud;

  // first: it is mandatory to initialize the pose of the robot
  geometry_msgs::msg::Pose::SharedPtr init = std::make_shared<geometry_msgs::msg::Pose>();
  init->position.x = -2;
  init->position.y = -0.5;
  init->orientation.w = 1;
  navigator.SetInitialPose(init);
  // wait for navigation stack to become operationale
  navigator.WaitUntilNav2Active();
  // spin in place of 90 degrees (default parameter)
  navigator.Spin();

  while ( ! navigator.IsTaskComplete() ) {
    // busy waiting for task to be completed
  }
  geometry_msgs::msg::Pose::SharedPtr goal_pos = std::make_shared<geometry_msgs::msg::Pose>();
  
  float arr_loc_x[] = {-1, 0, 1, 1.5, 1.5, 1.5, 1, 0, -1, -2};
  float arr_loc_y[] = {-2, -2, -2, -1, 0, 1, 1.75, 2, 2, 0.5};
  
  // var for the location array indecies
  int arr_index = 0;
  
  // Loop until we either visit all locations or find the goal
  while (arr_index != 10)
  {
  	goal_pos->position.x = arr_loc_x[arr_index];
  	goal_pos->position.y = arr_loc_y[arr_index];
  	goal_pos->orientation.w = 1;
  	// move to new pose
  	navigator.GoToPose(goal_pos);
  	while ( ! navigator.IsTaskComplete() ) {
  	  	
  	}
  	
  	// Increment arr_index
  	arr_index++;
   }
   
  float arr_loc_x_two[] = {-0.5, 0.5, 0.5, -0.5};
  float arr_loc_y_two[] = {0.5, 0.5, -0.5, -0.5};
  
  // var for the location array indecies
  arr_index = 0;
  
  // Loop until we either visit all locations or find the goal
  while (arr_index != 4)
  {
  	goal_pos->position.x = arr_loc_x_two[arr_index];
  	goal_pos->position.y = arr_loc_y_two[arr_index];
  	goal_pos->orientation.w = 1;
  	// move to new pose
  	navigator.GoToPose(goal_pos);
  	while ( ! navigator.IsTaskComplete() ) {
  	  	
  	}
  	
  	// Increment arr_index
  	arr_index++;
   }
  // complete here....

      // Get the map data
      auto map_sub = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10, [&](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
          map = *msg;
      });

      // Convert the map to a point cloud
      auto transform_listener = std::make_shared<tf2_ros::TransformListener>(node);
      tf2::Transform map_to_laser_transform;
      try {
        auto transform = transform_listener->lookupTransform(
          "laser_link", "map", rclcpp::Time(0));
        tf2::fromMsg(transform.transform, map_to_laser_transform);
      } catch (const tf2::TransformException &ex) {
        RCLCPP_ERROR(node->get_logger(), "%s", ex.what());
      }
      auto info = map.info;
      point_cloud.header.frame_id = "laser_link";
      point_cloud.height = info.height;
      point_cloud.width = info.width;
      point_cloud.fields.resize(3);
      point_cloud.fields[0].name = "x";
      point_cloud.fields[0].offset = 0;
      point_cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
      point_cloud.fields[0].count = 1;
      point_cloud.fields[1].name = "y";
      point_cloud.fields[1].offset = 4;
      point_cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
      point_cloud.fields[1].count = 1;
      point_cloud.fields[2].name = "z";
      point_cloud.fields[2].offset = 8;
      point_cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
      point_cloud.fields[2].count = 1;
      point_cloud.point_step = 12;
      point_cloud.row_step = info.width * point_cloud.point_step;
      point_cloud.is_dense = false;
      point_cloud.data.resize(point_cloud.row_step * point_cloud.height);
      for (int y = 0; y < info.height; y++) {
        for (int x = 0; x < info.width; x++) {
          float occ = map

        // Add this point to the point cloud
        int index = x + y * info.width;
        float *data = reinterpret_cast<float*>(&point_cloud.data[index * point_cloud.point_step]);
        data[0] = pos.x();
        data[1] = pos.y();
        data[2] = pos.z();
      }
    }
  rclcpp::shutdown();
  return 0;
}


//DONE added section









// #include <rclcpp/rclcpp.hpp> 
// #include <navigation/navigation.hpp>
// #include <iostream>



// int main(int argc,char **argv) {
 
//   rclcpp::init(argc,argv); // initialize ROS 
//   Navigator navigator(true,false); // create node with debug info but not verbose

//   // first: it is mandatory to initialize the pose of the robot
//   geometry_msgs::msg::Pose::SharedPtr init = std::make_shared<geometry_msgs::msg::Pose>();
//   init->position.x = -2;
//   init->position.y = -0.5;
//   init->orientation.w = 1;
//   navigator.SetInitialPose(init);
//   // wait for navigation stack to become operationale
//   navigator.WaitUntilNav2Active();
//   // spin in place of 90 degrees (default parameter)
//   navigator.Spin();
//   while ( ! navigator.IsTaskComplete() ) {
//     // busy waiting for task to be completed
//   }
//   geometry_msgs::msg::Pose::SharedPtr goal_pos = std::make_shared<geometry_msgs::msg::Pose>();
  
//   float arr_loc_x[] = {-1, 0, 1, 1.5, 1.5, 1.5, 1, 0, -1, -2};
//   float arr_loc_y[] = {-2, -2, -2, -1, 0, 1, 1.75, 2, 2, 0.5};
  
//   // var for the location array indecies
//   int arr_index = 0;
  
//   // Loop until we either visit all locations or find the goal
//   while (arr_index != 10)
//   {
//   	goal_pos->position.x = arr_loc_x[arr_index];
//   	goal_pos->position.y = arr_loc_y[arr_index];
//   	goal_pos->orientation.w = 1;
//   	// move to new pose
//   	navigator.GoToPose(goal_pos);
//   	while ( ! navigator.IsTaskComplete() ) {
  	  	
//   	}
  	
//   	// Increment arr_index
//   	arr_index++;
//    }
   
//   float arr_loc_x_two[] = {-0.5, 0.5, 0.5, -0.5};
//   float arr_loc_y_two[] = {0.5, 0.5, -0.5, -0.5};
  
//   // var for the location array indecies
//   arr_index = 0;
  
//   // Loop until we either visit all locations or find the goal
//   while (arr_index != 4)
//   {
//   	goal_pos->position.x = arr_loc_x_two[arr_index];
//   	goal_pos->position.y = arr_loc_y_two[arr_index];
//   	goal_pos->orientation.w = 1;
//   	// move to new pose
//   	navigator.GoToPose(goal_pos);
//   	while ( ! navigator.IsTaskComplete() ) {
  	  	
//   	}
  	
//   	// Increment arr_index
//   	arr_index++;
//    }
//   // complete here....
  
//   rclcpp::shutdown(); // shutdown ROS
//   return 0;
// }