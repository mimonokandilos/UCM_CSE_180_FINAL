#include <rclcpp/rclcpp.hpp> 
#include <navigation/navigation.hpp>
#include <iostream>


int main(int argc,char **argv) {
  // initialize ROS  and create the node for the debug info
  rclcpp::init(argc,argv); 
  Navigator navigator(true,false);

  // first:  initialize pose of the robot
  geometry_msgs::msg::Pose::SharedPtr init = std::make_shared<geometry_msgs::msg::Pose>();
  init->position.x = -2;
  init->position.y = -0.5;
  init->orientation.w = 1;
  navigator.SetInitialPose(init);

  // wait for navigation stack to become operational
  navigator.WaitUntilNav2Active();

  // spin in place (default)90 degrees 
  navigator.Spin();

  while ( ! navigator.IsTaskComplete() ) {
    // wait for task to complete
  }

  geometry_msgs::msg::Pose::SharedPtr goal_pos = std::make_shared<geometry_msgs::msg::Pose>();

  float arr_loc_x[] = {-1, 0, 1, 1.5, 1.5, 1.5, 1, 0, -1, -2};
  float arr_loc_y[] = {-2, -2, -2, -1, 0, 1, 1.75, 2, 2, 0.5};

  // index of location values for x&y
  int arr_index = 0;

  // Loop until visit all locations or find the goal
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

  // index of location values for x&y
  arr_index = 0;

  // Loop until visit all locations or find the goal
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

  // shutdown ROS
  rclcpp::shutdown(); 
  return 0;
}

// SUBSCRIBING TO "MAP"

/*#include <rclcpp/rclcpp.hpp> 
#include <navigation/navigation.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

class mapStuff : public rclcpp::Node {
public:
	mapStuff():Node("mapstuff") {
		sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>
      ("/map",1000,std::bind(&mapStuff::callback,this,std::placeholders::_1));
    }
    
private:
	void callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
		RCLCPP_INFO(nodeh->get_logger(), "size: %d", msg->data.size());
	}
	
	rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub;
	rclcpp::Node::SharedPtr nodeh;
};


int main(int argc,char **argv) {

  rclcpp::init(argc,argv); // initialize ROS subsystem
  rclcpp::spin(std::make_shared<mapStuff>());  // create and spin
  rclcpp::shutdown();
  return 0;
  
}*/

// SUBSCRIBING TO "SCAN"


/*#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float32.hpp>
#include <algorithm>

class scanStuff : public rclcpp::Node {
public:
	scanStuff():Node("scanstuff") {
		sub = this->create_subscription<sensor_msgs::nsg::LaserScan>
      ("scan",1000,std::bind(&scanStuff::scanCallback,this,std::placeholders::_1));
	}
	
private:
	void scanCallback(const sensor_msgs::msg::LasesrScan::SharedPtr msg) {
		std::vector<float>::const_iterator minval = min(msg->ranges.begin(), msg->ranges.end());
		
		RCLCPP_INFO(nodeh->get_logger(), "size: %f", *minval);
	}
	
	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub;
	rclcpp::Node::SharedPtr nodeh;
	
};


int main(int argc, char **argv) {

  rclcpp::init(argc,argv); // initialize ROS subsystem
  rclcpp::spin(std::make_shared<scanStuff>());  // create and spin
  rclcpp::shutdown();
  return 0;
  
}*/

//DONE added section



// #include <rclcpp/rclcpp.hpp> 
// #include <navigation/navigation.hpp>
// #include <iostream>
// #include <nav_msgs/msg/occupancy_grid.hpp>

// rclcpp::Node::SharedPtr nodeh;



// void callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
// 	RCLCPP_INFO(nodeh->get_logger(), "size: %d", msg->data.size());
// }




// int main(int argc,char **argv) {
 
//   rclcpp::init(argc,argv); // initialize ROS 
//   Navigator navigator(true,false); // create node with debug info but not verbose
//   nodeh = rclcpp::Node::make_shared("navigate");
// 	auto sub = nodeh->create_subscription<nav_msgs::msg::OccupancyGrid>
// 		("/map", 1000, &callback);



    
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
//   rclcpp::spin(nodeh);
//   rclcpp::shutdown(); // shutdown ROS
//   return 0;
// }
