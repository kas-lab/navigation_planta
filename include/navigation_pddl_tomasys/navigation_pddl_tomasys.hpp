#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>


namespace navigation_pddl_tomasys {



class NavigationPddlTomasys : public rclcpp::Node {

 public:

  NavigationPddlTomasys();

 private:

  void setup();

 private:
};


}
