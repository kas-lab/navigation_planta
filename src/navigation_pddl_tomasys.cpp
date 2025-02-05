#include <functional>

#include <navigation_pddl_tomasys/navigation_pddl_tomasys.hpp>


namespace navigation_pddl_tomasys {


/**
 * @brief Constructor
 *
 * @param options node options
 */
NavigationPddlTomasys::NavigationPddlTomasys() : Node("navigation_pddl_tomasys") {

  this->setup();
}


/**
 * @brief Sets up subscribers, publishers, etc. to configure the node
 */
void NavigationPddlTomasys::setup() {
}


}


int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<navigation_pddl_tomasys::NavigationPddlTomasys>());
  rclcpp::shutdown();

  return 0;
}
