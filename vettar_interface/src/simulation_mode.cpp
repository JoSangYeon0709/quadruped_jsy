#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

using SetBool = std_srvs::srv::SetBool;
using std::placeholders::_1;
using std::placeholders::_2;

class SimulationModeServer : public rclcpp::Node
{
public:
  SimulationModeServer()
  : Node("simulation_mode_server")
  {
    // Declare parameter with default value
    this->declare_parameter("simulation_mode", false);

    // Get initial value from parameter
    simulation_enabled_ = this->get_parameter("simulation_mode").as_bool();

    // Create service
    service_ = create_service<SetBool>(
      "simulation_mode", 
      std::bind(&SimulationModeServer::handle_service, this, _1, _2));

    RCLCPP_INFO(this->get_logger(), 
                "Simulation mode is currently %s", 
                simulation_enabled_ ? "enabled" : "disabled");

    RCLCPP_INFO(this->get_logger(), 
                "Waiting for client request...");
  }

private:
  void handle_service(
    const std::shared_ptr<SetBool::Request> request,
    std::shared_ptr<SetBool::Response> response)
  {
    simulation_enabled_ = request->data;

    // Update the parameter as well
    this->set_parameter(rclcpp::Parameter("simulation_mode", simulation_enabled_));

    response->success = true;
    response->message = simulation_enabled_ ? 
      "Simulation mode has been enabled." : 
      "Simulation mode has been disabled.";

    RCLCPP_INFO(this->get_logger(), 
                "Received request: set simulation mode to %s", 
                simulation_enabled_ ? "enabled" : "disabled");

    RCLCPP_INFO(this->get_logger(), "Request handled. Server will shut down in 0.5 seconds.");

    // Shutdown after short delay to allow response to be sent
    oneshot_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500), 
      [this]() {
        rclcpp::shutdown();
      });
  }

  rclcpp::Service<SetBool>::SharedPtr service_;
  bool simulation_enabled_;
  rclcpp::TimerBase::SharedPtr oneshot_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimulationModeServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
