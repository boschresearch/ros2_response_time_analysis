// Copyright (c) 2019 Robert Bosch GmbH
// All rights reserved.
//
// This source code is licensed under the BSD-3-Clause license found in the
// LICENSE file in the root directory of this source tree.

#include <iostream>
#include <string>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include "model_validation/srv/int32.hpp"
#include "model_validation/srv/duration.hpp"

namespace model_validation {
using std::chrono::milliseconds;
class ArbitraryCBTimeNode : public rclcpp::Node
{
public:
  ArbitraryCBTimeNode()
    : Node("arbitrary_cb_time_node")
  {
    using namespace std::placeholders;
    RCLCPP_DEBUG(get_logger(), "ArbitraryCBTimeNode starting up");

    timerCtrl = this->create_subscription<msg::Duration>(std::string("~/timer"),
							 std::bind(&ArbitraryCBTimeNode::timerCreator,this,_1));
    const std::string names[] = { "high", "medium", "low" };
    for (const auto& name : names) {
      subs.push_back(this->create_subscription<std_msgs::msg::Int32>(std::string("~/")+name,
								     [this,name] (const typename std_msgs::msg::Int32::SharedPtr msg) {
								       this->simulateExecution("topic", name, milliseconds(msg->data));}));
      servs.push_back(this->create_service<srv::Int32>(std::string("~/")+"srv_"+name,
						       [this,name] (const std::shared_ptr<rmw_request_id_t> request_header,
								    const std::shared_ptr<srv::Int32::Request> request,
								    std::shared_ptr<srv::Int32::Response> response) {
							 (void) request_header; (void)response;
							 this->simulateExecution("service", std::string("srv_")+name, milliseconds(request->time));}));
      
    }
    execution_order_publisher = this->create_publisher<std_msgs::msg::String>("~/execution_order");
  }
  
  void timerCreator(const typename msg::Duration::SharedPtr msg) {
    milliseconds duration (msg->duration);
    uint32_t id = this->nextTimerId++;

    RCLCPP_INFO(get_logger(), "Setting up timer #%u in %u ms, running for %u ms", id, msg->offset, duration);
    /* we store the shared pointer on the heap, so we can explicitly release it in the timer callback.
       we cannot move the timer into the callback (or something like that), because we only get the timer
       from create_wall_timer, at which point the callbacks is already fixed */
    rclcpp::TimerBase::SharedPtr *timer = new rclcpp::TimerBase::SharedPtr;
    *timer = this->create_wall_timer(
				     milliseconds(msg->offset),
				     [this, duration, id , timer] () -> void {
				       this->simulateExecution("timer", std::to_string(id), duration);
				       delete timer;
				     });
  }

private:
  /* on topic update, block for N milliseconds */
  std::vector<rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr> subs;
  /* on service request, block for N milliseconds */
  std::vector<rclcpp::Service<srv::Int32>::SharedPtr> servs;
  /* Trigger a timer in N milliseconds, running for M milliseconds */
  rclcpp::Subscription<msg::Duration>::SharedPtr timerCtrl;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr execution_order_publisher;
  uint32_t nextTimerId;

  void simulateExecution(const std::string category, const std::string name, std::chrono::milliseconds time) {
    RCLCPP_INFO(get_logger(), "Received %s on %s for %d ms\n", category.c_str(), name.c_str(), (int32_t)time.count());
    std::this_thread::sleep_for (time);
    auto name_msg = std_msgs::msg::String();
    name_msg.data = name;
    this->execution_order_publisher->publish(name_msg);
    RCLCPP_INFO(get_logger(), "Finished %s on %s\n", category.c_str(), name.c_str());
  }
};
}
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<model_validation::ArbitraryCBTimeNode>();

  // Creates a new executor and spins the node
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
