#pragma once

#include <rclcpp/rclcpp.hpp>
#include <keyboard_msgs/msg/key.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace robmovil
{

class KeysToTwist : public rclcpp::Node
{
  public:

    KeysToTwist();

    void on_key_up(const keyboard_msgs::msg::Key::SharedPtr key_event);
    void on_key_down(const keyboard_msgs::msg::Key::SharedPtr key_event);

  private:

    rclcpp::Subscription<keyboard_msgs::msg::Key>::SharedPtr key_up_sub_, key_down_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;

  // Acá pueden agregar las variables de instancia que necesiten
  // ...

    bool up_, down_, left_, right_;
};
}