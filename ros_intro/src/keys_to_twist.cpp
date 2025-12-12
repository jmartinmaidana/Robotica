#include "keys_to_twist.h"

using namespace robmovil;

KeysToTwist::KeysToTwist() : Node("nh")
{
  // nos susbribimos a los tópicos que reportan cuando se apreta o suelta una tecla.
  key_up_sub_ = this->create_subscription<keyboard_msgs::msg::Key>("/keyup", 10, std::bind(&KeysToTwist::on_key_up, this, std::placeholders::_1));
  key_down_sub_ = this->create_subscription<keyboard_msgs::msg::Key>("/keydown", 10, std::bind(&KeysToTwist::on_key_down, this, std::placeholders::_1));

  // anunciamos que vamos a publicar un mensaje de tipo geometry_msgs::msg::Twist en el tópico /cmd_vel.
  twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // ...

  up_ = down_ = left_ = right_ = false;
}

void KeysToTwist::on_key_up(const keyboard_msgs::msg::Key::SharedPtr key_event)
{
  geometry_msgs::msg::Twist twist;

  switch( key_event->code )
  {
    case keyboard_msgs::msg::Key::KEY_UP:
      // completar ...
      break;

    case keyboard_msgs::msg::Key::KEY_DOWN:
      // completar ...
      break;

    case keyboard_msgs::msg::Key::KEY_LEFT:
      // completar ...
      break;

    case keyboard_msgs::msg::Key::KEY_RIGHT:
      // completar ...
      break;

    default:
      return;
  }
  // completar ...

  twist_pub_->publish( twist );
}

void KeysToTwist::on_key_down(const keyboard_msgs::msg::Key::SharedPtr key_event)
{
  geometry_msgs::msg::Twist twist;

  switch( key_event->code )
  {
    case keyboard_msgs::msg::Key::KEY_UP:
      // completar ...
      break;

    case keyboard_msgs::msg::Key::KEY_DOWN:
      // completar ...
      break;

    case keyboard_msgs::msg::Key::KEY_LEFT:
      // completar ...
      break;

    case keyboard_msgs::msg::Key::KEY_RIGHT:
      // completar ...
      break;

    default:
      return;
  }

  // completar ...
  
  twist_pub_->publish( twist );
}
