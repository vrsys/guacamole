#include "Navigator.hpp"

#include <scm/gl_core/math.h>
#include <gua/math/math.hpp>
#include <chrono>
#include <iostream>

Navigator::Navigator()
    : transform_(scm::math::mat4f::identity()), current_location_(scm::math::vec4f(0.0)), current_rotation_(scm::math::vec2f(0.0)), mouse_position_(scm::math::vec2i(0)),
      mouse_movement_(scm::math::vec2i(0)), w_pressed_(false), s_pressed_(false), a_pressed_(false), d_pressed_(false), mlb_pressed_(false), frame_time_(-1.0), motion_speed_(0.02f)
{
}

void Navigator::update()
{
    if(frame_time_ == -1.0)
    {
        timer_.start();
        frame_time_ = 0.0;
    }
    else
    {
        frame_time_ = timer_.get_elapsed();

        const float rotation_speed = 0.2f;

        auto y_rot(scm::math::mat4f::identity());
        auto x_rot(scm::math::mat4f::identity());

        if(mlb_pressed_)
        {
            current_rotation_ -= mouse_movement_ * rotation_speed;
        }

        y_rot = scm::math::make_rotation(current_rotation_.x, 0.f, 1.f, 0.f);
        x_rot = scm::math::make_rotation(-current_rotation_.y, 1.f, 0.f, 0.f);

        auto rotation = y_rot * x_rot;

        float current_motion_speed = motion_speed_;

        if(shift_pressed_)
        {
            current_motion_speed *= 1.9f;
        }

        if(w_pressed_)
        {
            current_location_ += (rotation * scm::math::make_translation(0.f, 0.f, -current_motion_speed)) * scm::math::vec4f(0.f, 0.f, 0.f, 1.f);
        }

        if(s_pressed_)
        {
            current_location_ += (rotation * scm::math::make_translation(0.f, 0.f, current_motion_speed)) * scm::math::vec4f(0.f, 0.f, 0.f, 1.f);
        }

        if(a_pressed_)
        {
            current_location_ += (rotation * scm::math::make_translation(-current_motion_speed, 0.f, 0.f)) * scm::math::vec4f(0.f, 0.f, 0.f, 1.f);
        }

        if(d_pressed_)
        {
            current_location_ += (rotation * scm::math::make_translation(current_motion_speed, 0.f, 0.f)) * scm::math::vec4f(0.f, 0.f, 0.f, 1.f);
        }

        if(space_pressed_)
        {
            current_location_ += (rotation * scm::math::make_translation(0.f, current_motion_speed * 0.7f, 0.f)) * scm::math::vec4f(0.f, 0.f, 0.f, 1.f);
        }

        if(ctrl_pressed_)
        {
            current_location_ += (rotation * scm::math::make_translation(0.f, current_motion_speed * -0.7f, 0.f)) * scm::math::vec4f(0.f, 0.f, 0.f, 1.f);
        }

        auto target = scm::math::make_translation(current_location_.x, current_location_.y, current_location_.z) * rotation;
        float smoothness = frame_time_ * 10;
        transform_ = transform_ * (1.f - smoothness) + target * smoothness;
        mouse_movement_ = scm::math::vec2i(0);

        timer_.reset();
    }
}

void Navigator::set_transform(scm::math::mat4f const& transform)
{
    transform_ = transform;
    current_location_ = scm::math::vec4f(transform_[12], transform_[13], transform_[14], 1.0f);
}

void Navigator::reset()
{
    transform_ = scm::math::mat4f::identity();
    current_location_ = scm::math::vec4f(0.f, 0.f, 0.f, 1.0f);
    current_rotation_ = scm::math::vec2f(0.0);
}

scm::math::mat4f const& Navigator::get_transform() const { return transform_; }

void Navigator::set_key_press(gua::Key key, int action)
{
    switch(key)
    {
    case gua::Key::W:
        w_pressed_ = action != 0;
        break;

    case gua::Key::S:
        s_pressed_ = action != 0;
        break;

    case gua::Key::A:
        a_pressed_ = action != 0;
        break;

    case gua::Key::D:
        d_pressed_ = action != 0;
        break;

    case gua::Key::SPACE:
        space_pressed_ = action != 0;
        break;

    case gua::Key::LEFT_CONTROL:
        ctrl_pressed_ = action != 0;
        break;

    case gua::Key::LEFT_SHIFT:
        shift_pressed_ = action != 0;
        break;
    }
}

void Navigator::set_mouse_button(int button, int state)
{
    switch(button)
    {
    case 0:
        mlb_pressed_ = state == 1;
        break;
    }
}

void Navigator::set_mouse_position(scm::math::vec2i const& new_position)
{
    mouse_movement_ = new_position - mouse_position_;
    mouse_position_ = new_position;
}

void Navigator::set_motion_speed(float motion_speed) { motion_speed_ = motion_speed; }
