#include <tsl/Camera.hpp>
#include <tsl/Application.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <iostream>

using glm::lookAt;
using glm::radians;
using glm::normalize;
using glm::pi;
using glm::cross;
using glm::rotateX;
using glm::rotateY;
using glm::dot;
using glm::abs;

using std::cout;
using std::endl;
using std::nullopt;

namespace tsl {

mat4 Camera::get_view_matrix() const {
    return lookAt(pos, pos + direction, up);
}

void Camera::cursor_pos_changed(double x_pos, double y_pos) {
    auto time = Application::getInstance().get_time();

    // if mouse pos is not initialised, dont do anything except initialising it
    if (!last_cursor_pos) {
        last_cursor_pos = MousePos(x_pos, y_pos);
        last_mouse_time = time;
        return;
    }

    // calc mouse movement
    double time_delta = time - last_mouse_time;
    double x_pos_delta = x_pos - (*last_cursor_pos).x;
    double y_pos_delta = y_pos - (*last_cursor_pos).y;
    auto x_offset = x_pos_delta * MOUSE_SENSITIVITY * time_delta;
    auto y_offset = y_pos_delta * MOUSE_SENSITIVITY * time_delta;
    last_mouse_time = time;
    (*last_cursor_pos).x = x_pos;
    (*last_cursor_pos).y = y_pos;

    // constraint view to not turn upside down
    auto new_direction = rotateY(rotateX(direction, static_cast<float>(y_offset)), static_cast<float>(x_offset));
    if (abs(dot(new_direction, up)) < 0.9) {
        direction = new_direction;
    }
}

void Camera::reset_curos_pos() {
    last_cursor_pos = nullopt;
}

void Camera::move_forward() {
    auto time = Application::getInstance().get_time();
    auto time_delta = static_cast<float>(time - last_move_time);
    last_move_time = time;

    pos += direction * time_delta * MOVE_SPEED;
}

void Camera::move_backwards() {
    auto time = Application::getInstance().get_time();
    auto time_delta = static_cast<float>(time - last_move_time);
    last_move_time = time;

    pos -= direction * time_delta * MOVE_SPEED;
}

void Camera::move_left() {
    auto time = Application::getInstance().get_time();
    auto time_delta = static_cast<float>(time - last_move_time);
    last_move_time = time;

    auto left = cross(up, direction);
    pos += left * time_delta * MOVE_SPEED;
}

void Camera::move_right() {
    auto time = Application::getInstance().get_time();
    auto time_delta = static_cast<float>(time - last_move_time);
    last_move_time = time;

    auto right = cross(direction, up);
    pos += right * time_delta * MOVE_SPEED;
}

void Camera::reset_move_time() {
    last_move_time = Application::getInstance().get_time();
}

}
