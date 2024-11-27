/**
 * @file help_marker.cpp
 * @author Pandu Surya Tantra (pandustantra@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-04-02
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "ros2_utils/help_marker.hpp"

HelpMarker::HelpMarker() {}

void HelpMarker::cllbck_tim_10hz()
{
    if (_msg_marker_array.markers.empty())
    {
        return;
    }

    _pub_marker_array->publish(_msg_marker_array);
    _msg_marker_array.markers.clear();
}

bool HelpMarker::is_initialized()
{
    return _is_initialized;
}

bool HelpMarker::init(rclcpp::Node::SharedPtr node)
{
    if (_is_initialized)
    {
        return true;
    }

    if (!_logger.init())
    {
        return false;
    }

    _node = node;
    _tim_10hz = _node->create_wall_timer(std::chrono::milliseconds(100), std::bind(&HelpMarker::cllbck_tim_10hz, this));
    _pub_marker_array = _node->create_publisher<visualization_msgs::msg::MarkerArray>("marker_array", 10);

    _is_initialized = true;
    return true;
}

std_msgs::msg::ColorRGBA
HelpMarker::to_color(std::vector<float> input)
{
    if (input.size() != 4)
    {
        return std_msgs::msg::ColorRGBA();
    }

    std_msgs::msg::ColorRGBA output;
    output.r = input[0];
    output.g = input[1];
    output.b = input[2];
    output.a = input[3];
    return output;
}

void HelpMarker::arrow(std::string frame_id, std::string ns, int id, std::vector<geometry_msgs::msg::Point> pts,
                       std::vector<float> color, float s_shaft, float s_head)
{
    if (!_is_initialized || color.size() != 4)
    {
        return;
    }

    visualization_msgs::msg::Marker marker;
    //----
    marker.header.frame_id = frame_id;
    marker.header.stamp = _node->now();
    marker.ns = ns;
    marker.id = abs(id);
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = pts.size() < 2 ? visualization_msgs::msg::Marker::DELETE : visualization_msgs::msg::Marker::ADD;
    //----
    marker.pose.orientation.w = 1;
    marker.scale.x = s_shaft;
    marker.scale.y = s_head;
    marker.color = to_color(color);
    marker.frame_locked = true;
    marker.points = pts;
    //----
    _msg_marker_array.markers.push_back(marker);
}

void HelpMarker::cube(std::string frame_id, std::string ns, int id, geometry_msgs::msg::Point pt,
                      geometry_msgs::msg::Quaternion quat, std::vector<float> color, float s_x, float s_y, float s_z)
{
    if (!_is_initialized || id == 0 || color.size() != 4)
    {
        return;
    }

    visualization_msgs::msg::Marker marker;
    //----
    marker.header.frame_id = frame_id;
    marker.header.stamp = _node->now();
    marker.ns = ns;
    marker.id = abs(id);
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = id > 0 ? visualization_msgs::msg::Marker::ADD : visualization_msgs::msg::Marker::DELETE;
    //----
    marker.pose.position = pt;
    marker.pose.orientation = quat;
    marker.scale.x = s_x;
    marker.scale.y = s_y;
    marker.scale.z = s_z;
    marker.color = to_color(color);
    marker.frame_locked = true;
    //----
    _msg_marker_array.markers.push_back(marker);
}

void HelpMarker::sphere(std::string frame_id, std::string ns, int id, geometry_msgs::msg::Point pt,
                        geometry_msgs::msg::Quaternion quat, std::vector<float> color, float s_x, float s_y, float s_z)
{
    if (!_is_initialized || id == 0 || color.size() != 4)
    {
        return;
    }

    visualization_msgs::msg::Marker marker;
    //----
    marker.header.frame_id = frame_id;
    marker.header.stamp = _node->now();
    marker.ns = ns;
    marker.id = abs(id);
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = id > 0 ? visualization_msgs::msg::Marker::ADD : visualization_msgs::msg::Marker::DELETE;
    //----
    marker.pose.position = pt;
    marker.pose.orientation = quat;
    marker.scale.x = s_x;
    marker.scale.y = s_y;
    marker.scale.z = s_z;
    marker.color = to_color(color);
    marker.frame_locked = true;
    //----
    _msg_marker_array.markers.push_back(marker);
}

void HelpMarker::cylinder(std::string frame_id, std::string ns, int id, geometry_msgs::msg::Point pt,
                          geometry_msgs::msg::Quaternion quat, std::vector<float> color, float s_x, float s_y, float s_z)
{
    if (!_is_initialized || id == 0 || color.size() != 4)
    {
        return;
    }

    visualization_msgs::msg::Marker marker;
    //----
    marker.header.frame_id = frame_id;
    marker.header.stamp = _node->now();
    marker.ns = ns;
    marker.id = abs(id);
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = id > 0 ? visualization_msgs::msg::Marker::ADD : visualization_msgs::msg::Marker::DELETE;
    //----
    marker.pose.position = pt;
    marker.pose.orientation = quat;
    marker.scale.x = s_x;
    marker.scale.y = s_y;
    marker.scale.z = s_z;
    marker.color = to_color(color);
    marker.frame_locked = true;
    //----
    _msg_marker_array.markers.push_back(marker);
}

void HelpMarker::line_strip(std::string frame_id, std::string ns, int id, std::vector<geometry_msgs::msg::Point> pts,
                            std::vector<float> color, float s)
{
    if (!_is_initialized || color.size() != 4)
    {
        return;
    }

    visualization_msgs::msg::Marker marker;
    //----
    marker.header.frame_id = frame_id;
    marker.header.stamp = _node->now();
    marker.ns = ns;
    marker.id = abs(id);
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = pts.size() < 2 ? visualization_msgs::msg::Marker::DELETE : visualization_msgs::msg::Marker::ADD;
    //----
    marker.pose.orientation.w = 1;
    marker.scale.x = s;
    marker.color = to_color(color);
    marker.frame_locked = true;
    marker.points = pts;
    //----
    _msg_marker_array.markers.push_back(marker);
}

void HelpMarker::line_list(std::string frame_id, std::string ns, int id, std::vector<geometry_msgs::msg::Point> pts,
                           std::vector<float> color, float s)
{
    if (!_is_initialized || color.size() != 4)
    {
        return;
    }

    visualization_msgs::msg::Marker marker;
    //----
    marker.header.frame_id = frame_id;
    marker.header.stamp = _node->now();
    marker.ns = ns;
    marker.id = abs(id);
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = pts.size() % 2 ? visualization_msgs::msg::Marker::DELETE : visualization_msgs::msg::Marker::ADD;
    //----
    marker.pose.orientation.w = 1;
    marker.scale.x = s;
    marker.color = to_color(color);
    marker.frame_locked = true;
    marker.points = pts;
    //----
    _msg_marker_array.markers.push_back(marker);
}

void HelpMarker::cube_list(std::string frame_id, std::string ns, int id, std::vector<geometry_msgs::msg::Point> pts,
                           geometry_msgs::msg::Quaternion quat, std::vector<float> color, float s_x, float s_y, float s_z)
{
    if (!_is_initialized || color.size() != 4)
    {
        return;
    }

    visualization_msgs::msg::Marker marker;
    //----
    marker.header.frame_id = frame_id;
    marker.header.stamp = _node->now();
    marker.ns = ns;
    marker.id = abs(id);
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = !pts.empty() ? visualization_msgs::msg::Marker::ADD : visualization_msgs::msg::Marker::DELETE;
    //----
    marker.pose.orientation = quat;
    marker.scale.x = s_x;
    marker.scale.y = s_y;
    marker.scale.z = s_z;
    marker.color = to_color(color);
    marker.frame_locked = true;
    marker.points = pts;
    //----
    _msg_marker_array.markers.push_back(marker);
}

void HelpMarker::sphere_list(std::string frame_id, std::string ns, int id, std::vector<geometry_msgs::msg::Point> pts,
                             geometry_msgs::msg::Quaternion quat, std::vector<float> color, float s_x, float s_y,
                             float s_z)
{
    if (!_is_initialized || color.size() != 4)
    {
        return;
    }

    visualization_msgs::msg::Marker marker;
    //----
    marker.header.frame_id = frame_id;
    marker.header.stamp = _node->now();
    marker.ns = ns;
    marker.id = abs(id);
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.action = !pts.empty() ? visualization_msgs::msg::Marker::ADD : visualization_msgs::msg::Marker::DELETE;
    //----
    marker.pose.orientation = quat;
    marker.scale.x = s_x;
    marker.scale.y = s_y;
    marker.scale.z = s_z;
    marker.color = to_color(color);
    marker.frame_locked = true;
    marker.points = pts;
    //----
    _msg_marker_array.markers.push_back(marker);
}

void HelpMarker::points(std::string frame_id, std::string ns, int id, std::vector<geometry_msgs::msg::Point> pts,
                        std::vector<float> color, float s_w, float s_h)
{
    if (!_is_initialized || color.size() != 4)
    {
        return;
    }

    visualization_msgs::msg::Marker marker;
    //----
    marker.header.frame_id = frame_id;
    marker.header.stamp = _node->now();
    marker.ns = ns;
    marker.id = abs(id);
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = !pts.empty() ? visualization_msgs::msg::Marker::ADD : visualization_msgs::msg::Marker::DELETE;
    //----
    marker.pose.orientation.w = 1;
    marker.scale.x = s_w;
    marker.scale.y = s_h;
    marker.color = to_color(color);
    marker.frame_locked = true;
    marker.points = pts;
    //----
    _msg_marker_array.markers.push_back(marker);
}

void HelpMarker::text_view_facing(std::string frame_id, std::string ns, int id, geometry_msgs::msg::Point pt,
                                  std::vector<float> color, std::string text, float s)
{
    if (!_is_initialized || id == 0 || color.size() != 4)
    {
        return;
    }

    visualization_msgs::msg::Marker marker;
    //----
    marker.header.frame_id = frame_id;
    marker.header.stamp = _node->now();
    marker.ns = ns;
    marker.id = abs(id);
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = id > 0 ? visualization_msgs::msg::Marker::ADD : visualization_msgs::msg::Marker::DELETE;
    //----
    marker.pose.position = pt;
    marker.scale.z = s;
    marker.color = to_color(color);
    marker.text = text;
    //----
    _msg_marker_array.markers.push_back(marker);
}