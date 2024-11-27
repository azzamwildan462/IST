/**
 * @file help_marker.hpp
 * @author Pandu Surya Tantra (pandustantra@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-04-02
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef HELP_MARKER_HPP_
#define HELP_MARKER_HPP_

#include "ros2_utils/help_logger.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

class HelpMarker
{
private:
    bool _is_initialized = false;

    rclcpp::Node::SharedPtr _node;
    rclcpp::TimerBase::SharedPtr _tim_10hz;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _pub_marker_array;
    HelpLogger _logger;

    visualization_msgs::msg::MarkerArray _msg_marker_array;

public:
    HelpMarker();

    void cllbck_tim_10hz();

    bool is_initialized();

    bool init(rclcpp::Node::SharedPtr node);

    std_msgs::msg::ColorRGBA to_color(std::vector<float> input);

    void arrow(std::string frame_id, std::string ns, int id, std::vector<geometry_msgs::msg::Point> pts,
               std::vector<float> color, float s_shaft, float s_head);
    void cube(std::string frame_id, std::string ns, int id, geometry_msgs::msg::Point pt,
              geometry_msgs::msg::Quaternion quat, std::vector<float> color, float s_x, float s_y, float s_z);
    void sphere(std::string frame_id, std::string ns, int id, geometry_msgs::msg::Point pt,
                geometry_msgs::msg::Quaternion quat, std::vector<float> color, float s_x, float s_y, float s_z);
    void cylinder(std::string frame_id, std::string ns, int id, geometry_msgs::msg::Point pt,
                  geometry_msgs::msg::Quaternion quat, std::vector<float> color, float s_x, float s_y, float s_z);
    void line_strip(std::string frame_id, std::string ns, int id, std::vector<geometry_msgs::msg::Point> pts,
                    std::vector<float> color, float s);
    void line_list(std::string frame_id, std::string ns, int id, std::vector<geometry_msgs::msg::Point> pts,
                   std::vector<float> color, float s);
    void cube_list(std::string frame_id, std::string ns, int id, std::vector<geometry_msgs::msg::Point> pts,
                   geometry_msgs::msg::Quaternion quat, std::vector<float> color, float s_x, float s_y, float s_z);
    void sphere_list(std::string frame_id, std::string ns, int id, std::vector<geometry_msgs::msg::Point> pts,
                     geometry_msgs::msg::Quaternion quat, std::vector<float> color, float s_x, float s_y, float s_z);
    void points(std::string frame_id, std::string ns, int id, std::vector<geometry_msgs::msg::Point> pts,
                std::vector<float> color, float s_w, float s_h);
    void text_view_facing(std::string frame_id, std::string ns, int id, geometry_msgs::msg::Point pt,
                          std::vector<float> color, std::string text, float s);
};

#endif // HELP_MARKER_HPP_