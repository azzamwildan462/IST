#include "master/master.hpp"

void Master::process_marker()
{
    // ===========
    // BODY marker
    // ===========
    marker.cube("body_link", "body", 1, get_point(0, 0, 0), get_quat(0, 0, 0), {0.2, 0.2, 0.2, 0.5}, 0.5, 1.5, 1.5);
}

void Master::process_fsm()
{
}

geometry_msgs::msg::Point
Master::get_point(double x, double y, double z)
{
    geometry_msgs::msg::Point p_msg;
    p_msg.x = x;
    p_msg.y = y;
    p_msg.z = z;
    return p_msg;
}

geometry_msgs::msg::Quaternion
Master::get_quat(double r, double p, double y)
{
    tf2::Quaternion q_tf2;
    q_tf2.setRPY(r, p, y);
    geometry_msgs::msg::Quaternion q_msg;
    q_msg.x = q_tf2.x();
    q_msg.y = q_tf2.y();
    q_msg.z = q_tf2.z();
    q_msg.w = q_tf2.w();
    return q_msg;
}

std::vector<geometry_msgs::msg::Point>
Master::get_path(float x0, float y0, float x1, float y1, float resolution)
{
    std::vector<geometry_msgs::msg::Point> ps;

    float dx = x1 - x0;
    float dy = y1 - y0;

    int n = sqrt(dx * dx + dy * dy) / resolution;
    for (int i = 0; i < n; i++)
    {
        float x = x0 + dx * i / n;
        float y = y0 + dy * i / n;
        ps.push_back(get_point(x, y, 0));
    }
    float x = x1;
    float y = y1;
    ps.push_back(get_point(x, y, 0));

    return ps;
}

geometry_msgs::msg::Point
Master::get_near(float x, float y, std::vector<geometry_msgs::msg::Point> ps)
{
    geometry_msgs::msg::Point p;

    float d_min = __FLT_MAX__;

    for (auto i : ps)
    {
        float dx = x - i.x;
        float dy = y - i.y;

        float d = sqrt(dx * dx + dy * dy);
        if (d_min > d)
        {
            d_min = d;
            p = i;
        }
    }

    return p;
}

void Master::set_initialpose(float x, float y, float yaw)
{
    geometry_msgs::msg::PoseWithCovarianceStamped msg_initialpose;
    msg_initialpose.header.frame_id = "map";
    msg_initialpose.header.stamp = this->now();
    msg_initialpose.pose.pose.position = get_point(x, y, 0);
    msg_initialpose.pose.pose.orientation = get_quat(0, 0, yaw);
    pub_initialpose->publish(msg_initialpose);
}