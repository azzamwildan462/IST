#include "master/master.hpp"

void Master::process_marker()
{
    // ===========
    // BODY marker
    // ===========
    marker.cube("body_link", "body", 1, get_point(0, 0, 0), get_quat(0, 0, 0), {0.2, 0.2, 0.2, 0.5}, 0.5, 1.5, 1.5);
}

void Master::process_local_fsm()
{
    switch (local_fsm.value)
    {
    case FSM_LOCAL_PRE_FOLLOW_LANE:
        manual_motion(0, 0, 0);
        time_start_follow_lane = current_time;
        local_fsm.value = FSM_LOCAL_FOLLOW_LANE;
        break;

    case FSM_LOCAL_FOLLOW_LANE:
        follow_lane_2_cam(7, 0, 0.3);

        if ((current_time - time_start_follow_lane).seconds() > 10 && aruco_kanan_detected)
        {
            local_fsm.value = FSM_LOCAL_MENUNGGU_STATION_1;
        }
        break;

    case FSM_LOCAL_MENUNGGU_STATION_1:
        manual_motion(-profile_max_braking, 0, 0);
        break;

    case FSM_LOCAL_MENUNGGU_STATION_2:
        manual_motion(-profile_max_braking, 0, 0);
        break;
    }
}

void Master::process_transmitter()
{
    std_msgs::msg::Int16 msg_global_fsm;
    msg_global_fsm.data = global_fsm.value;
    pub_global_fsm->publish(msg_global_fsm);

    std_msgs::msg::Int16 msg_local_fsm;
    msg_local_fsm.data = local_fsm.value;
    pub_local_fsm->publish(msg_local_fsm);

    std_msgs::msg::Float32MultiArray msg_to_ui;
    msg_to_ui.data.push_back(target_velocity);
    msg_to_ui.data.push_back(actuation_wz);
    msg_to_ui.data.push_back(fb_final_vel_dxdydo[0]);
    msg_to_ui.data.push_back(fb_steering_angle);
    pub_to_ui->publish(msg_to_ui);

    std_msgs::msg::Float32MultiArray msg_actuator;
    msg_actuator.data.push_back(actuation_vx);
    msg_actuator.data.push_back(actuation_wz);
    pub_actuator->publish(msg_actuator);

    std_msgs::msg::Int16 msg_transmission_master;
    msg_transmission_master.data = transmission_joy_master;
    pub_transmission_master->publish(msg_transmission_master);
}

//=================================================================================================

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
