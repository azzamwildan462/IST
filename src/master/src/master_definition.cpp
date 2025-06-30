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
    local_fsm.reentry(999, 1);

    static float stop_time_s = timeout_terminal_1;

    switch (local_fsm.value)
    {
    case 999:
        local_fsm.resetUptimeTimeout();
        local_fsm.value = FSM_LOCAL_PRE_FOLLOW_LANE;
        break;

    case FSM_LOCAL_PRE_FOLLOW_LANE:
        manual_motion(-1, 0, 0);
        time_start_follow_lane = current_time;
        local_fsm.value = FSM_LOCAL_FOLLOW_LANE;
        break;

    case FSM_LOCAL_FOLLOW_LANE:

        if (metode_following == 0)
        {
            follow_waypoints(2, 0, 1.5, 1.5, true);
        }
        else if (metode_following == 1)
        {
            follow_lane(2, 0, 1.5);
        }
        else if (metode_following == 2)
        {
            fusion_follow_lane_waypoints(2, 0, 1.5, 1, true);
        }

        if ((current_time - time_start_follow_lane).seconds() > 10)
        {
            // if (aruco_nearest_marker_id == ARUCO_TERMINAL_1)
            // {
            //     local_fsm.resetUptimeTimeout();
            //     local_fsm.value = FSM_LOCAL_MENUNGGU_STATION_1;
            // }
            // else if (aruco_nearest_marker_id == ARUCO_TERMINAL_2)
            // {
            //     local_fsm.resetUptimeTimeout();
            //     local_fsm.value = FSM_LOCAL_MENUNGGU_STATION_2;
            // }

            for (size_t i = 0; i < terminals.terminals.size(); i++)
            {
                float error_arah_hadap = terminals.terminals[i].target_pose_theta - fb_final_pose_xyo[2];
                while (error_arah_hadap > M_PI)
                    error_arah_hadap -= 2 * M_PI;
                while (error_arah_hadap < -M_PI)
                    error_arah_hadap += 2 * M_PI;

                if (fabs(error_arah_hadap) < 0.87)
                {
                    float jarak_robot_terminal = pythagoras(fb_final_pose_xyo[0], fb_final_pose_xyo[1], terminals.terminals[i].target_pose_x, terminals.terminals[i].target_pose_y);
                    if (jarak_robot_terminal < terminals.terminals[i].radius_area)
                    {
                        if (terminals.terminals[i].type == TERMINAL_TYPE_STOP1)
                        {
                            local_fsm.resetUptimeTimeout();
                            local_fsm.value = FSM_LOCAL_MENUNGGU_STATION_1;
                            stop_time_s = terminals.terminals[i].stop_time_s;
                        }
                        else if (terminals.terminals[i].type == TERMINAL_TYPE_STOP2)
                        {
                            local_fsm.resetUptimeTimeout();
                            local_fsm.value = FSM_LOCAL_MENUNGGU_STATION_2;
                            stop_time_s = terminals.terminals[i].stop_time_s;
                        }
                        else if (terminals.terminals[i].type == TERMINAL_TYPE_STOP)
                        {
                            local_fsm.resetUptimeTimeout();
                            local_fsm.value = FSM_LOCAL_MENUNGGU_STOP;
                            stop_time_s = terminals.terminals[i].stop_time_s;
                        }
                        break;
                    }
                }
            }
        }
        break;

    case FSM_LOCAL_MENUNGGU_STATION_1:
        if (metode_following == 0)
        {
            follow_waypoints_gas_manual(-1, 0, 0, 1, true);
        }
        else if (metode_following == 1)
        {
            follow_lane_gas_manual(-1, 0, 0);
        }
        else if (metode_following == 2)
        {
            fusion_follow_lane_waypoints_gas_manual(-1, 0, 0, 1, true);
        }
        // IN_NEXT_TERMINAL
        if ((fb_beckhoff_digital_input & IN_NEXT_TERMINAL) == IN_NEXT_TERMINAL)
        {
            local_fsm.value = FSM_LOCAL_PRE_FOLLOW_LANE;
        }

        // local_fsm.timeout(FSM_LOCAL_PRE_FOLLOW_LANE, stop_time_s);
        break;

    case FSM_LOCAL_MENUNGGU_STATION_2:
        if (metode_following == 0)
        {
            follow_waypoints_gas_manual(-1, 0, 0, 1, true);
        }
        else if (metode_following == 1)
        {
            follow_lane_gas_manual(-1, 0, 0);
        }
        else if (metode_following == 2)
        {
            fusion_follow_lane_waypoints_gas_manual(-1, 0, 0, 1, true);
        }

        if ((fb_beckhoff_digital_input & IN_NEXT_TERMINAL) == IN_NEXT_TERMINAL)
        {
            local_fsm.value = FSM_LOCAL_PRE_FOLLOW_LANE;
        }

        // local_fsm.timeout(FSM_LOCAL_PRE_FOLLOW_LANE, stop_time_s);
        break;

    case FSM_LOCAL_MENUNGGU_STOP:
        if (metode_following == 0)
        {
            follow_waypoints_gas_manual(-1, 0, 0, 1, true);
        }
        else if (metode_following == 1)
        {
            follow_lane_gas_manual(-1, 0, 0);
        }
        else if (metode_following == 2)
        {
            fusion_follow_lane_waypoints_gas_manual(-1, 0, 0, 1, true);
        }

        if ((fb_beckhoff_digital_input & IN_NEXT_TERMINAL) == IN_NEXT_TERMINAL)
        {
            local_fsm.value = FSM_LOCAL_PRE_FOLLOW_LANE;
        }

        // local_fsm.timeout(FSM_LOCAL_PRE_FOLLOW_LANE, stop_time_s);
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
    msg_to_ui.data.push_back(fb_encoder_meter);
    msg_to_ui.data.push_back(fb_steering_angle);
    pub_to_ui->publish(msg_to_ui);

    std_msgs::msg::Float32MultiArray msg_actuator;
    msg_actuator.data.push_back(actuation_vx);
    msg_actuator.data.push_back(actuation_wz);
    pub_actuator->publish(msg_actuator);

    std_msgs::msg::Int16 msg_transmission_master;
    msg_transmission_master.data = transmission_joy_master;
    pub_transmission_master->publish(msg_transmission_master);

    tf2::Quaternion q;
    q.setRPY(0, 0, fb_filtered_final_pose_xyo[2]);
    nav_msgs::msg::Odometry msg_pose_filtered;
    msg_pose_filtered.pose.pose.position.x = fb_filtered_final_pose_xyo[0];
    msg_pose_filtered.pose.pose.position.y = fb_filtered_final_pose_xyo[1];
    msg_pose_filtered.pose.pose.orientation.x = q.x();
    msg_pose_filtered.pose.pose.orientation.y = q.y();
    msg_pose_filtered.pose.pose.orientation.z = q.z();
    msg_pose_filtered.pose.pose.orientation.w = q.w();
    pub_pose_filtered->publish(msg_pose_filtered);

    std_msgs::msg::UInt8 msg_slam_status;
    msg_slam_status.data = slam_status;
    pub_slam_status->publish(msg_slam_status);

    static uint16_t divider_waypoint_pub_counter = 0;
    if (divider_waypoint_pub_counter++ >= 25)
    {
        divider_waypoint_pub_counter = 0;

        sensor_msgs::msg::PointCloud msg_waypoints;
        for (auto i : waypoints)
        {
            geometry_msgs::msg::Point32 p;
            p.x = i.x;
            p.y = i.y;
            p.z = 0;
            msg_waypoints.points.push_back(p);
        }
        pub_waypoints->publish(msg_waypoints);

        pub_terminals->publish(terminals);
    }
}

void Master::process_load_terminals()
{
    std::ifstream fin;
    std::string line;
    std::vector<std::string> tokens;
    bool is_terminal_loaded_normally = false;
    try
    {
        fin.open(terminal_file_path, std::ios::in);
        if (fin.is_open())
        {
            is_terminal_loaded_normally = true;
            while (std::getline(fin, line))
            {
                if (line.find("type") != std::string::npos)
                {
                    continue;
                }
                boost::split(tokens, line, boost::is_any_of(","));

                for (auto &token : tokens)
                {
                    boost::trim(token);
                }

                ros2_interface::msg::Terminal terminal;
                terminal.type = std::stoi(tokens[0]);
                terminal.id = std::stoi(tokens[1]);
                terminal.target_pose_x = std::stof(tokens[2]);
                terminal.target_pose_y = std::stof(tokens[3]);
                terminal.target_pose_theta = std::stof(tokens[4]);
                terminal.target_max_velocity_x = std::stof(tokens[5]);
                terminal.target_max_velocity_y = std::stof(tokens[6]);
                terminal.target_max_velocity_theta = std::stof(tokens[7]);
                terminal.radius_area = std::stof(tokens[8]);
                terminal.target_lookahead_distance = std::stof(tokens[9]);
                terminal.obs_scan_r = std::stof(tokens[10]);
                terminal.stop_time_s = std::stof(tokens[11]);
                terminal.scan_min_x = std::stof(tokens[12]);
                terminal.scan_max_x = std::stof(tokens[13]);
                terminal.scan_min_y = std::stof(tokens[14]);
                terminal.scan_max_y = std::stof(tokens[15]);
                terminal.obs_threshold = std::stof(tokens[16]);

                if (transform_map2odom)
                {
                    tf2::Transform tf_terminal_pose;
                    tf_terminal_pose.setOrigin(tf2::Vector3(terminal.target_pose_x, terminal.target_pose_y, 0));
                    tf2::Quaternion q;
                    q.setRPY(0, 0, terminal.target_pose_theta);
                    tf_terminal_pose.setRotation(q);

                    tf2::Transform tf_transformed = manual_map2odom_tf * tf_terminal_pose;

                    terminal.target_pose_x = tf_transformed.getOrigin().getX();
                    terminal.target_pose_y = tf_transformed.getOrigin().getY();

                    tf2::Matrix3x3 m(tf_transformed.getRotation());
                    double roll, pitch, yaw;
                    m.getRPY(roll, pitch, yaw);
                    terminal.target_pose_theta = yaw;
                }

                terminals.terminals.push_back(terminal);

                tokens.clear();
            }
            fin.close();

            logger.info("Terminal file loaded");
        }

        if (fin.fail() && !fin.is_open() && !is_terminal_loaded_normally)
        {
            logger.warn("Failed to load terminal file, Recreate the terminal file");
            process_save_terminals();
        }
    }
    catch (const std::exception &e)
    {
        logger.error("Failed to load terminal file: %s, Recreate the terminal file", e.what());
        process_save_terminals();
    }
}

void Master::process_save_terminals()
{
    std::ofstream fout;

    try
    {
        fout.open(terminal_file_path, std::ios::out);
        if (fout.is_open())
        {
            fout << "type, id, x, y, theta, max_vx, max_vy, max_vtheta, radius_area, lookahead_distance, obs_scan_r, stop_time_s, scan_min_x, scan_max_x, scan_min_y, scan_max_y, obs_threshold" << std::endl;
            for (auto terminal : terminals.terminals)
            {
                int terminal_type_integer = terminal.type;
                int terminal_id_integer = terminal.id;
                fout << terminal_type_integer << ", " << terminal_id_integer << ", " << terminal.target_pose_x << ", " << terminal.target_pose_y << ", " << terminal.target_pose_theta << ", " << terminal.target_max_velocity_x << ", " << terminal.target_max_velocity_y << ", " << terminal.target_max_velocity_theta << ", " << terminal.radius_area << ", " << terminal.target_lookahead_distance << ", " << terminal.obs_scan_r << ", " << terminal.stop_time_s << ", " << terminal.scan_min_x << ", " << terminal.scan_max_x << ", " << terminal.scan_min_y << ", " << terminal.scan_max_y << ", " << terminal.obs_threshold << std::endl;
            }
            fout.close();

            logger.info("Terminal file saved");
        }
    }
    catch (const std::exception &e)
    {
        logger.warn("Failed to open terminal file, Recreate the terminal file");
    }
}

void Master::process_load_waypoints()
{
    static float prev_wp_x = 0;
    static float prev_wp_y = 0;

    waypoints.clear();

    if (!boost::filesystem::exists(waypoint_file_path))
    {
        logger.error("File %s does not exist. Create a new file.", waypoint_file_path.c_str());
        std::ofstream file(waypoint_file_path);
        file << "x,y,fb_velocity,fb_steering" << std::endl;
        file.close();
    }

    std::ifstream file;

    file.open(waypoint_file_path);
    if (file.is_open())
    {
        file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        while (file.good())
        {
            if (file.peek() == EOF)
            {
                break;
            }
            waypoint_t wp;
            file >> wp.x;
            file.ignore(std::numeric_limits<std::streamsize>::max(), ',');
            file >> wp.y;
            file.ignore(std::numeric_limits<std::streamsize>::max(), ',');
            file >> wp.fb_velocity;
            file.ignore(std::numeric_limits<std::streamsize>::max(), ',');
            file >> wp.fb_steering;
            file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

            if (transform_map2odom)
            {
                tf2::Transform tf_wp;
                tf_wp.setOrigin(tf2::Vector3(wp.x, wp.y, 0));
                tf2::Quaternion q;
                q.setRPY(0, 0, 0);
                tf_wp.setRotation(q);

                tf2::Transform tf_transformed = manual_map2odom_tf * tf_wp;
                wp.x = tf_transformed.getOrigin().getX();
                wp.y = tf_transformed.getOrigin().getY();
            }

            float dx = wp.x - prev_wp_x;
            float dy = wp.y - prev_wp_y;
            wp.arah = atan2f(dy, dx);

            prev_wp_x = wp.x;
            prev_wp_y = wp.y;

            waypoints.push_back(wp);
        }
        file.close();
        logger.info("Read %d waypoints from file %s.", waypoints.size(), waypoint_file_path.c_str());
    }
}

void Master::process_save_waypoints()
{
    std::ofstream file;
    file.open(waypoint_file_path);

    if (file.is_open())
    {
        file << "x,y,fb_velocity,fb_steering" << std::endl;
        for (auto i : waypoints)
        {
            file << i.x << "," << i.y << "," << i.fb_velocity << "," << i.fb_steering << std::endl;
        }
        file.close();
        logger.info("Saved %d waypoints to file %s.", waypoints.size(), waypoint_file_path.c_str());
    }
    else
    {
        logger.error("Failed to save waypoints to file %s.", waypoint_file_path.c_str());
    }
}

void Master::process_record_route()
{
    static float prev_x = fb_final_pose_xyo[0];
    static float prev_y = fb_final_pose_xyo[1];

    float dx = fb_final_pose_xyo[0] - prev_x;
    float dy = fb_final_pose_xyo[1] - prev_y;
    float d = sqrt(dx * dx + dy * dy);

    if (d > 0.2)
    {
        waypoint_t wp;
        wp.x = fb_final_pose_xyo[0];
        wp.y = fb_final_pose_xyo[1];
        wp.fb_velocity = fb_encoder_meter;
        wp.fb_steering = fb_steering_angle;
        waypoints.push_back(wp);
        prev_x = wp.x;
        prev_y = wp.y;
        // logger.info("Recorded waypoint %.2f %.2f %.2f %.2f", wp.x, wp.y, wp.fb_velocity, wp.fb_steering);
    }
}

void Master::process_add_terminal()
{
    static float prev_x = -9999;
    static float prev_y = -9999;

    float dx = fb_final_pose_xyo[0] - prev_x;
    float dy = fb_final_pose_xyo[1] - prev_y;
    float d = sqrt(dx * dx + dy * dy);

    /* Prevent double klik atau semacamnya */
    if (d > 0.2)
    {
        ros2_interface::msg::Terminal terminal;
        terminal.type = TERMINAL_TYPE_STOP;
        terminal.id = terminals.terminals.size();
        terminal.target_pose_x = fb_final_pose_xyo[0];
        terminal.target_pose_y = fb_final_pose_xyo[1];
        terminal.target_pose_theta = fb_final_pose_xyo[2];
        terminal.target_max_velocity_x = 0.9;
        terminal.target_max_velocity_y = 0.9;
        terminal.target_max_velocity_theta = 0.2; // gk dipakai
        terminal.radius_area = 3;
        terminal.target_lookahead_distance = 3;
        terminal.obs_scan_r = 2;
        terminal.stop_time_s = 10;
        terminals.terminals.push_back(terminal);
        logger.info("Add Terminal Success");
    }
}

void Master::set_lidar_obstacle_filter_param(double scan_range, double min_y, double max_y, double obstacle_error_tolerance)
{
    // Prepare parameter changes
    std::vector<rclcpp::Parameter> params;
    params.emplace_back("scan_range", scan_range);
    params.emplace_back("scan_min_y", min_y);
    params.emplace_back("scan_max_y", max_y);
    params.emplace_back("obstacle_error_tolerance", obstacle_error_tolerance);

    // Apply parameters atomically
    auto result = lidar_obstacle_param_client->set_parameters(params);
}

//==================================================================================================

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

void Master::set_pose_offset(float x, float y, float yaw)
{
    nav_msgs::msg::Odometry msg_pose_offset;
    msg_pose_offset.header.frame_id = "pose_offset";
    msg_pose_offset.header.stamp = this->now();
    msg_pose_offset.pose.pose.position = get_point(x, y, 0);
    msg_pose_offset.pose.pose.orientation = get_quat(0, 0, yaw);
    pub_pose_offset->publish(msg_pose_offset);
}
