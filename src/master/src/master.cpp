#include "master/master.hpp"
#include "rclcpp/rclcpp.hpp"

Master::Master()
    : Node("master")
{
    this->declare_parameter("offset_sudut_steering", 0.0);
    this->get_parameter("offset_sudut_steering", offset_sudut_steering);

    this->declare_parameter("use_ekf_odometry", false);
    this->get_parameter("use_ekf_odometry", use_ekf_odometry);

    this->declare_parameter<std::vector<double>>("pid_terms", {0.0070, 0.000000, 0, 0.02, -0.04, 0.4, -0.0005, 0.0005});
    this->get_parameter("pid_terms", pid_terms);

    this->declare_parameter("waypoint_file_path", "/home/robot/waypoints.yaml");
    this->get_parameter("waypoint_file_path", waypoint_file_path);

    this->declare_parameter("terminal_file_path", "/home/robot/terminal.yaml");
    this->get_parameter("terminal_file_path", terminal_file_path);

    this->declare_parameter("wheelbase", 0.75);
    this->get_parameter("wheelbase", wheelbase);

    this->declare_parameter("metode_following", 0);
    this->get_parameter("metode_following", metode_following);

    this->declare_parameter("enable_obs_detection", false);
    this->get_parameter("enable_obs_detection", enable_obs_detection);

    this->declare_parameter("enable_obs_detection_camera", false);
    this->get_parameter("enable_obs_detection_camera", enable_obs_detection_camera);

    this->declare_parameter("timeout_terminal_1", 10.0);
    this->get_parameter("timeout_terminal_1", timeout_terminal_1);

    this->declare_parameter("timeout_terminal_2", 10.0);
    this->get_parameter("timeout_terminal_2", timeout_terminal_2);

    this->declare_parameter("profile_max_velocity", 1.0);
    this->get_parameter("profile_max_velocity", profile_max_velocity);

    this->declare_parameter("transform_map2odom", false);
    this->get_parameter("transform_map2odom", transform_map2odom);

    this->declare_parameter("toribay_ready_threshold", 0.5);
    this->get_parameter("toribay_ready_threshold", toribay_ready_threshold);

    this->declare_parameter("use_filtered_pose", false);
    this->get_parameter("use_filtered_pose", use_filtered_pose);

    this->declare_parameter("debug_motion", false);
    this->get_parameter("debug_motion", debug_motion);

    this->declare_parameter("threshold_icp_score", 100.0);
    this->get_parameter("threshold_icp_score", threshold_icp_score);

    this->declare_parameter<std::vector<double>>("complementary_terms", {0.30, 0.03, 0.01, 0.9});
    this->get_parameter("complementary_terms", complementary_terms);

    this->declare_parameter("camera_scan_min_x_", 0.2);
    this->get_parameter("camera_scan_min_x_", camera_scan_min_x_);
    this->declare_parameter("camera_scan_max_x_", 2.0);
    this->get_parameter("camera_scan_max_x_", camera_scan_max_x_);
    this->declare_parameter("camera_scan_min_y_", -1.0);
    this->get_parameter("camera_scan_min_y_", camera_scan_min_y_);
    this->declare_parameter("camera_scan_max_y_", 1.0);
    this->get_parameter("camera_scan_max_y_", camera_scan_max_y_);

    if (!logger.init())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize logger");
        rclcpp::shutdown();
    }

    cloud_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    pub_initialpose = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("slam/initialpose", 10);
    pub_global_fsm = this->create_publisher<std_msgs::msg::Int16>("/master/global_fsm", 1);
    pub_local_fsm = this->create_publisher<std_msgs::msg::Int16>("/master/local_fsm", 1);
    pub_to_ui = this->create_publisher<std_msgs::msg::Float32MultiArray>("/master/to_ui", 1);
    pub_actuator = this->create_publisher<std_msgs::msg::Float32MultiArray>("/master/actuator", 1);
    pub_transmission_master = this->create_publisher<std_msgs::msg::Int16>("/master/transmission", 1);
    pub_pose_offset = this->create_publisher<nav_msgs::msg::Odometry>("/master/pose_offset", 1);
    pub_waypoints = this->create_publisher<sensor_msgs::msg::PointCloud>("/master/waypoints", 1);
    pub_terminals = this->create_publisher<ros2_interface::msg::TerminalArray>("/master/terminals", 1);
    pub_obs_find = this->create_publisher<std_msgs::msg::Float32>("/master/obs_find", 1);
    pub_pose_filtered = this->create_publisher<nav_msgs::msg::Odometry>("/master/pose_filtered", 1);
    pub_slam_status = this->create_publisher<std_msgs::msg::UInt8>("/master/slam_status", 1);
    pub_camera_obs_emergency = this->create_publisher<std_msgs::msg::Float32>("/master/camera_obs_emergency", 1);
    pub_master_status_emergency = this->create_publisher<std_msgs::msg::Int16>("/master/status_emergency", 1);
    pub_master_status_klik_terminal_terakhir = this->create_publisher<std_msgs::msg::Int16>("/master/terminal_terakhir", 1);

    auto sub_opts = rclcpp::SubscriptionOptions();
    sub_opts.callback_group = cloud_cb_group_;

    sub_detected_forklift = this->create_subscription<std_msgs::msg::Int8>(
        "/forklift_number", 1, std::bind(&Master::callback_sub_detected_forklift, this, std::placeholders::_1));
    sub_obs_find = this->create_subscription<std_msgs::msg::Float32>(
        "/lidar_obstacle_filter/obs_find", 1, std::bind(&Master::callback_sub_obs_find, this, std::placeholders::_1));
    sub_CAN_eps_encoder = this->create_subscription<std_msgs::msg::Float32>(
        "/can/eps_encoder", 1, std::bind(&Master::callback_sub_CAN_eps_encoder, this, std::placeholders::_1));
    sub_beckhoff_sensor = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/beckhoff/analog_input", 1, std::bind(&Master::callback_sub_beckhoff_sensor, this, std::placeholders::_1));
    sub_lane_kiri = this->create_subscription<ros2_interface::msg::PointArray>(
        "/lane_detection/point_kiri", 1, std::bind(&Master::callback_sub_lane_kiri, this, std::placeholders::_1));
    sub_lane_kanan = this->create_subscription<ros2_interface::msg::PointArray>(
        "/lane_detection/point_kanan", 1, std::bind(&Master::callback_sub_lane_kanan, this, std::placeholders::_1));
    sub_lane_tengah = this->create_subscription<ros2_interface::msg::PointArray>(
        "/lane_detection/point_tengah", 1, std::bind(&Master::callback_sub_lane_tengah, this, std::placeholders::_1));
    sub_error_code_beckhoff = this->create_subscription<std_msgs::msg::Int16>(
        "/beckhoff/error_code", 1, std::bind(&Master::callback_sub_error_code_beckhoff, this, std::placeholders::_1));
    sub_error_code_pose_estimator = this->create_subscription<std_msgs::msg::Int16>(
        "/slam/error_code", 1, std::bind(&Master::callback_sub_error_code_pose_estimator, this, std::placeholders::_1));
    sub_error_code_lane_detection = this->create_subscription<std_msgs::msg::Int16>(
        "/lane_detection/error_code", 1, std::bind(&Master::callback_error_code_lane_detection, this, std::placeholders::_1));
    sub_error_code_aruco_detection = this->create_subscription<std_msgs::msg::Int16>(
        "/aruco_detection/error_code", 1, std::bind(&Master::callback_error_code_aruco_detection, this, std::placeholders::_1));
    sub_error_code_obstacle_filter = this->create_subscription<std_msgs::msg::Int16>(
        "/obstacle_filter/error_code", 1, std::bind(&Master::callback_sub_error_code_obstacle_filter, this, std::placeholders::_1));
    sub_joy = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 1, std::bind(&Master::callback_sub_joy, this, std::placeholders::_1));
    sub_error_code_can = this->create_subscription<std_msgs::msg::Int16>(
        "/can/error_code", 1, std::bind(&Master::callback_sub_error_code_can, this, std::placeholders::_1));
    sub_encoder_meter = this->create_subscription<std_msgs::msg::Float32>(
        "/encoder_meter", 1, std::bind(&Master::callback_sub_encoder_meter, this, std::placeholders::_1));
    sub_key_pressed = this->create_subscription<std_msgs::msg::Int16>(
        "/key_pressed", 1, std::bind(&Master::callback_sub_key_pressed, this, std::placeholders::_1));
    sub_ui_control_btn = this->create_subscription<std_msgs::msg::UInt16>(
        "/master/ui_control_btn", 1, std::bind(&Master::callback_sub_ui_control_btn, this, std::placeholders::_1));
    sub_ui_control_velocity_and_steering = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/master/ui_target_velocity_and_steering", 1, std::bind(&Master::callback_sub_ui_control_velocity_and_steering, this, std::placeholders::_1));
    sub_CAN_eps_mode_fb = this->create_subscription<std_msgs::msg::UInt8>(
        "/can/eps_mode", 1, std::bind(&Master::callback_sub_CAN_eps_mode_fb, this, std::placeholders::_1));
    sub_beckhoff_digital_input = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
        "/beckhoff/digital_input", 1, std::bind(&Master::callback_sub_beckhoff_digital_input, this, std::placeholders::_1));
    sub_aruco_nearest_marker_id = this->create_subscription<std_msgs::msg::Int16>(
        "/aruco_detection/aruco_nearest_marker_id", 1, std::bind(&Master::callback_sub_aruco_nearest_marker_id, this, std::placeholders::_1));
    sub_lidar_depan_scan = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 1, std::bind(&Master::callback_sub_lidar_depan_scan, this, std::placeholders::_1));
    sub_lidar_belakang_scan = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/lidar2/scan", 1, std::bind(&Master::callback_sub_lidar_belakang_scan, this, std::placeholders::_1));
    sub_rtabmap_info = this->create_subscription<rtabmap_msgs::msg::Info>(
        "/slam/info", 1, std::bind(&Master::callback_sub_rtabmap_info, this, std::placeholders::_1));
    sub_map = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/slam/map", 1, std::bind(&Master::callback_sub_map, this, std::placeholders::_1));
    sub_localization_pose = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/slam/localization_pose", 1, std::bind(&Master::callback_sub_localization_pose, this, std::placeholders::_1));
    if (enable_obs_detection_camera)
    {
        sub_camera_pcl = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/rs2_cam_main/depth/color/points", 1, std::bind(&Master::callback_sub_camera_pcl, this, std::placeholders::_1), sub_opts);
    }
    sub_icp_score = this->create_subscription<std_msgs::msg::Float32>(
        "/lidar_obstacle_filter/icp_score", 1, std::bind(&Master::callback_sub_icp_score, this, std::placeholders::_1), sub_opts);
    sub_detected_forklift_contour = this->create_subscription<std_msgs::msg::Float32>(
        "/forklift_detector_vision/forklift_detected", 1, std::bind(&Master::callback_sub_detected_forklift_contour, this, std::placeholders::_1), sub_opts);
    sub_gyro_counter = this->create_subscription<std_msgs::msg::UInt8>(
        "/can/gyro_counter", 1, std::bind(&Master::callback_sub_gyro_counter, this, std::placeholders::_1), sub_opts);

    srv_set_record_route_mode = this->create_service<std_srvs::srv::SetBool>(
        "/master/set_record_route_mode", std::bind(&Master::callback_srv_set_record_route_mode, this, std::placeholders::_1, std::placeholders::_2));

    srv_set_terminal = this->create_service<std_srvs::srv::SetBool>(
        "/master/set_terminal", std::bind(&Master::callback_srv_set_terminal, this, std::placeholders::_1, std::placeholders::_2));

    srv_rm_terminal = this->create_service<std_srvs::srv::SetBool>(
        "/master/rm_terminal", std::bind(&Master::callback_srv_rm_terminal, this, std::placeholders::_1, std::placeholders::_2));

    sub_icp_pose_estimate = this->create_subscription<nav_msgs::msg::Odometry>(
        "/lidar_obstacle_filter/icp_estimate", 1, std::bind(&Master::callback_sub_icp_pose_estimate, this, std::placeholders::_1));
    if (use_ekf_odometry)
        sub_odometry = this->create_subscription<nav_msgs::msg::Odometry>(
            "/slam/odometry/filtered", 1, std::bind(&Master::callback_sub_odometry, this, std::placeholders::_1));
    else
        sub_odometry = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 1, std::bind(&Master::callback_sub_odometry, this, std::placeholders::_1));

    lidar_obstacle_param_client = std::make_shared<rclcpp::SyncParametersClient>(
        this, "lidar_obstacle_filter");

    // while (!tf_is_initialized)
    // {
    //     sleep(1);
    //     try
    //     {
    //         tf_lidar_base = tf_lidar_base_buffer->lookupTransform("base_link", "camera_depth_optical_frame", tf2::TimePointZero);
    //         tf_is_initialized = true;
    //     }
    //     catch (tf2::TransformException &ex)
    //     {
    //         logger.error("tunggu");
    //         tf_is_initialized = false;
    //     }
    // }

    //----Timer
    tim_50hz = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&Master::callback_tim_50hz, this), timer_cb_group_);

    // pid_vx.init(0.0030, 0.000000, 0, dt, -0.04, 0.4, -0.0005, 0.0005);
    pid_vx.init(pid_terms[0], pid_terms[1], pid_terms[2], pid_terms[3], pid_terms[4], pid_terms[5], pid_terms[6], pid_terms[7]);

    logger.info("Master init success %.2f", threshold_icp_score);
    global_fsm.value = FSM_GLOBAL_INIT;
}
Master::~Master()
{
}

void Master::callback_sub_detected_forklift(const std_msgs::msg::Int8::SharedPtr msg)
{
    detected_forklift_number = msg->data;

    static uint16_t counter_detected = 0;
    static uint16_t counter_not_detected = 0;

    if (detected_forklift_number >= 0)
    {
        counter_detected++;
        counter_not_detected = 0;
    }
    else
    {
        counter_detected = 0;
        counter_not_detected++;
    }

    if (counter_detected > 10000)
    {
        counter_detected = 10000;
    }
    if (counter_not_detected > 10000)
    {
        counter_not_detected = 10000;
    }

    if (counter_detected > 0)
    {
        detected_forklift_number_filtered = true;
    }
    if (counter_not_detected > 300)
    {
        detected_forklift_number_filtered = false;
    }
}

void Master::callback_sub_gyro_counter(const std_msgs::msg::UInt8::SharedPtr msg)
{
    gyro_counter = msg->data;
}
void Master::callback_sub_detected_forklift_contour(const std_msgs::msg::Float32::SharedPtr msg)
{
    detected_forklift_contour = msg->data;
}
void Master::callback_sub_icp_score(const std_msgs::msg::Float32::SharedPtr msg)
{
    icp_score = msg->data;
}

void Master::callback_sub_camera_pcl(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    last_time_kamera_pcl = rclcpp::Clock(RCL_SYSTEM_TIME).now();

    // if (!tf_is_initialized)
    // {
    //     return;
    // }

    // static pcl::PointCloud<pcl::PointXYZ> points_lidar_;
    // static pcl::PointCloud<pcl::PointXYZ> points_lidar2base_;
    // static pcl::PointCloud<pcl::PointXYZ> points_lidar2base_filtered_;
    // static pcl::PointCloud<pcl::PointXYZ> temp_;
    // static pcl::PointCloud<pcl::PointXYZ> filtered_;

    // pass_x_.setFilterLimits(camera_scan_min_x_, camera_scan_max_x_);
    // pass_y_.setFilterLimits(camera_scan_min_y_, camera_scan_max_y_);

    // pcl::fromROSMsg(*msg, points_lidar_);

    // if (points_lidar_.empty())
    // {
    //     points_lidar2base_.clear();
    //     filtered_.clear();
    // }
    // else
    // {
    //     pcl_ros::transformPointCloud(points_lidar_, points_lidar2base_, tf_lidar_base);

    //     // voxel_.setInputCloud(points_lidar2base_.makeShared());
    //     // voxel_.filter(points_lidar2base_);

    //     pass_x_.setInputCloud(points_lidar2base_.makeShared());
    //     pass_x_.filter(temp_);

    //     pass_y_.setInputCloud(temp_.makeShared());
    //     pass_y_.filter(filtered_);
    // }

    // camera_scan_obs_result = filtered_.size();

    // std_msgs::msg::Float32 out;
    // out.data = static_cast<float>(filtered_.size());
    // pub_camera_obs_emergency->publish(out);

    // Convert to PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    // PassThrough filter on Z
    pcl::PointCloud<pcl::PointXYZ>::Ptr zFiltered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_z.setInputCloud(cloud);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(0.0, camera_scan_max_x_); // keep 0 ≤ z ≤ 3 m
    pass_z.filter(*zFiltered);

    // PassThrough filter on X
    pcl::PointCloud<pcl::PointXYZ>::Ptr xFiltered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass_x;
    pass_x.setInputCloud(zFiltered);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(-camera_scan_max_y_, -camera_scan_min_y_);
    pass_x.filter(*xFiltered);

    // PassThrough filter on Y
    pcl::PointCloud<pcl::PointXYZ>::Ptr yFiltered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass_y;
    pass_y.setInputCloud(xFiltered);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(-0.8, 0.8);
    pass_y.filter(*yFiltered);

    camera_scan_obs_result = yFiltered->size();

    std_msgs::msg::Float32 out;
    out.data = static_cast<float>(yFiltered->size());
    pub_camera_obs_emergency->publish(out);
}

void Master::callback_sub_localization_pose(const geometry_msgs::msg::PoseWithCovarianceStamped msg)
{
    (void)msg;
    is_pose_corrected = true;

    slam_status |= 0b100;
}

void Master::callback_sub_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    map2odom_offset_x = msg->info.origin.position.x;
    map2odom_offset_y = msg->info.origin.position.y;

    // get_angle from quartenion
    tf2::Quaternion q(msg->info.origin.orientation.x, msg->info.origin.orientation.y, msg->info.origin.orientation.z, msg->info.origin.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    map2odom_offset_theta = yaw;

    manual_map2odom_tf.setOrigin(tf2::Vector3(map2odom_offset_x, map2odom_offset_y, 0));
    manual_map2odom_tf.setRotation(q);

    slam_status |= 0b01;
}

void Master::callback_sub_rtabmap_info(const rtabmap_msgs::msg::Info::SharedPtr msg)
{
    if (msg->loop_closure_id > 0 || msg->proximity_detection_id > 0 || msg->landmark_id > 0 || msg->ref_id > 0)
    {
        is_rtabmap_ready = true;
    }

    slam_status |= 0b10;
}

void Master::callback_sub_lidar_depan_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    mutex_lidar_depan_points.lock();
    lidar_depan_points = *msg;
    mutex_lidar_depan_points.unlock();

    std_msgs::msg::Float32 msg_obs_find;
    msg_obs_find.data = local_obstacle_influence(lidar_obs_scan_thr, 4.0);
    obs_find_baru = msg_obs_find.data;
    pub_obs_find->publish(msg_obs_find);
}

void Master::callback_sub_lidar_belakang_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    lidar_belakang_points = *msg;

    static float ret_buffer = 0;
    static float toribay_find_local = 0;
    static const float max_toribay_find_value_local = 100;
    static const float x_min = 0;
    static const float y_min = -0.5;
    static const float y_max = 0.5;
    float toribay_scan_r = 1.5;

    float toribay_scan_r_decimal = 1 / toribay_scan_r;
    toribay_find_local = 0;
    for (size_t i = 0; i < lidar_belakang_points.ranges.size(); ++i)
    {
        float range = lidar_belakang_points.ranges[i];
        if (std::isfinite(range) && range >= lidar_belakang_points.range_min && range <= lidar_belakang_points.range_max)
        {
            float angle = lidar_belakang_points.angle_min + i * lidar_belakang_points.angle_increment;
            angle *= -1;
            float delta_a = 0 - angle;
            while (delta_a > M_PI)
                delta_a -= 2 * M_PI;
            while (delta_a < -M_PI)
                delta_a += 2 * M_PI;

            // Ketika masuk lingkaran scan
            if (std::fabs(delta_a) < 1.57 && range < 6)
            {
                float point_x = range * cosf(angle);
                float point_y = range * sinf(angle);

                // Ketika masuk scan box
                if (point_x > x_min && point_x < toribay_scan_r && point_y > y_min && point_y < y_max)
                {
                    toribay_find_local += toribay_scan_r_decimal * (toribay_scan_r - range);
                }
            }
        }
    }

    if (toribay_find_local > toribay_ready_threshold)
    {
        is_toribay_ready = true;
    }
    else
    {
        is_toribay_ready = false;
    }
}

void Master::callback_sub_aruco_nearest_marker_id(const std_msgs::msg::Int16::SharedPtr msg)
{
    aruco_nearest_marker_id = msg->data;
}

void Master::callback_error_code_lane_detection(const std_msgs::msg::Int16::SharedPtr msg)
{
    last_time_error_code_lane_detection = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    error_code_lane_detection = msg->data;
}

void Master::callback_error_code_aruco_detection(const std_msgs::msg::Int16::SharedPtr msg)
{
    last_time_error_code_aruco_detection = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    error_code_aruco_detection = msg->data;
}

void Master::callback_srv_rm_terminal(const std_srvs::srv::SetBool::Request::SharedPtr request, std_srvs::srv::SetBool::Response::SharedPtr response)
{
    response->success = false;
    if (request->data)
    {
        terminals.terminals.clear();
        response->message = "Success pop terminal";
        response->success = true;
    }
    else
    {
        terminals.terminals.pop_back();
        response->message = "Success clear terminals";
        response->success = true;
    }
}
void Master::callback_srv_set_terminal(const std_srvs::srv::SetBool::Request::SharedPtr request, std_srvs::srv::SetBool::Response::SharedPtr response)
{
    response->success = false;
    if (request->data)
    {
        process_add_terminal();
        response->message = "Success add terminal";
        response->success = true;
    }
    else
    {
        process_save_terminals();
        response->message = "Success save terminals";
        response->success = true;
    }
}
void Master::callback_srv_set_record_route_mode(const std_srvs::srv::SetBool::Request::SharedPtr request, std_srvs::srv::SetBool::Response::SharedPtr response)
{
    response->success = false;
    if (request->data)
    {
        waypoints.clear();
        global_fsm.value = FSM_GLOBAL_RECORD_ROUTE;
        response->message = "Record route mode enabled";
        response->success = true;
    }
    else
    {
        global_fsm.value = FSM_GLOBAL_INIT;
        response->message = "Record route mode disabled";
        response->success = true;
    }
}

void Master::callback_sub_beckhoff_digital_input(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)
{
    last_time_beckhoff = rclcpp::Clock(RCL_SYSTEM_TIME).now();

    fb_beckhoff_digital_input = msg->data[0];
    fb_beckhoff_digital_input |= (msg->data[1] << 8);
    fb_beckhoff_digital_input |= (msg->data[2] << 16);
    fb_beckhoff_digital_input |= (msg->data[3] << 24);
}

void Master::callback_sub_CAN_eps_mode_fb(const std_msgs::msg::UInt8::SharedPtr msg)
{
    fb_eps_mode = msg->data;
}

void Master::callback_sub_ui_control_btn(const std_msgs::msg::UInt16::SharedPtr msg)
{
    last_time_ui_control_btn = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    uint16_t data = msg->data;

    if ((data & 0b10) == 0)
        global_fsm.value = FSM_GLOBAL_OP_3;
    else
        global_fsm.value = (data & 0b11100) >> 2;

    // if (global_fsm.value != FSM_GLOBAL_SAFEOP)
    transmission_joy_master = (data & 0b11100000) >> 5;
}

void Master::callback_sub_ui_control_velocity_and_steering(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    target_velocity_joy_x = msg->data[0];
    target_velocity_joy_wz = msg->data[1];
}

void Master::callback_sub_key_pressed(const std_msgs::msg::Int16::SharedPtr msg)
{
    last_time_key_pressed = rclcpp::Clock(RCL_SYSTEM_TIME).now();

    switch (msg->data)
    {
    case 'j':
        target_velocity_joy_x = 0.4166;
        break;
    case 'u':
        target_velocity_joy_x = 0.638;
        break;
    case 'i':
        target_velocity_joy_x = profile_max_velocity;
        break;
    case 'm':
        target_velocity_joy_wz = fb_steering_angle - 0.1;
        break;
    case 'n':
        target_velocity_joy_wz = fb_steering_angle;
        break;
    case 'b':
        target_velocity_joy_wz = fb_steering_angle + 0.1;
        break;

    case ' ':
        target_velocity_joy_x = -1;
        break;

    case 'e':
        transmission_joy_master = TRANSMISSION_FORWARD;
        break;
    case 'd':
        transmission_joy_master = TRANSMISSION_NEUTRAL;
        break;
    case 'c':
        transmission_joy_master = TRANSMISSION_REVERSE;
        break;
    case 's':
        transmission_joy_master = TRANSMISSION_AUTO;
        break;

    case 'o':
        set_pose_offset(0, 0, 0);
        break;
    case 'p':
        set_pose_offset(1, 2, -1.57);
        break;
    case 'l':
        set_pose_offset(1, 2, 1.57);
        break;

    case '0':
        global_fsm.value = FSM_GLOBAL_INIT;
        break;
    case '1':
        global_fsm.value = FSM_GLOBAL_PREOP;
        break;
    case '2':
        global_fsm.value = FSM_GLOBAL_SAFEOP;
        break;
    case '3':
        global_fsm.value = FSM_GLOBAL_OP_3;
        break;
    case '4':
        global_fsm.value = FSM_GLOBAL_OP_4;
        break;
    case '5':
        global_fsm.value = FSM_GLOBAL_OP_5;
        break;
    case '6':
        global_fsm.value = FSM_GLOBAL_OP_2;
        break;
    }
}

void Master::callback_sub_encoder_meter(const std_msgs::msg::Float32::SharedPtr msg)
{
    fb_encoder_meter = msg->data;
}

void Master::callback_sub_joy(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    last_time_joy = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    target_velocity_joy_x = profile_max_velocity * msg->axes[3];
    target_velocity_joy_y = 0;
    target_velocity_joy_wz = profile_max_steering_rad * msg->axes[0];

    if (msg->buttons[0] > 0)
    {
        transmission_joy_master = TRANSMISSION_REVERSE;
    }
    else if (msg->buttons[1] > 0)
    {
        transmission_joy_master = TRANSMISSION_NEUTRAL;
    }
    else if (msg->buttons[2] > 0)
    {
        transmission_joy_master = TRANSMISSION_AUTO;
    }
    else if (msg->buttons[3] > 0)
    {
        transmission_joy_master = TRANSMISSION_FORWARD;
    }
}

void Master::callback_sub_CAN_eps_encoder(const std_msgs::msg::Float32::SharedPtr msg)
{
    last_time_CANbus = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    fb_steering_angle = msg->data;
}
void Master::callback_sub_beckhoff_sensor(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    last_time_beckhoff = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    (void)msg;
}

void Master::callback_sub_obs_find(const std_msgs::msg::Float32::SharedPtr msg)
{
    last_time_obstacle_filter = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    obs_find = msg->data;
}

void Master::callback_sub_lane_kiri(const ros2_interface::msg::PointArray::SharedPtr msg)
{
    lane_kiri = *msg;
}

void Master::callback_sub_lane_tengah(const ros2_interface::msg::PointArray::SharedPtr msg)
{
    lane_tengah = *msg;
}

void Master::callback_sub_lane_kanan(const ros2_interface::msg::PointArray::SharedPtr msg)
{
    lane_kanan = *msg;
}

void Master::callback_sub_icp_pose_estimate(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    tf2::Quaternion q;
    double roll, pitch, yaw;
    tf2::fromMsg(msg->pose.pose.orientation, q);
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    dx_icp = msg->twist.twist.linear.x;
    dy_icp = msg->twist.twist.linear.y;
    dth_icp = msg->twist.twist.angular.z;

    while (dth_icp > M_PI)
        dth_icp -= 2 * M_PI;
    while (dth_icp < -M_PI)
        dth_icp += 2 * M_PI;

    icp_mag = sqrtf(dx_icp * dx_icp + dy_icp * dy_icp);

    static const float new_pose_gain = 0.07;

    fb_filtered_final_pose_xyo[0] = fb_final_pose_xyo[0] * (1 - new_pose_gain) + (fb_final_pose_xyo[0] + dx_icp) * new_pose_gain;
    fb_filtered_final_pose_xyo[1] = fb_final_pose_xyo[1] * (1 - new_pose_gain) + (fb_final_pose_xyo[1] + dy_icp) * new_pose_gain;
    fb_filtered_final_pose_xyo[2] = fb_final_pose_xyo[2] * (1 - new_pose_gain) + (fb_final_pose_xyo[2] + dth_icp) * new_pose_gain;

    while (fb_filtered_final_pose_xyo[2] > M_PI)
        fb_filtered_final_pose_xyo[2] -= M_PI * 2;
    while (fb_filtered_final_pose_xyo[2] < -M_PI)
        fb_filtered_final_pose_xyo[2] += M_PI * 2;
}
void Master::callback_sub_odometry(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    last_time_pose_estimator = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    tf2::Quaternion q;
    double roll, pitch, yaw;
    tf2::fromMsg(msg->pose.pose.orientation, q);
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    static float prev_msg_x = 0;
    static float prev_msg_y = 0;
    static float prev_msg_theta = 0;

    fb_final_pose_xyo[0] = msg->pose.pose.position.x;
    fb_final_pose_xyo[1] = msg->pose.pose.position.y;
    fb_final_pose_xyo[2] = yaw;

    // float distance = pythagoras(prev_msg_x, prev_msg_y, msg->pose.pose.position.x, msg->pose.pose.position.y);

    // float normalized_distance = (distance - complementary_terms[0]) / (complementary_terms[1] - complementary_terms[0]);
    // if (normalized_distance < 0)
    //     normalized_distance = 0;
    // else if (normalized_distance > 1)
    //     normalized_distance = 1;

    // float new_pose_gain = normalized_distance * (complementary_terms[2] - complementary_terms[1]) + complementary_terms[1];

    // fb_filtered_final_pose_xyo[0] = fb_filtered_final_pose_xyo[0] * (1 - new_pose_gain) + msg->pose.pose.position.x * new_pose_gain;
    // fb_filtered_final_pose_xyo[1] = fb_filtered_final_pose_xyo[1] * (1 - new_pose_gain) + msg->pose.pose.position.y * new_pose_gain;
    // fb_filtered_final_pose_xyo[2] = fb_filtered_final_pose_xyo[2] * (1 - new_pose_gain) + yaw * new_pose_gain;

    // while (fb_filtered_final_pose_xyo[2] > M_PI)
    //     fb_filtered_final_pose_xyo[2] -= M_PI * 2;
    // while (fb_filtered_final_pose_xyo[2] < -M_PI)
    //     fb_filtered_final_pose_xyo[2] += M_PI * 2;

    // prev_msg_x = fb_filtered_final_pose_xyo[0];
    // prev_msg_y = fb_filtered_final_pose_xyo[1];
    // prev_msg_theta = fb_filtered_final_pose_xyo[2];

    // if (transform_map2odom)
    // {
    //     tf2::Transform tf_final_pose;
    //     tf_final_pose.setOrigin(tf2::Vector3(fb_final_pose_xyo[0], fb_final_pose_xyo[1], 0));
    //     tf_final_pose.setRotation(q);
    //     tf2::Transform tf_transformed = manual_map2odom_tf * tf_final_pose;

    //     fb_final_pose_xyo[0] = tf_transformed.getOrigin().getX();
    //     fb_final_pose_xyo[1] = tf_transformed.getOrigin().getY();

    //     tf2::Matrix3x3 m(tf_transformed.getRotation());
    //     m.getRPY(roll, pitch, yaw);
    //     fb_final_pose_xyo[2] = yaw;
    // }

    fb_final_vel_dxdydo[0] = msg->twist.twist.linear.x;
    fb_final_vel_dxdydo[1] = msg->twist.twist.linear.y;
    fb_final_vel_dxdydo[2] = msg->twist.twist.angular.z;
}

void Master::callback_sub_error_code_beckhoff(const std_msgs::msg::Int16::SharedPtr msg)
{
    error_code_beckhoff = msg->data;
}

void Master::callback_sub_error_code_pose_estimator(const std_msgs::msg::Int16::SharedPtr msg)
{
    error_code_pose_estimator = msg->data;
}

void Master::callback_sub_error_code_obstacle_filter(const std_msgs::msg::Int16::SharedPtr msg)
{
    error_code_obstacle_filter = msg->data;
}

void Master::callback_sub_error_code_can(const std_msgs::msg::Int16::SharedPtr msg)
{
    error_code_can = msg->data;
}

void Master::callback_tim_50hz()
{
    if (!marker.init(this->shared_from_this()))
    {
        rclcpp::shutdown();
    }
    process_marker();

    current_time = rclcpp::Clock(RCL_SYSTEM_TIME).now();

    switch (global_fsm.value)
    {
    case FSM_GLOBAL_INIT:
        if (global_fsm.prev_value == FSM_GLOBAL_RECORD_ROUTE)
        {
            process_save_waypoints();
        }
        else
        {
            process_load_waypoints();
        }
        process_load_terminals();
        global_fsm.value = FSM_GLOBAL_PREOP;
        break;

    /**
     * Pre-operation
     * Keadaan ini memastikan semua sistem tidak ada error
     */
    case FSM_GLOBAL_PREOP:
        if (error_code_beckhoff + error_code_pose_estimator + error_code_can == 0)
        {
            local_fsm.value = 0;
            global_fsm.value = FSM_GLOBAL_SAFEOP;
        }

        local_fsm.value = 0;

        if (fabs(target_velocity_joy_x) > 0.1 || fabs(target_velocity_joy_y) > 0.1 || fabs(target_velocity_joy_wz) > 0.1)
        {
            if ((fb_beckhoff_digital_input & IN_MASK_BUMPER) != IN_MASK_BUMPER)
                manual_motion(-profile_max_braking, target_velocity_joy_y, target_velocity_joy_wz);
            else
                manual_motion(target_velocity_joy_x, target_velocity_joy_y, target_velocity_joy_wz);
        }
        else
        {
            if ((fb_beckhoff_digital_input & IN_MASK_BUMPER) != IN_MASK_BUMPER)
                manual_motion(-profile_max_braking, target_velocity_joy_y, target_velocity_joy_wz);
            else
                manual_motion(-1, 0, 0);
        }
        break;

    /**
     * Safe operation
     * Memastikan semua data bisa diterima
     */
    case FSM_GLOBAL_SAFEOP:
        if (error_code_beckhoff + error_code_pose_estimator + error_code_can > 0)
        {
            global_fsm.value = FSM_GLOBAL_PREOP;
        }

        master_status_emergency &= ~STATUS_TOWING_CONNECTED;

        /* Untuk manual maju mundur */
        if ((fb_beckhoff_digital_input & IN_MANUAL_MAJU) == IN_MANUAL_MAJU || (fb_beckhoff_digital_input & IN_MANUAL_MUNDUR) == IN_MANUAL_MUNDUR)
        {
            if ((fb_beckhoff_digital_input & IN_MANUAL_MAJU) == IN_MANUAL_MAJU)
            {
                transmission_joy_master = TRANSMISSION_AUTO;
            }
            else if ((fb_beckhoff_digital_input & IN_MANUAL_MUNDUR) == IN_MANUAL_MUNDUR)
            {
                transmission_joy_master = TRANSMISSION_REVERSE;
            }
        }
        /* Handle transmisi dari hardware */
        else
        {
            if ((fb_beckhoff_digital_input & IN_TR_FORWARD) == IN_TR_FORWARD)
            {
                transmission_joy_master = TRANSMISSION_FORWARD;
            }
            else if ((fb_beckhoff_digital_input & IN_TR_REVERSE) == IN_TR_REVERSE)
            {
                transmission_joy_master = TRANSMISSION_REVERSE;
            }
            else if ((fb_beckhoff_digital_input & 0b011) == 0)
            {
                transmission_joy_master = TRANSMISSION_AUTO;
            }
        }

        local_fsm.value = 0;

        if (debug_motion)
        {
            logger.info("icp %.2f %.2f %.2f %.2f || %.2f %d", icp_score, dx_icp, dy_icp, dth_icp, threshold_icp_score, fb_beckhoff_digital_input);
        }

        if (icp_score < threshold_icp_score)
            slam_status |= 0b1000; // Set status SLAM ready
        else
            slam_status &= ~0b1000; // Clear status SLAM ready

        if (((fb_beckhoff_digital_input & IN_START_OP3) == IN_START_OP3) || ((fb_beckhoff_digital_input & IN_START_OP3_HANDLER) == IN_START_OP3_HANDLER))
        {
            rclcpp::Duration dt_pose_estimator = current_time - last_time_pose_estimator;
            rclcpp::Duration dt_beckhoff = current_time - last_time_beckhoff;
            rclcpp::Duration dt_CANbus = current_time - last_time_CANbus;
            rclcpp::Duration dt_kamera = current_time - last_time_kamera_pcl;

            // Jika sudah berhasil menerima semua data yang diperlukan
            if (dt_pose_estimator.seconds() < 1 && dt_beckhoff.seconds() < 1 && dt_CANbus.seconds() < 1 && is_rtabmap_ready && is_pose_corrected && ((fb_beckhoff_digital_input & IN_EPS_nFAULT) == IN_EPS_nFAULT) && icp_score < threshold_icp_score)
            {
                if (enable_obs_detection_camera)
                {
                    if (dt_kamera.seconds() < 1)
                    {
                        target_velocity_joy_x = 0;
                        target_velocity_joy_y = 0;
                        target_velocity_joy_wz = 0;
                        local_fsm.value = 0;
                        global_fsm.value = FSM_GLOBAL_OP_3;
                        time_start_operation = current_time;
                    }
                }
                else
                {
                    target_velocity_joy_x = 0;
                    target_velocity_joy_y = 0;
                    target_velocity_joy_wz = 0;
                    local_fsm.value = 0;
                    global_fsm.value = FSM_GLOBAL_OP_3;
                    time_start_operation = current_time;
                }
            }
        }
        else if ((fb_beckhoff_digital_input & IN_START_GAS_MANUAL) == IN_START_GAS_MANUAL)
        {
            global_fsm.value = FSM_GLOBAL_OP_5;
        }

        if (!prev_is_rtabmap_ready && is_rtabmap_ready)
        {
            global_fsm.value = FSM_GLOBAL_INIT;
        }

        if (fabs(target_velocity_joy_x) > 0.1 || fabs(target_velocity_joy_y) > 0.1 || fabs(target_velocity_joy_wz) > 0.1)
        {
            if ((fb_beckhoff_digital_input & IN_MASK_BUMPER) != IN_MASK_BUMPER)
                manual_motion(-profile_max_braking, target_velocity_joy_y, target_velocity_joy_wz);
            else
            {
                if ((fb_beckhoff_digital_input & IN_MANUAL_MAJU) == IN_MANUAL_MAJU || (fb_beckhoff_digital_input & IN_MANUAL_MUNDUR) == IN_MANUAL_MUNDUR)
                {
                    manual_motion(0.48, target_velocity_joy_y, target_velocity_joy_wz);
                }
                else
                {
                    manual_motion(target_velocity_joy_x, target_velocity_joy_y, target_velocity_joy_wz);
                }
            }
        }
        else
        {
            if ((fb_beckhoff_digital_input & IN_MASK_BUMPER) != IN_MASK_BUMPER)
                manual_motion(-profile_max_braking, target_velocity_joy_y, target_velocity_joy_wz);
            else
            {
                if ((fb_beckhoff_digital_input & IN_MANUAL_MAJU) == IN_MANUAL_MAJU || (fb_beckhoff_digital_input & IN_MANUAL_MUNDUR) == IN_MANUAL_MUNDUR)
                {
                    manual_motion(0.48, target_velocity_joy_y, target_velocity_joy_wz);
                }
                else
                {
                    manual_motion(-1, 0, 0);
                }
            }
        }
        break;

    /**
     * Global operation
     * Sistem beroperasi secara otomatis
     */
    case FSM_GLOBAL_OP_3:
        if (error_code_beckhoff + error_code_pose_estimator + error_code_can > 0)
        {
            global_fsm.value = FSM_GLOBAL_PREOP;
            break;
        }

        if (enable_obs_detection_camera)
        {
            rclcpp::Duration dt_kamera = current_time - last_time_kamera_pcl;
            if (dt_kamera.seconds() > 0.8)
            {
                global_fsm.value = FSM_GLOBAL_PREOP;
                break;
            }
        }

        if ((fb_beckhoff_digital_input & IN_SYSTEM_FULL_ENABLE) == 0)
        {
            global_fsm.value = FSM_GLOBAL_SAFEOP;
            break;
        }

        // if ((fb_beckhoff_digital_input & IN_STOP_OP3) == IN_STOP_OP3)
        // {
        //     global_fsm.value = FSM_GLOBAL_SAFEOP;
        //     break;
        // }

        if ((fb_beckhoff_digital_input & IN_STOP_OP3_HANDLER) == IN_STOP_OP3_HANDLER)
        {
            global_fsm.value = FSM_GLOBAL_SAFEOP;
            break;
        }

        master_status_emergency |= STATUS_TOWING_CONNECTED;

        process_local_fsm();
        break;

    /**
     * Global operation (Untuk testing)
     * Sistem beroperasi secara otomatis, tetapi untuk steeringnya bisa dikontrol melalui joystick
     */
    case FSM_GLOBAL_OP_4:
        local_fsm.value = 0;

        if (metode_following == 0)
        {
            follow_waypoints_steer_manual(0, 0, target_velocity_joy_wz, 1, false);
        }
        else if (metode_following == 1)
        {
            follow_lane_steer_manual(0, 0, target_velocity_joy_wz);
        }
        else if (metode_following == 2)
        {
            fusion_follow_lane_waypoints_steer_manual(0, 0, target_velocity_joy_wz, 1, false);
        }

        break;

    /**
     * Global operation (Untuk testing)
     * Sistem beroperasi secara otomatis, tetapi untuk gasnya bisa dikontrol melalui joystick
     */
    case FSM_GLOBAL_OP_5:
        local_fsm.value = 0;

        if (metode_following == 0)
        {
            follow_waypoints_gas_manual(target_velocity_joy_x, 0, 0, 1.5, true);
        }
        else if (metode_following == 1)
        {
            follow_lane_gas_manual(target_velocity_joy_x, 0, 0);
        }
        else if (metode_following == 2)
        {
            fusion_follow_lane_waypoints_gas_manual(target_velocity_joy_x, 0, 0, 1.5, true);
        }

        if ((fb_beckhoff_digital_input & IN_STOP_OP3) == IN_STOP_OP3)
        {
            global_fsm.value = FSM_GLOBAL_SAFEOP;
            break;
        }

        break;

    /**
     * Global operation (Untuk testing)
     * Sistem beroperasi secara manual bisa dikontrol melalui joystick
     */
    case FSM_GLOBAL_OP_2:
        local_fsm.value = 0;
        manual_motion(target_velocity_joy_x, target_velocity_joy_y, target_velocity_joy_wz);
        break;

    case FSM_GLOBAL_RECORD_ROUTE:
        local_fsm.value = 0;
        process_record_route();
        break;
    }

    process_transmitter();

    global_fsm.prev_value = global_fsm.value;
    local_fsm.prev_value = local_fsm.value;
    prev_is_rtabmap_ready = is_rtabmap_ready;
}

// ================================================================================================

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node_master = std::make_shared<Master>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node_master);
    executor.spin();

    return 0;
}