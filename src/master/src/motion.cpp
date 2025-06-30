#include "master/master.hpp"

float Master::pythagoras(float x1, float y1, float x2, float y2)
{
    return sqrtf((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

void Master::manual_motion(float vx, float vy, float wz)
{
    static float vx_buffer = 0; // Throttle velocity
    static float wz_buffer = 0; // Steering angle

    (void)vy;

    /**
     * Menghitung kecepatan mobil
     * Braking system aktif ketika vx < 0, sisanya kontrol kecepatan
     */

    /**
     * Ketika brake
     */
    static uint8_t state_control = 0;
    static uint8_t prev_state_control = 0;
    if (vx < 0)
    {
        state_control = 0;
        if (prev_state_control != 0)
            actuation_ax = 0;

        actuation_ax += -profile_max_braking_jerk * 0.5 * dt * dt;
        if (actuation_ax < -profile_max_braking_acceleration)
        {
            actuation_ax = -profile_max_braking_acceleration;
        }
        vx_buffer += actuation_ax * dt;
        if (vx_buffer < vx)
        {
            vx_buffer = vx;
        }
    }
    /**
     * Accelerate setelah braking, segera lepas pedal brake
     */
    else if (vx > 0 && vx_buffer < 0)
    {
        state_control = 1;
        if (prev_state_control != 1)
            actuation_ax = 0;

        actuation_ax += profile_max_braking_jerk * 0.5 * dt * dt;
        if (actuation_ax > profile_max_braking_acceleration)
        {
            actuation_ax = profile_max_braking_acceleration;
        }
        vx_buffer += actuation_ax * dt;
        if (vx_buffer > vx)
        {
            vx_buffer = vx;
        }
    }
    /**
     * Normal acceleration
     */
    else if (vx > vx_buffer)
    {
        state_control = 2;
        if (prev_state_control != 2)
            actuation_ax = 0;

        actuation_ax += profile_max_accelerate_jerk * 0.5 * dt * dt;
        if (actuation_ax > profile_max_acceleration)
        {
            actuation_ax = profile_max_acceleration;
        }
        vx_buffer += actuation_ax * dt;
        if (vx_buffer > vx)
        {
            vx_buffer = vx;
        }
    }
    /**
     * Normal deceleration
     */
    else if (vx < vx_buffer)
    {
        state_control = 3;
        if (prev_state_control != 3)
            actuation_ax = 0;

        actuation_ax -= profile_max_decelerate_jerk * 0.5 * dt * dt;
        if (actuation_ax < -profile_max_decceleration)
        {
            actuation_ax = -profile_max_decceleration;
        }
        vx_buffer += actuation_ax * dt;
        if (vx_buffer < vx)
        {
            vx_buffer = vx;
        }
    }
    prev_state_control = state_control;

    /**
     * Menghitung kecepatan steer berdasarkan kecepatan mobil
     * Semakin cepat mobil, semakin lambat perputaran steer
     */
    static const float min_velocity = 2.0 / 3.6;              // 2 km/h
    static const float max_velocity = 7.0 / 3.6;              // 7 km/h
    static const float min_steering_rate = 14 * M_PI / 180.0; // 7 deg/s
    static const float max_steering_rate = 36 * M_PI / 180.0; // 21 deg/s
    static const float gradient_steering_rate = (min_steering_rate - max_steering_rate) / (max_velocity - min_velocity);

    float steering_rate = fmaxf(min_steering_rate,
                                fminf(max_steering_rate,
                                      gradient_steering_rate * (fb_encoder_meter - min_velocity) + max_steering_rate));

    if (wz > wz_buffer)
    {
        wz_buffer += steering_rate * dt;
        if (wz_buffer > wz)
        {
            wz_buffer = wz;
        }
    }
    else if (wz < wz_buffer)
    {
        wz_buffer -= steering_rate * dt;
        if (wz_buffer < wz)
        {
            wz_buffer = wz;
        }
    }

    if (vx_buffer < 0)
    {
        actuation_vx = vx_buffer;
        actuation_ay = 0;
        actuation_wz = offset_sudut_steering + wz_buffer;
    }
    else
    {
        actuation_vx = pid_vx.calculate(vx_buffer - fb_encoder_meter);
        actuation_ay = 0;
        actuation_wz = offset_sudut_steering + wz_buffer;
    }

    target_velocity = fmaxf(0, vx_buffer);

    // logger.info("%.2f %.2f || %.2f %.2f || %.2f %.2f || %.2f %.2f", vx, wz, actuation_ax, steering_rate, vx_buffer, wz_buffer, actuation_vx, actuation_wz);
}

void Master::lane2velocity_steering(float *pvelocity, float *psteering, float *pconfidence)
{
    static const float gain_kemiringan_terhadap_steering = 0.8;
    static float target_steering_angle = 0; // Ini adalah arah hadap roda
    static float target_max_velocity = 0;
    static const float lookahead_distance = 1.5;
    static const float max_error_steering_mean = M_PI_2;

    /**
     * Panic state!
     * Ketika tidak atau kurang mendeteksi garis
     */
    if (lane_tengah.points.size() < 4)
    {
        *pvelocity = -1;
        *psteering = 0;
        *pconfidence = 0;
        return;
    }

    /**
     * Mencari target max velocity
     * dan menghitung rata rata kemiringan lane
     */
    float kemiringan_lane_buffer = 0;
    uint16_t kemiringan_lane_count = 0;
    uint16_t it_counter = 0;
    float prev_x = 0;
    float prev_y = 0;
    float buffer_y = lane_tengah.points[lane_tengah.points.size() - 1].y;
    for (size_t i = 0; i < lane_tengah.points.size(); i++)
    {
        if (it_counter++ > 5)
        {
            float dx = prev_x - lane_tengah.points[i].x;
            float dy = (buffer_y - prev_y) - (buffer_y - lane_tengah.points[i].y);
            kemiringan_lane_buffer += (atan2f(dy, dx) - 1.57);
            kemiringan_lane_count++;
            prev_x = lane_tengah.points[i].x;
            prev_y = lane_tengah.points[i].y;
            it_counter = 0;
        }
    }

    /**
     * Menghitung steering angle
     */
    float point_x_terdekat = lane_tengah.points[lane_tengah.points.size() - 1].x;
    float point_y_terdekat = lane_tengah.points[lane_tengah.points.size() - 1].y - lane_tengah.points[lane_tengah.points.size() - 1].y;
    float point_x_terjauh = lane_tengah.points[0].x;
    float point_y_terjauh = lane_tengah.points[lane_tengah.points.size() - 1].y - lane_tengah.points[0].y;

    float dx = point_x_terjauh - point_x_terdekat;
    float dy = point_y_terjauh - point_y_terdekat;
    float kemiringan_lane = (atan2f(dy, dx) - 1.57) * gain_kemiringan_terhadap_steering;

    target_steering_angle = atan2(2 * wheelbase * sinf(kemiringan_lane), lookahead_distance);

    if (target_steering_angle < -1.2)
        target_steering_angle = -1.2;
    else if (target_steering_angle > 1.2)
        target_steering_angle = 1.2;

    target_max_velocity = profile_max_velocity * (1 - lookahead_distance * fabs(target_steering_angle));

    if (target_max_velocity > FLT_EPSILON && target_max_velocity < 0.48)
        target_max_velocity = 0.48;
    else if (target_max_velocity > FLT_EPSILON && target_max_velocity < 1.2)
        target_max_velocity = 1.2;

    if (target_max_velocity > profile_max_velocity)
    {
        target_max_velocity = profile_max_velocity;
    }

    /**
     * Menghitung confidence
     */
    float error_steering_mean = 0;
    float steering_mean = 0;
    if (kemiringan_lane_count > 0)
    {
        steering_mean = kemiringan_lane_buffer / kemiringan_lane_count;
    }
    error_steering_mean = steering_mean - kemiringan_lane;

    while (error_steering_mean < -M_PI)
        error_steering_mean += 2 * M_PI;
    while (error_steering_mean > M_PI)
        error_steering_mean -= 2 * M_PI;

    float confidence = 1 - fabs(error_steering_mean) / max_error_steering_mean;
    if (confidence < 0)
        confidence = 0;
    else if (confidence > 0.8)
        confidence = 1;

    /**
     * Pointer assignment
     */
    *pvelocity = target_max_velocity;
    *psteering = target_steering_angle;
    *pconfidence = confidence;
}

void Master::wp2velocity_steering(float lookahead_distance, float *pvelocity, float *psteering, bool is_loop)
{
    if (waypoints.size() == 0)
    {
        *pvelocity = -1;
        *psteering = 0;
        return;
    }

    static uint32_t index_sekarang = 0;
    static uint32_t index_lookahead = 0;
    float obs_scan_r = 2;
    static const float threshold_error_sangat_besar = 1.5;
    static const float thresholad_error_kecil = 0.4;
    static const int16_t error_kecil_window_index_search = 5;
    static const float offset_fb_velocity = 3;
    static const float threshold_error_arah_hadap_waypoint = 1.57;
    static const float threshold_error_arah_hadap_terminal = 1.37;
    static int16_t terminal_now = 0;
    static int16_t prev_terminal = 0;

    float pose_used_x = fb_final_pose_xyo[0];
    float pose_used_y = fb_final_pose_xyo[1];
    if (use_filtered_pose)
    {
        pose_used_x = fb_filtered_final_pose_xyo[0];
        pose_used_y = fb_filtered_final_pose_xyo[1];
    }

    /* Mencari index sekarang pada waypoints */
    float error_index_sekarang = pythagoras(pose_used_x, pose_used_y, waypoints[index_sekarang].x, waypoints[index_sekarang].y);
    if (error_index_sekarang > threshold_error_sangat_besar)
    {
        float min_error = FLT_MAX;
        uint32_t index_terdekat = 0;
        for (size_t i = 0; i < waypoints.size(); i++)
        {
            float error_arah_hadap = waypoints[i].arah - fb_final_pose_xyo[2];
            while (error_arah_hadap > M_PI)
                error_arah_hadap -= 2 * M_PI;
            while (error_arah_hadap < -M_PI)
                error_arah_hadap += 2 * M_PI;

            if (fabsf(error_arah_hadap) < threshold_error_arah_hadap_waypoint)
            {
                float error = pythagoras(pose_used_x, pose_used_y, waypoints[i].x, waypoints[i].y);
                if (error < min_error)
                {
                    min_error = error;
                    index_terdekat = i;
                }
            }
        }
        index_sekarang = index_terdekat;
    }
    else if (error_index_sekarang > thresholad_error_kecil)
    {
        float min_error = FLT_MAX;
        uint32_t index_terdekat = 0;
        for (size_t i = index_sekarang - error_kecil_window_index_search; i < index_sekarang + error_kecil_window_index_search * 2; i++)
        {
            float error_arah_hadap = waypoints[i].arah - fb_final_pose_xyo[2];
            while (error_arah_hadap > M_PI)
                error_arah_hadap -= 2 * M_PI;
            while (error_arah_hadap < -M_PI)
                error_arah_hadap += 2 * M_PI;

            if (fabsf(error_arah_hadap) < threshold_error_arah_hadap_waypoint)
            {
                float error = pythagoras(pose_used_x, pose_used_y, waypoints[i].x, waypoints[i].y);
                if (error < min_error)
                {
                    min_error = error;
                    index_terdekat = i;
                }
            }
        }
        index_sekarang = index_terdekat;
    }

    /* Menentukan efek terminal */
    static float max_velocity_terminal = profile_max_velocity;
    static float lookahead_distance_terminal = lookahead_distance;
    static float obs_scan_r_terminal = obs_scan_r;
    static float obs_scan_camera_thr_terminal = 100000;

    for (size_t i = 0; i < terminals.terminals.size(); i++)
    {
        float error_arah_hadap = terminals.terminals[i].target_pose_theta - fb_final_pose_xyo[2];
        while (error_arah_hadap > M_PI)
            error_arah_hadap -= 2 * M_PI;
        while (error_arah_hadap < -M_PI)
            error_arah_hadap += 2 * M_PI;

        if (fabsf(error_arah_hadap) < threshold_error_arah_hadap_terminal)
        {
            float jarak_robot_terminal = pythagoras(pose_used_x, pose_used_y, terminals.terminals[i].target_pose_x, terminals.terminals[i].target_pose_y);
            if (jarak_robot_terminal < terminals.terminals[i].radius_area)
            {
                max_velocity_terminal = terminals.terminals[i].target_max_velocity_x;
                lookahead_distance_terminal = terminals.terminals[i].target_lookahead_distance;
                obs_scan_r_terminal = terminals.terminals[i].obs_scan_r;

                prev_terminal = terminal_now;
                terminal_now = i;
                obs_scan_camera_thr_terminal = terminals.terminals[i].obs_threshold;
                break;
            }
        }
    }

    /* Jika ganti terminal, update parameter */
    if (prev_terminal != terminal_now)
    {
        camera_scan_min_x_ = terminals.terminals[terminal_now].scan_min_x;
        camera_scan_max_x_ = terminals.terminals[terminal_now].scan_max_x;
        camera_scan_min_y_ = terminals.terminals[terminal_now].scan_min_y;
        camera_scan_max_y_ = terminals.terminals[terminal_now].scan_max_y;
    }

    /* Filter lookahed_distance agar tidak langsung berubah */
    // filtered_lookahead_distance = filtered_lookahead_distance * 0.2 + lookahead_distance_terminal * 0.8;

    /* Agar tidak merubah kebawahnya */
    lookahead_distance = lookahead_distance_terminal;
    obs_scan_r = obs_scan_r_terminal;
    lidar_obs_scan_thr = obs_scan_r;

    /* Mencari waypoint sesuai lookahead_distance */
    index_lookahead = index_sekarang;
    if (!is_loop)
    {
        for (size_t i = index_sekarang; i < waypoints.size(); i++)
        {
            float error = pythagoras(pose_used_x, pose_used_y, waypoints[i].x, waypoints[i].y);
            if (error > lookahead_distance)
            {
                index_lookahead = i;
                break;
            }
        }
    }
    else
    {
        for (size_t i = index_sekarang; i < waypoints.size() * 2; i++)
        {
            size_t index_used = i;

            if (i >= waypoints.size())
            {
                index_used -= waypoints.size();
            }

            float error = pythagoras(pose_used_x, pose_used_y, waypoints[index_used].x, waypoints[index_used].y);
            if (error > lookahead_distance)
            {
                index_lookahead = index_used;
                break;
            }
        }
    }

    if (debug_motion)
    {
        logger.info("Pose: %.2f %.2f %.2f || idx %d %d || term %d %.2f %.2f %d || %.2f %.2f || %.2f %.2f", pose_used_x, pose_used_y, fb_final_pose_xyo[2], index_sekarang, index_lookahead, terminal_now, lookahead_distance, obs_scan_r, lidar_obs_scan_thr, obs_scan_camera_thr_terminal, waypoints[index_sekarang].x, waypoints[index_sekarang].y, waypoints[index_lookahead].x, waypoints[index_lookahead].y);
    }

    /* Jika sudah mencapai waypoint terakhir */
    if (index_lookahead == waypoints.size() - 1)
    {
        if (is_loop)
        {
            index_lookahead = 0;
        }
        else
        {
            *pvelocity = -1;
            *psteering = 0;
            return;
        }
    }

    /* Safety tambahan */
    if (index_lookahead == index_sekarang)
    {
        *pvelocity = -1;
        *psteering = 0;
        return;
    }

    /* Safety ketika update lookahed_distance */
    // if (fabsf(lookahead_distance_terminal - prev_lookahead_distance_terminal) > 0.1)
    // {
    //     if (counter_tahan_steer_sebelumnya < 10)
    //     {
    //         counter_tahan_steer_sebelumnya++;
    //         *pvelocity = prev_velocity;
    //         *psteering = prev_steering;
    //         return;
    //     }
    //     else
    //     {
    //         counter_tahan_steer_sebelumnya = 0;
    //     }
    // }
    // else
    // {
    //     counter_tahan_steer_sebelumnya = 0;
    // }

    /* Menghitung target velocity */
    float dx = waypoints[index_lookahead].x - pose_used_x;
    float dy = waypoints[index_lookahead].y - pose_used_y;
    float target_velocity = waypoints[index_lookahead].fb_velocity + offset_fb_velocity;

    /* Jika diatas 0 maka dibuat minimal 0.48 */
    if (target_velocity > FLT_EPSILON && target_velocity < 0.48)
        target_velocity = fmaxf(0.48, target_velocity);

    if (target_velocity > max_velocity_terminal)
        target_velocity = max_velocity_terminal;

    /* Menghitung target steering angle */
    float direction = atan2(dy, dx) - fb_final_pose_xyo[2];
    float target_steering_angle = atan2(2 * wheelbase * sinf(direction), lookahead_distance);
    while (target_steering_angle > M_PI)
        target_steering_angle -= 2 * M_PI;
    while (target_steering_angle < -M_PI)
        target_steering_angle += 2 * M_PI;

    /* Menghitung expected steering angle (jika robot selalu berada di waypoints) */
    // float expected_dy = waypoints[index_lookahead].y - waypoints[index_sekarang].y;
    // float expected_dx = waypoints[index_lookahead].x - waypoints[index_sekarang].x;
    // float expected_direction = atan2(expected_dy, expected_dx) - fb_final_pose_xyo[2];
    // float expected_steering_angle = atan2(2 * wheelbase * sinf(expected_direction), lookahead_distance);
    // while (expected_steering_angle > M_PI)
    //     expected_steering_angle -= 2 * M_PI;
    // while (expected_steering_angle < -M_PI)
    //     expected_steering_angle += 2 * M_PI;

    // /* Menggunakan average weighting untuk menggabungkan hasil steering angle */
    // target_steering_angle = target_steering_angle * 0.7 + expected_steering_angle * 0.3;
    // while (target_steering_angle > M_PI)
    //     target_steering_angle -= 2 * M_PI;
    // while (target_steering_angle < -M_PI)
    //     target_steering_angle += 2 * M_PI;

    // logger.info("%.2f %.2f || %.2f %.2f", pose_used_x, pose_used_y, target_velocity, target_steering_angle);

    if (debug_motion)
    {
        logger.info("vstr %.2f %.2f || obs %.2f %d", target_velocity, target_steering_angle, obs_find_baru, camera_scan_obs_result);
    }

    /* Menghitung obstacle */
    if (enable_obs_detection)
    {
        if (obs_find_baru > 0.05)
        {
            target_velocity = fmaxf(-profile_max_braking, target_velocity - obs_find_baru);

            if (target_velocity > FLT_EPSILON && target_velocity < 0.48)
                target_velocity = -1;

            if (target_velocity > max_velocity_terminal)
                target_velocity = max_velocity_terminal;
        }

        // float obs_find_filtered = obstacle_influence(40.0);
        // if (obs_find_filtered > 0.05)
        // {
        //     target_velocity = fmaxf(-profile_max_braking, target_velocity - obs_find_filtered * 50);

        //     if (target_velocity > FLT_EPSILON && target_velocity < 0.48)
        //         target_velocity = -1;

        //     if (target_velocity > max_velocity_terminal)
        //         target_velocity = max_velocity_terminal;
        // }
    }

    if (enable_obs_detection_camera)
    {
        if (camera_scan_obs_result > obs_scan_camera_thr_terminal)
        {
            target_velocity = fmaxf(-profile_max_braking, target_velocity - (camera_scan_obs_result - obs_scan_camera_thr_terminal) * 0.01);

            if (target_velocity > FLT_EPSILON && target_velocity < 0.48)
                target_velocity = -1;

            if (target_velocity > max_velocity_terminal)
                target_velocity = max_velocity_terminal;
        }
    }

    *pvelocity = target_velocity;
    *psteering = target_steering_angle;
}

void Master::follow_lane(float vx, float vy, float wz)
{
    (void)vx;
    (void)vy;
    (void)wz;

    float target_velocity = -1;
    float target_steering_angle = 0;
    float confidence = 0;
    lane2velocity_steering(&target_velocity, &target_steering_angle, &confidence);

    /* Menghitung obstacle */
    if (enable_obs_detection)
    {
        float obstacle_emergency = obstacle_influence(0.25);
        if (obstacle_emergency > 0.2)
        {
            target_velocity = fmaxf(-profile_max_braking, target_velocity - obstacle_emergency);
        }
    }

    manual_motion(target_velocity, 0, target_steering_angle);
}

void Master::follow_lane_gas_manual(float vx, float vy, float wz)
{
    (void)vy;
    (void)wz;

    float target_velocity = -1;
    float target_steering_angle = 0;
    float confidence = 0;
    lane2velocity_steering(&target_velocity, &target_steering_angle, &confidence);

    manual_motion(vx, 0, target_steering_angle);
}

void Master::follow_lane_steer_manual(float vx, float vy, float wz)
{
    (void)vx;
    (void)vy;

    float target_velocity = -1;
    float target_steering_angle = 0;
    float confidence = 0;
    lane2velocity_steering(&target_velocity, &target_steering_angle, &confidence);

    manual_motion(target_velocity, 0, wz);
}

void Master::follow_waypoints(float vx, float vy, float wz, float lookahead_distance, bool is_loop)
{
    (void)vx;
    (void)vy;
    (void)wz;

    float target_velocity = -1;
    float target_steering_angle = 0;
    wp2velocity_steering(lookahead_distance, &target_velocity, &target_steering_angle, is_loop);

    if ((fb_beckhoff_digital_input & IN_MASK_BUMPER) != IN_MASK_BUMPER)
        manual_motion(-profile_max_braking, target_velocity_joy_y, target_velocity_joy_wz);
    else
        manual_motion(target_velocity, 0, target_steering_angle);
}

void Master::follow_waypoints_gas_manual(float vx, float vy, float wz, float lookahead_distance, bool is_loop)
{
    (void)vy;
    (void)wz;

    float target_velocity = -1;
    float target_steering_angle = 0;
    wp2velocity_steering(lookahead_distance, &target_velocity, &target_steering_angle, is_loop);

    manual_motion(vx, 0, target_steering_angle);
}
void Master::follow_waypoints_steer_manual(float vx, float vy, float wz, float lookahead_distance, bool is_loop)
{
    follow_waypoints(vx, vy, wz, lookahead_distance, is_loop);
}

void Master::fusion_follow_lane_waypoints(float vx, float vy, float wz, float lookahead_distance, bool is_loop)
{
    (void)vx;
    (void)vy;
    (void)wz;

    float target_velocity_follow_wp = -1;
    float target_steering_angle_follow_wp = 0;
    wp2velocity_steering(lookahead_distance, &target_velocity_follow_wp, &target_steering_angle_follow_wp, is_loop);

    float target_velocity_follow_lane = -1;
    float target_steering_angle_follow_lane = 0;
    float confidence_follow_lane = 0;
    lane2velocity_steering(&target_velocity_follow_lane, &target_steering_angle_follow_lane, &confidence_follow_lane);

    float target_velocity = target_velocity_follow_wp * 0.2 + target_velocity_follow_lane * 0.8;
    float target_steering_angle = target_steering_angle_follow_wp * (1 - confidence_follow_lane) + target_steering_angle_follow_lane * confidence_follow_lane;

    /* Menghitung obstacle */
    if (enable_obs_detection)
    {
        float obstacle_emergency = obstacle_influence(0.5);
        if (obstacle_emergency > 0.2)
        {
            target_velocity = fmaxf(-profile_max_braking, target_velocity - obstacle_emergency);
        }
    }

    manual_motion(target_velocity, 0, target_steering_angle);
}
void Master::fusion_follow_lane_waypoints_gas_manual(float vx, float vy, float wz, float lookahead_distance, bool is_loop)
{
    (void)vy;
    (void)wz;

    float target_velocity_follow_wp = -1;
    float target_steering_angle_follow_wp = 0;
    wp2velocity_steering(lookahead_distance, &target_velocity_follow_wp, &target_steering_angle_follow_wp, is_loop);

    float target_velocity_follow_lane = -1;
    float target_steering_angle_follow_lane = 0;
    float confidence_follow_lane = 0;
    lane2velocity_steering(&target_velocity_follow_lane, &target_steering_angle_follow_lane, &confidence_follow_lane);

    float target_steering_angle = target_steering_angle_follow_wp * (1 - confidence_follow_lane) + target_steering_angle_follow_lane * confidence_follow_lane;

    manual_motion(vx, 0, target_steering_angle);
}
void Master::fusion_follow_lane_waypoints_steer_manual(float vx, float vy, float wz, float lookahead_distance, bool is_loop)
{
    (void)vx;
    (void)vy;
    (void)wz;
    (void)lookahead_distance;
    (void)is_loop;
}

float Master::obstacle_influence(float gain)
{
    static float ret_buffer = 0;

    /**
     * Jika ada obstacle, maka efek obs_find semakin besar
     * Hal itu membuat robot untuk berhenti lebih cepat
     *
     * Jika obstacle menghilang, maka efek obs_find semakin kecil
     * Hal itu membuat robot untuk jalan lebih lama
     */
    if (obs_find > ret_buffer)
        ret_buffer = ret_buffer * 0.1 + obs_find / max_obs_find_value * gain * 0.9;
    else
        ret_buffer = ret_buffer * 0.85 + obs_find / max_obs_find_value * gain * 0.15;

    return ret_buffer;
}

float Master::local_obstacle_influence(float obs_scan_r, float gain)
{
    static float ret_buffer = 0;
    static float obs_find_local = 0;
    static const float max_obs_find_value_local = 100;
    static const float x_min = 0;
    static const float y_min = -0.5;
    // static const float x_max = 1;
    static const float y_max = 0.5;

    sensor_msgs::msg::LaserScan lidar_depan_points_local;

    mutex_lidar_depan_points.lock();
    lidar_depan_points_local = lidar_depan_points;
    mutex_lidar_depan_points.unlock();

    float obs_scan_r_decimal = 1 / obs_scan_r;
    obs_find_local = 0;
    for (size_t i = 0; i < lidar_depan_points_local.ranges.size(); ++i)
    {
        float range = lidar_depan_points_local.ranges[i];
        if (std::isfinite(range) && range >= lidar_depan_points_local.range_min && range <= lidar_depan_points_local.range_max)
        {
            float angle = lidar_depan_points_local.angle_min + i * lidar_depan_points_local.angle_increment;
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
                if (point_x > x_min && point_x < obs_scan_r && point_y > y_min && point_y < y_max)
                {
                    obs_find_local += obs_scan_r_decimal * (obs_scan_r - range);
                }
            }
        }
    }
    // logger.info("====== obs_find_local: %.2f (%.2f)", obs_find_local, ret_buffer);

    /**
     * Jika ada obstacle, maka efek obs_find_local semakin besar
     * Hal itu membuat robot untuk berhenti lebih cepat
     *
     * Jika obstacle menghilang, maka efek obs_find_local semakin kecil
     * Hal itu membuat robot untuk jalan lebih lama
     */
    if (obs_find_local > ret_buffer)
        ret_buffer = ret_buffer * 0.1 + obs_find_local / max_obs_find_value_local * gain * 0.9;
    else
        ret_buffer = ret_buffer * 0.85 + obs_find_local / max_obs_find_value_local * gain * 0.15;

    return ret_buffer;
}