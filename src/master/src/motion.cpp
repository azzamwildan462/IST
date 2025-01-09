#include "master/master.hpp"

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
        // logger.info("Brake");
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
        // logger.info("Accelerate after brake");
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
        // logger.info("Normal acceleration");
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
        // logger.info("Normal deceleration");
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
    static const float min_steering_rate = 7 * M_PI / 180.0;  // 7 deg/s
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
        actuation_wz = wz_buffer;
    }
    else
    {
        actuation_vx = pid_vx.calculate(vx_buffer - fb_encoder_meter);
        // actuation_vx = vx_buffer; // Sementara untuk testing
        actuation_ay = 0;
        actuation_wz = wz_buffer;
    }

    target_velocity = fmaxf(0, vx_buffer);

    // logger.info("%.2f %.2f || %.2f %.2f || %.2f %.2f || %.2f %.2f", vx, wz, actuation_ax, steering_rate, vx_buffer, wz_buffer, actuation_vx, actuation_wz);
}

void Master::follow_lane_2_cam(float vx, float vy, float wz)
{
    static float target_max_velocity = 0;

    (void)vy;
    (void)wz;

    /**
     * Panic state!
     * Ketika tidak atau kurang mendeteksi garis
     */
    if (lane_kiri_single_cam.points.size() < 4 && lane_kanan_single_cam.points.size() < 4)
    {
        manual_motion(-profile_max_braking, 0, 0);
        return;
    }

    float out_steer = cam_kiri_pid_output * 0.5 + cam_kanan_pid_output * 0.5;
    float velocity_gain = cam_kiri_velocity_gain * 0.5 + cam_kanan_velocity_gain * 0.5;

    target_max_velocity = target_max_velocity * 0.2 + fmaxf(vx, profile_max_velocity) * velocity_gain * 0.8;

    /**
     * Menghitung efek obstacle
     * Semakin besar emergency, semakin cepat robot untuk berhenti
     */
    float obstacle_emergency = obstacle_influence(2);
    if (obstacle_emergency > 0.2)
    {
        // logger.info("Emergency brake ada obstacle %.2f", obstacle_emergency);
        manual_motion(-profile_max_braking * obstacle_emergency, 0, 0);
        return;
    }

    manual_motion(target_max_velocity, 0, out_steer);
}

void Master::follow_lane_2_cam_gas_manual(float vx, float vy, float wz)
{
    static float target_max_velocity = 0;

    (void)vy;
    (void)wz;

    /**
     * Panic state!
     * Ketika tidak atau kurang mendeteksi garis
     */
    if (lane_kiri_single_cam.points.size() < 4 && lane_kanan_single_cam.points.size() < 4)
    {
        manual_motion(-profile_max_braking, 0, 0);
        return;
    }

    float out_steer = cam_kiri_pid_output * 0.5 + cam_kanan_pid_output * 0.5;

    manual_motion(vx, 0, out_steer);
}

void Master::follow_lane_2_cam_steer_manual(float vx, float vy, float wz)
{
    static float target_max_velocity = 0;

    (void)vy;

    /**
     * Panic state!
     * Ketika tidak atau kurang mendeteksi garis
     */
    if (lane_kiri_single_cam.points.size() < 4 && lane_kanan_single_cam.points.size() < 4)
    {
        manual_motion(-profile_max_braking, 0, 0);
        return;
    }

    float velocity_gain = cam_kiri_velocity_gain * 0.5 + cam_kanan_velocity_gain * 0.5;

    target_max_velocity = target_max_velocity * 0.2 + fmaxf(vx, profile_max_velocity) * velocity_gain * 0.8;

    /**
     * Menghitung efek obstacle
     * Semakin besar emergency, semakin cepat robot untuk berhenti
     */
    float obstacle_emergency = obstacle_influence(2);
    if (obstacle_emergency > 0.2)
    {
        // logger.info("Emergency brake ada obstacle %.2f", obstacle_emergency);
        manual_motion(-profile_max_braking * obstacle_emergency, 0, 0);
        return;
    }

    manual_motion(target_max_velocity, 0, wz);
}

void Master::follow_lane(float vx, float vy, float wz)
{
    static const float pixel_to_world = 0.053;
    static const float gain_kemiringan_terhadap_steering = 0.8;
    static const float steer2lane_sf = 0.99; // Smooth factor
    static float target_steering_angle = 0;  // Ini adalah arah hadap roda
    static float target_max_velocity = 0;

    (void)vy;
    (void)wz;

    /**
     * Panic state!
     * Ketika tidak atau kurang mendeteksi garis
     */
    if (lane_tengah.points.size() < 4)
    {
        target_steering_angle += target_steering_angle * (1 - steer2lane_sf) + 0 * steer2lane_sf;
        manual_motion(-profile_max_braking, 0, 0);
        return;
    }

    /**
     * Mencari target max velocity
     */
    float min_y_point = FLT_MAX;
    float max_y_point = -FLT_MAX;
    for (size_t i = 0; i < lane_tengah.points.size(); i++)
    {
        if (lane_tengah.points[i].y < min_y_point)
        {
            min_y_point = lane_tengah.points[i].y;
        }

        if (lane_tengah.points[i].y > max_y_point)
        {
            max_y_point = lane_tengah.points[i].y;
        }
    }

    target_max_velocity = fabs(max_y_point - min_y_point) * pixel_to_world;

    if (target_max_velocity > fmaxf(profile_max_velocity, vx))
    {
        target_max_velocity = fmaxf(profile_max_velocity, vx);
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

    target_steering_angle += target_steering_angle * steer2lane_sf + kemiringan_lane * (1 - steer2lane_sf);

    if (kemiringan_lane > 0 && target_steering_angle > kemiringan_lane)
    {
        target_steering_angle = kemiringan_lane;
    }
    else if (kemiringan_lane < 0 && target_steering_angle < kemiringan_lane)
    {
        target_steering_angle = kemiringan_lane;
    }

    /**
     * Menghitung efek obstacle
     * Semakin besar emergency, semakin cepat robot untuk berhenti
     */
    float obstacle_emergency = obstacle_influence(0.5);
    if (obstacle_emergency > 0.2)
    {
        target_max_velocity = fmaxf(0, target_max_velocity - obstacle_emergency);
    }

    manual_motion(target_max_velocity, 0, target_steering_angle);
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