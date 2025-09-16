#include "rclcpp/rclcpp.hpp"
#include "ros2_utils/global_definitions.hpp"
#include "ros2_utils/help_logger.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <stdint.h>
#include <unistd.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <termios.h>
#include "hardware/jhctech_291.h"

#include <thread>

#define COB_ID_CAR_ENCODER 0x388
#define COB_ID_CAR_BATTERY 0x109
#define COB_ID_CAR_FB_ACCELERATOR 0x101
#define COB_ID_CAR_FB_TRANSMISSION 0x18C
#define COB_ID_EPS_ACTUATION 0x321
#define COB_ID_EPS_ENCODER 0x231
#define COB_ID_GYRO_RION 0x585

#define EPS_ENCODER_MAX_COUNTER 10000
#define EPS_ENCODER_MAX_RAD 1.5708

class CANbus_HAL : public rclcpp::Node
{
public:
    rclcpp::TimerBase::SharedPtr tim_50hz;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_battery;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_encoder;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_fb_tps_accelerator;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_fb_transmission;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_fb_eps_mode;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_error_code;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_eps_encoder;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_can;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_gyro_counter;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_master_actuator;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_master_global_fsm;

    HelpLogger logger;
    int error_code = 0;

    // Configs
    // =======================================================
    std::string if_name;
    int bitrate = 125000;
    bool use_socket_can = true;
    bool can_to_car = false;
    int jhctech_can_id = -1; // Jika -1, masuk default, 1 untuk EPS, 2 untuk mobil
    int counter_divider_can_send = 4;
    int counter_divider_publish = 200;

    int socket_can = -1;

    int battery = 0;
    int16_t encoder = 0;
    uint8_t fb_tps_accelerator = 0;
    uint8_t fb_transmission = 0;
    float eps_actuation = 0;
    uint8_t eps_flag = 0;
    float eps_encoder_fb = 0;
    uint8_t eps_mode_fb = 0;
    int16_t master_global_fsm = 0;

    uint8_t epoch_encoder = 0;
    uint8_t prev_epoch_encoder = 0;

    uint8_t counter_gyro_update = 0;

    sensor_msgs::msg::Imu imu_msg;

    std::thread thread_routine;
    int counter_can_send = 0;

    CANbus_HAL()
        : Node("CANbus_HAL")
    {
        if (!logger.init())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize logger");
            rclcpp::shutdown();
        }

        logger.info("CANbus_HAL init");

        this->declare_parameter("if_name", "can0");
        this->get_parameter("if_name", if_name);

        this->declare_parameter("bitrate", 125000);
        this->get_parameter("bitrate", bitrate);

        this->declare_parameter("use_socket_can", true);
        this->get_parameter("use_socket_can", use_socket_can);

        this->declare_parameter("jhctech_can_id", -1);
        this->get_parameter("jhctech_can_id", jhctech_can_id);

        this->declare_parameter("can_to_car", false);
        this->get_parameter("can_to_car", can_to_car);

        this->declare_parameter("counter_divider_can_send", 4);
        this->get_parameter("counter_divider_can_send", counter_divider_can_send);

        this->declare_parameter("counter_divider_publish", 200);
        this->get_parameter("counter_divider_publish", counter_divider_publish);

        //----Publiher
        if (use_socket_can)
        {
            if (can_to_car)
            {
                pub_battery = this->create_publisher<std_msgs::msg::Int16>("/can/battery", 1);
                pub_encoder = this->create_publisher<std_msgs::msg::Int32>("/can/encoder", 1);
                pub_fb_tps_accelerator = this->create_publisher<std_msgs::msg::UInt8>("/can/fb_tps_accelerator", 1);
                pub_fb_transmission = this->create_publisher<std_msgs::msg::UInt8>("/can/fb_transmission", 1);
                pub_error_code = this->create_publisher<std_msgs::msg::Int16>("/can/error_code", 1);
            }
            else
            {
                pub_eps_encoder = this->create_publisher<std_msgs::msg::Float32>("/can/eps_encoder", 1);
                pub_fb_eps_mode = this->create_publisher<std_msgs::msg::UInt8>("/can/eps_mode", 1);
                pub_imu_can = this->create_publisher<sensor_msgs::msg::Imu>("/can/imu", 1);
                pub_gyro_counter = this->create_publisher<std_msgs::msg::UInt8>("/can/gyro_counter", 1);
            }
        }
        else
        {
            pub_battery = this->create_publisher<std_msgs::msg::Int16>("/can/battery", 1);
            pub_encoder = this->create_publisher<std_msgs::msg::Int32>("/can/encoder", 1);
            pub_fb_tps_accelerator = this->create_publisher<std_msgs::msg::UInt8>("/can/fb_tps_accelerator", 1);
            pub_fb_transmission = this->create_publisher<std_msgs::msg::UInt8>("/can/fb_transmission", 1);
            pub_error_code = this->create_publisher<std_msgs::msg::Int16>("/can/error_code", 1);
            pub_eps_encoder = this->create_publisher<std_msgs::msg::Float32>("/can/eps_encoder", 1);
            pub_fb_eps_mode = this->create_publisher<std_msgs::msg::UInt8>("/can/eps_mode", 1);
            pub_imu_can = this->create_publisher<sensor_msgs::msg::Imu>("/can/imu", 1);
            pub_gyro_counter = this->create_publisher<std_msgs::msg::UInt8>("/can/gyro_counter", 1);
        }

        //----Timer
        thread_routine = std::thread(std::bind(&CANbus_HAL::callback_routine_multi_thread, this), this);
        // tim_50hz = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&CANbus_HAL::callback_routine, this));

        //----Subscriber
        sub_master_actuator = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/master/actuator", 1, std::bind(&CANbus_HAL::callback_sub_master_actuator, this, std::placeholders::_1));
        sub_master_global_fsm = this->create_subscription<std_msgs::msg::Int16>(
            "/master/global_fsm", 1, std::bind(&CANbus_HAL::callback_sub_master_global_fsm, this, std::placeholders::_1));

        if (use_socket_can)
        {
            char cmd[100];
            sprintf(cmd, "sudo ip link set %s up type can bitrate %d", if_name.c_str(), bitrate);
            logger.info("Executing command: %s", cmd);
            system(cmd);

            socket_can = can_init(if_name.c_str());
            if (socket_can < 0)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to initialize CAN interface");
                rclcpp::shutdown();
            }
        }
        else
        {
            char char_ptr_if_name[100];
            sprintf(char_ptr_if_name, "%s", if_name.c_str());
            socket_can = jhctech_Open(char_ptr_if_name);
            if (socket_can < 0)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to initialize CAN interface");
                rclcpp::shutdown();
            }

            int ret_buffer = set_can_jhctech_bitrate_id(bitrate);
            if (ret_buffer < 0)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to initialize CAN Bitrate");
                rclcpp::shutdown();
            }
        }

        logger.info("CANbus_HAL init success");
    }

    void callback_sub_master_global_fsm(const std_msgs::msg::Int16::SharedPtr msg)
    {
        master_global_fsm = msg->data;
    }

    void callback_sub_master_actuator(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        eps_actuation = msg->data[1];
    }

    void callback_routine_multi_thread()
    {
        while (rclcpp::ok())
        {
            callback_routine();
            // std::this_thread::sleep_for(std::chrono::microseconds(1));
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    void callback_routine()
    {
        eps_flag = 1;
        if (master_global_fsm == 3 || master_global_fsm == 5 || master_global_fsm == 6)
        {
            eps_flag = 2;
        }

        if (use_socket_can)
        {
            if (!can_to_car)
            {
                if (counter_can_send >= counter_divider_can_send)
                {
                    struct can_frame frame;
                    frame.can_id = COB_ID_EPS_ACTUATION;
                    frame.can_dlc = 3;

                    static const float ENC_RAD2CNTR = EPS_ENCODER_MAX_COUNTER / EPS_ENCODER_MAX_RAD;
                    int16_t eps_actuation_cntr = eps_actuation * ENC_RAD2CNTR;

                    memcpy(&frame.data[0], &eps_flag, 1);
                    memcpy(&frame.data[1], &eps_actuation_cntr, 2);

                    if (write(socket_can, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
                    {
                        logger.warn("SOCKET CAN SEND FAILED\n");
                    }
                    counter_can_send = 0;
                }
            }
            counter_can_send++;

            parse_can_frame(); // Ini blocking
        }
        else
        {
            if (counter_can_send >= counter_divider_can_send)
            {
                uint8_t data_send_buffer[3] = {0};

                static const float ENC_RAD2CNTR = EPS_ENCODER_MAX_COUNTER / EPS_ENCODER_MAX_RAD;
                int16_t eps_actuation_cntr = eps_actuation * ENC_RAD2CNTR;

                memcpy(data_send_buffer, &eps_flag, 1);
                memcpy(data_send_buffer + 1, &eps_actuation_cntr, 2);

                int eps_jhctech_can_id = jhctech_can_id;
                if (jhctech_can_id == -1)
                {
                    eps_jhctech_can_id = 1;
                }

                long ret = jhctech_SendDataFrame(socket_can, 't', eps_jhctech_can_id, COB_ID_EPS_ACTUATION, data_send_buffer, 3);
                if (ret < 0)
                {
                    logger.warn("CAN%d SEND FAILED", eps_jhctech_can_id);
                }
                counter_can_send = 0;
            }
            counter_can_send++;

            parse_can_jhctech(); // Ini blocking
        }

        if (use_socket_can)
        {
            if (can_to_car)
            {
                static uint16_t counter_divider_pub_battery = 0;
                if (counter_divider_pub_battery++ >= counter_divider_publish)
                {
                    counter_divider_pub_battery = 0;
                    std_msgs::msg::Int16 msg_battery;
                    msg_battery.data = battery;
                    pub_battery->publish(msg_battery);

                    std_msgs::msg::Int16 msg_error_code;
                    msg_error_code.data = error_code;
                    pub_error_code->publish(msg_error_code);
                }

                if (prev_epoch_encoder != epoch_encoder)
                {
                    std_msgs::msg::Int32 msg_encoder;
                    msg_encoder.data = encoder;
                    pub_encoder->publish(msg_encoder);
                }

                std_msgs::msg::UInt8 msg_fb_tps_accelerator;
                msg_fb_tps_accelerator.data = fb_tps_accelerator;
                pub_fb_tps_accelerator->publish(msg_fb_tps_accelerator);

                std_msgs::msg::UInt8 msg_fb_transmission;
                msg_fb_transmission.data = fb_transmission;
                pub_fb_transmission->publish(msg_fb_transmission);
            }
            else
            {
                std_msgs::msg::Float32 msg_eps_encoder;
                msg_eps_encoder.data = eps_encoder_fb;
                pub_eps_encoder->publish(msg_eps_encoder);

                std_msgs::msg::UInt8 msg_fb_eps_mode;
                msg_fb_eps_mode.data = eps_mode_fb;
                pub_fb_eps_mode->publish(msg_fb_eps_mode);

                std_msgs::msg::UInt8 msg_gyro_counter;
                msg_gyro_counter.data = counter_gyro_update;
                pub_gyro_counter->publish(msg_gyro_counter);

                pub_imu_can->publish(imu_msg);
            }
        }
        else
        {
            static uint16_t counter_divider_pub_battery = 0;
            if (counter_divider_pub_battery++ >= counter_divider_publish)
            {
                counter_divider_pub_battery = 0;
                std_msgs::msg::Int16 msg_battery;
                msg_battery.data = battery;
                pub_battery->publish(msg_battery);

                std_msgs::msg::Int16 msg_error_code;
                msg_error_code.data = error_code;
                pub_error_code->publish(msg_error_code);
            }

            if (prev_epoch_encoder != epoch_encoder)
            {
                std_msgs::msg::Int32 msg_encoder;
                msg_encoder.data = encoder;
                pub_encoder->publish(msg_encoder);
            }

            std_msgs::msg::UInt8 msg_fb_tps_accelerator;
            msg_fb_tps_accelerator.data = fb_tps_accelerator;
            pub_fb_tps_accelerator->publish(msg_fb_tps_accelerator);

            std_msgs::msg::UInt8 msg_fb_transmission;
            msg_fb_transmission.data = fb_transmission;
            pub_fb_transmission->publish(msg_fb_transmission);

            std_msgs::msg::Float32 msg_eps_encoder;
            msg_eps_encoder.data = eps_encoder_fb;
            pub_eps_encoder->publish(msg_eps_encoder);

            std_msgs::msg::UInt8 msg_fb_eps_mode;
            msg_fb_eps_mode.data = eps_mode_fb;
            pub_fb_eps_mode->publish(msg_fb_eps_mode);

            std_msgs::msg::UInt8 msg_gyro_counter;
            msg_gyro_counter.data = counter_gyro_update;
            pub_gyro_counter->publish(msg_gyro_counter);

            pub_imu_can->publish(imu_msg);
        }
    }

    void parse_can_jhctech()
    {
        static uint8_t can_data_buffer[1024] = {0}; // Raw serial data
        static Canbus_msg *can_data = NULL;

        {
            int ret_buffer = jhctech_GetComMessage(socket_can, can_data_buffer, 1024);
            uint32_t errorCode = 0;

            if (0 != (errorCode = jhctech_CheckUartError(can_data_buffer, ret_buffer)))
            {
                logger.error("UART errorCode is 0x%08x", errorCode);
            }
            if (0 != (errorCode = jhctech_CheckCan0Error(can_data_buffer, ret_buffer)))
            {
                logger.error("CAN0 errorCode is 0x%08x", errorCode);
            }
            if (0 != (errorCode = jhctech_CheckCan1Error(can_data_buffer, ret_buffer)))
            {
                logger.error("CAN1 errorCode is 0x%08x", errorCode);
            }
        }

        {
            int ret_buffer = jhctech_Receive(&can_data, can_data_buffer);
            (void)ret_buffer;

            while (can_data != NULL)
            {
                // logger.info("Id: %x", can_data->canId);
                if (can_data->canId == COB_ID_CAR_ENCODER)
                {
                    encoder = (can_data->data[3] | (can_data->data[2] << 8));

                    prev_epoch_encoder = epoch_encoder;
                    epoch_encoder++;
                    if (epoch_encoder >= 255)
                        epoch_encoder = 0;
                }
                else if (can_data->canId == COB_ID_CAR_BATTERY)
                {
                    battery = can_data->data[1] + 1;
                }
                else if (can_data->canId == COB_ID_CAR_FB_ACCELERATOR)
                {
                    fb_tps_accelerator = can_data->data[2];
                }
                else if (can_data->canId == COB_ID_CAR_FB_TRANSMISSION)
                {
                    /**
                     * F-N-R = 1-3-5
                     */
                    fb_transmission = can_data->data[5];
                }
                else if (can_data->canId == COB_ID_EPS_ENCODER)
                {
                    // logger.warn("MASUK");
                    eps_mode_fb = can_data->data[0];

                    static const float ENC_CNTR2RAD = EPS_ENCODER_MAX_RAD / EPS_ENCODER_MAX_COUNTER;

                    int16_t fb_eps_encoder_buffer = 0;
                    memcpy(&fb_eps_encoder_buffer, &can_data->data[1], 2);

                    eps_encoder_fb = fb_eps_encoder_buffer * ENC_CNTR2RAD;
                }
                else if (can_data->canId == COB_ID_GYRO_RION)
                {
                    int16_t data_buffer = 0;

                    // Posisi
                    if (can_data->data[7] == 0x00)
                    {
                        float r = 0, p = 0, y = 0;
                        data_buffer = can_data->data[0] | (can_data->data[1] << 8);
                        r = (float)data_buffer * 0.01 * M_PI / 180;
                        data_buffer = can_data->data[2] | (can_data->data[3] << 8);
                        p = (float)data_buffer * 0.01 * M_PI / 180;
                        data_buffer = can_data->data[4] | (can_data->data[5] << 8);
                        y = (float)data_buffer * 0.01 * M_PI / 180;

                        tf2::Quaternion q_tf2;
                        q_tf2.setRPY(r, p, y);
                        geometry_msgs::msg::Quaternion q_msg;
                        q_msg.x = q_tf2.x();
                        q_msg.y = q_tf2.y();
                        q_msg.z = q_tf2.z();
                        q_msg.w = q_tf2.w();

                        imu_msg.orientation = q_msg;

                        counter_gyro_update++;
                        if (counter_gyro_update >= 255)
                            counter_gyro_update = 0;
                    }
                    // Akselero
                    else if (can_data->data[7] == 0x01)
                    {
                        data_buffer = can_data->data[0] | (can_data->data[1] << 8);
                        imu_msg.linear_acceleration.x = (float)data_buffer * 0.001;
                        data_buffer = can_data->data[2] | (can_data->data[3] << 8);
                        imu_msg.linear_acceleration.y = (float)data_buffer * 0.001;
                        data_buffer = can_data->data[4] | (can_data->data[5] << 8);
                        imu_msg.linear_acceleration.z = (float)data_buffer * 0.001;
                    }
                    // Gyro
                    else if (can_data->data[7] == 0x02)
                    {
                        data_buffer = can_data->data[0] | (can_data->data[1] << 8);
                        imu_msg.angular_velocity.x = (float)data_buffer * 0.01;
                        data_buffer = can_data->data[2] | (can_data->data[3] << 8);
                        imu_msg.angular_velocity.y = (float)data_buffer * 0.01;
                        data_buffer = can_data->data[4] | (can_data->data[5] << 8);
                        imu_msg.angular_velocity.z = (float)data_buffer * 0.01;
                    }
                }

                can_data = can_data->Next;
            }
        }
    }

    void parse_can_frame()
    {
        // Read the CAN frame
        struct can_frame frame;
        fd_set read_fds;
        struct timeval timeout;
        int retval;

        // Set up the file descriptor set
        FD_ZERO(&read_fds);
        FD_SET(socket_can, &read_fds);

        // Set timeout values
        timeout.tv_sec = 0;
        timeout.tv_usec = 100000;

        // Wait for data to be available on the set
        retval = select(socket_can + 1, &read_fds, NULL, NULL, &timeout);

        if (retval == -1)
        {
            error_code = 2;
            logger.error("Select error on CAN socket");
            return;
        }
        else if (retval == 0)
        {
            error_code = 3;
            logger.warn("CAN socket read timeout");
            return;
        }

        if (read(socket_can, &frame, sizeof(struct can_frame)) < 0)
        {
            error_code = 1;
            logger.error("Error reading CAN frame");
            return;
        }

        // Check if the connection is broken
        if (frame.can_id == 0x000)
        {
            error_code = 4;
            logger.error("CAN connection broken");
            return;
        }

        error_code = 0;

        // Check if the response is from the expected node
        if (frame.can_id == COB_ID_CAR_ENCODER)
        {
            encoder = (frame.data[3] | (frame.data[2] << 8));

            prev_epoch_encoder = epoch_encoder;
            epoch_encoder++;
            if (epoch_encoder >= 255)
                epoch_encoder = 0;
        }
        else if (frame.can_id == COB_ID_CAR_BATTERY)
        {
            battery = frame.data[1] + 1;
        }
        else if (frame.can_id == COB_ID_CAR_FB_ACCELERATOR)
        {
            fb_tps_accelerator = frame.data[2];
        }
        else if (frame.can_id == COB_ID_CAR_FB_TRANSMISSION)
        {
            /**
             * F-N-R = 1-3-5
             */
            fb_transmission = frame.data[5];
        }
        else if (frame.can_id == COB_ID_EPS_ENCODER)
        {
            eps_mode_fb = frame.data[0];

            static const float ENC_CNTR2RAD = EPS_ENCODER_MAX_RAD / EPS_ENCODER_MAX_COUNTER;

            int16_t fb_eps_encoder_buffer = 0;
            memcpy(&fb_eps_encoder_buffer, &frame.data[1], 2);

            eps_encoder_fb = fb_eps_encoder_buffer * ENC_CNTR2RAD;
        }
        else if (frame.can_id == COB_ID_GYRO_RION)
        {
            int16_t data_buffer = 0;
            counter_gyro_update++;
            if (counter_gyro_update >= 255)
                counter_gyro_update = 0;

            // Posisi
            if (frame.data[7] == 0x00)
            {
                float r = 0, p = 0, y = 0;
                data_buffer = frame.data[0] | (frame.data[1] << 8);
                r = (float)data_buffer * 0.01 * M_PI / 180;
                data_buffer = frame.data[2] | (frame.data[3] << 8);
                p = (float)data_buffer * 0.01 * M_PI / 180;
                data_buffer = frame.data[4] | (frame.data[5] << 8);
                y = (float)data_buffer * 0.01 * M_PI / 180;

                tf2::Quaternion q_tf2;
                q_tf2.setRPY(r, p, y);
                geometry_msgs::msg::Quaternion q_msg;
                q_msg.x = q_tf2.x();
                q_msg.y = q_tf2.y();
                q_msg.z = q_tf2.z();
                q_msg.w = q_tf2.w();

                imu_msg.orientation = q_msg;

                counter_gyro_update++;
                if (counter_gyro_update >= 255)
                    counter_gyro_update = 0;
            }
            // Akselero
            else if (frame.data[7] == 0x01)
            {
                data_buffer = frame.data[0] | (frame.data[1] << 8);
                imu_msg.linear_acceleration.x = (float)data_buffer * 0.001;
                data_buffer = frame.data[2] | (frame.data[3] << 8);
                imu_msg.linear_acceleration.y = (float)data_buffer * 0.001;
                data_buffer = frame.data[4] | (frame.data[5] << 8);
                imu_msg.linear_acceleration.z = (float)data_buffer * 0.001;
            }
            // Gyro
            else if (frame.data[7] == 0x02)
            {
                data_buffer = frame.data[0] | (frame.data[1] << 8);
                imu_msg.angular_velocity.x = (float)data_buffer * 0.01 * M_PI / 180;
                data_buffer = frame.data[2] | (frame.data[3] << 8);
                imu_msg.angular_velocity.y = (float)data_buffer * 0.01 * M_PI / 180;
                data_buffer = frame.data[4] | (frame.data[5] << 8);
                imu_msg.angular_velocity.z = (float)data_buffer * 0.01 * M_PI / 180;
            }
        }
    }

    int8_t can_clear_recv_buffer(int8_t s)
    {
        struct can_frame frame;
        int flags = fcntl(s, F_GETFL, 0);
        int nbytes;

        // Set socket to non-blocking mode
        fcntl(s, F_SETFL, flags | O_NONBLOCK);

        // Read and discard all frames
        while ((nbytes = read(s, &frame, sizeof(struct can_frame))) > 0)
        {
            // Frame read and discarded
        }

        // Check for read error
        if (nbytes < 0 && errno != EAGAIN)
        {
            perror("Error clearing CAN receive buffer");
            return -1;
        }

        // Restore original socket flags (back to blocking mode if it was originally)
        fcntl(s, F_SETFL, flags);

        return 0;
    }

    int set_can_jhctech_bitrate_id(int bitrate)
    {
        int ret_buffer = 0;
        uint8_t baud = 0;
        switch (bitrate)
        {
        case 10000:
            baud = 0;
            break;
        case 20000:
            baud = 1;
            break;
        case 50000:
            baud = 2;
            break;
        case 100000:
            baud = 3;
            break;
        case 125000:
            baud = 4;
            break;
        case 250000:
            baud = 5;
            break;
        case 500000:
            baud = 6;
            break;
        case 800000:
            baud = 7;
            break;
        case 1000000:
            baud = 8;
            break;
        default:
            ret_buffer = -1;
            break;
        }

        ret_buffer = jhctech_SetBaudRate(socket_can, 1, baud);
        sleep(1);
        ret_buffer = jhctech_SetBaudRate(socket_can, 2, baud);

        return ret_buffer;
    }

    int8_t can_init(const char *interface)
    {
        int8_t s;
        struct sockaddr_can addr;
        struct ifreq ifr;

        if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
        {
            perror("Error while opening socket");
            return -1;
        }

        strcpy(ifr.ifr_name, interface);
        ioctl(s, SIOCGIFINDEX, &ifr);

        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
        {
            perror("Error in socket bind");
            return -1;
        }

        return s;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node_CANbus_HAL = std::make_shared<CANbus_HAL>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node_CANbus_HAL);
    executor.spin();

    return 0;
}