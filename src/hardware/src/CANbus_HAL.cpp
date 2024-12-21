#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ros2_utils/global_definitions.hpp"
#include "ros2_utils/help_logger.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_msgs/msg/int32.hpp"

#include <cstring>
#include <cstdio>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <cerrno>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

class CANbus_HAL : public rclcpp::Node
{
public:
    rclcpp::TimerBase::SharedPtr tim_50hz;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_battery;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_encoder;

    HelpLogger logger;
    int error_code = 0;

    // Configs
    // =======================================================
    std::string if_name;
    int bitrate = 125000;

    int socket_can = -1;

    int battery = 0;
    int encoder = 0;
    uint8_t fb_tps_accelerator = 0;
    uint8_t fb_transmission = 0;

    CANbus_HAL() : Node("CANbus_HAL")
    {
        this->declare_parameter("if_name", "can0");
        this->get_parameter("if_name", if_name);

        this->declare_parameter("bitrate", 125000);
        this->get_parameter("bitrate", bitrate);

        //----Timer
        tim_50hz = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&CANbus_HAL::callback_routine, this));

        //----Publiher
        pub_battery = this->create_publisher<std_msgs::msg::Int16>("/can/battery", 1);
        pub_encoder = this->create_publisher<std_msgs::msg::Int32>("/can/encoder", 1);

        if (!logger.init())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize logger");
            rclcpp::shutdown();
        }

        char cmd[100];
        sprintf(cmd, "sudo ip link set %s up type can bitrate %d", if_name.c_str(), bitrate);
        system(cmd);

        socket_can = can_init(if_name.c_str());
        if (socket_can < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize CAN interface");
            rclcpp::shutdown();
        }

        logger.info("CANbus_HAL init success");
    }

    void callback_routine()
    {
        parse_can_frame();

        std_msgs::msg::Int16 msg_battery;
        msg_battery.data = battery;
        pub_battery->publish(msg_battery);

        std_msgs::msg::Int32 msg_encoder;
        msg_encoder.data = encoder;
        pub_encoder->publish(msg_encoder);
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

        if (retval == -1 || retval == 0)
        {
            return;
        }

        if (read(socket_can, &frame, sizeof(struct can_frame)) < 0)
        {
            perror("Read");
            logger.error("Error reading CAN frame");
        }

        // Check if the response is from the expected node
        if (frame.can_id == 0x388)
        {
            encoder = frame.data[2] | (frame.data[3] << 8);
        }
        else if (frame.can_id == 0x109)
        {
            battery = frame.data[1] + 1;
        }
        else if (frame.can_id == 0x101)
        {
            fb_tps_accelerator = frame.data[2];
        }
        else if (frame.can_id == 0x18C)
        {
            /**
             * F-N-R = 1-3-5
             */
            fb_transmission = frame.data[5];
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

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node_CANbus_HAL);
    executor.spin();

    return 0;
}