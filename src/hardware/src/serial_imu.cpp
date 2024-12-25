#include "rclcpp/rclcpp.hpp"
#include "ros2_utils/global_definitions.hpp"
#include "ros2_utils/help_logger.hpp"
#include "sensor_msgs/msg/imu.hpp"

//--Linux Headers
#include <errno.h>     // Error integer and strerror() function
#include <fcntl.h>     // Contains file controls like O_RDWR
#include <sys/ioctl.h> // ioctl()
#include <termios.h>   // Contains POSIX terminal control definitions
#include <unistd.h>    // write(), read(), close()

#define READ_PROTOCOL 0x55
#define WRITE_PROTOCOL 0xFF

#define TYPE_TIME 0x50
#define TYPE_ACC 0x51
#define TYPE_ANGULAR_VELOCITY 0x52
#define TYPE_ANGLE 0x53
#define TYPE_MAGNETIC 0x54
#define TYPE_PORT 0x55
#define TYPE_BAROMETRIC 0x56
#define TYPE_LAT_LONG 0x57
#define TYPE_GROUND_SPEED 0x58
#define TYPE_QUATERNION 0x59
#define TYPE_GPS_ACCURACY 0x5A
#define TYPE_READ 0x5F

#define GRAVITY 9.8

HelpLogger logger;

class SerialIMU : public rclcpp::Node
{
public:
    rclcpp::TimerBase::SharedPtr tim_50hz;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu;

    // Configs
    // =======================================================
    std::string port;
    std::string frame_id = "imu";

    // Vars
    // =======================================================
    int serial_port_fd;

    SerialIMU()
        : Node("serial_imu")
    {
        this->declare_parameter("port", "/dev/ttyUSB0");
        this->get_parameter("port", port);

        if (!logger.init())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize logger");
            rclcpp::shutdown();
        }

        if (init_serial() > 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize serial");
            rclcpp::shutdown();
        }

        //----Timer
        tim_50hz = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&SerialIMU::callback_tim_50hz, this));

        //----Publisher
        pub_imu = this->create_publisher<sensor_msgs::msg::Imu>("/hardware/imu", 1);

        logger.info("Serial IMU init");
    }

    void callback_tim_50hz()
    {
        read_serial();
    }

    int8_t init_serial()
    {
        // Open the serial port. Change device path as needed (currently set to an standard FTDI USB-UART cable type device)
        serial_port_fd = open(port.c_str(), O_RDWR);

        if (serial_port_fd < 0)
        {
            logger.error("Error %i from opening usb device: %s", errno, strerror(errno));
            return 1;
        }

        // Create new termios struct, we call it 'tty' for convention
        struct termios tty;

        // Read in existing settings, and handle any error
        if (tcgetattr(serial_port_fd, &tty) != 0)
        {
            logger.error("Error %i from tcgetattr: %s", errno, strerror(errno));
            return 1;
        }

        tty.c_cflag &= ~PARENB;        // Clear parity bit, disabling parity (most common)
        tty.c_cflag &= ~CSTOPB;        // Clear stop field, only one stop bit used in communication (most common)
        tty.c_cflag &= ~CSIZE;         // Clear all bits that set the data size
        tty.c_cflag |= CS8;            // 8 bits per byte (most common)
        tty.c_cflag &= ~CRTSCTS;       // Disable RTS/CTS hardware flow control (most common)
        tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

        tty.c_lflag &= ~ICANON;
        tty.c_lflag &= ~ECHO;                                                        // Disable echo
        tty.c_lflag &= ~ECHOE;                                                       // Disable erasure
        tty.c_lflag &= ~ECHONL;                                                      // Disable new-line echo
        tty.c_lflag &= ~ISIG;                                                        // Disable interpretation of INTR, QUIT and SUSP
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);                                      // Turn off s/w flow ctrl
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes

        tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
        tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
        // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
        // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

        tty.c_cc[VTIME] = 0; // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
        tty.c_cc[VMIN] = 0;

        // Set in/out baud rate to be 9600
        cfsetispeed(&tty, B9600);
        cfsetospeed(&tty, B9600);

        // status |= TIOCM_DTR; /* turn on DTR */
        // status |= TIOCM_RTS; /* turn on RTS */

        // Save tty settings, also checking for error
        if (tcsetattr(serial_port_fd, TCSANOW, &tty) != 0)
        {
            logger.error("Error %i from tcsetattr: %s", errno, strerror(errno));
            return 1;
        }

        return 0;
    }

    int8_t read_serial()
    {
        unsigned char recv_buffer[128];
        static sensor_msgs::msg::Imu imu_msg;

        uint16_t bytes_available = 0;
        if (ioctl(serial_port_fd, FIONREAD, &bytes_available) == -1)
        {
            return -1;
        }

        if (bytes_available == 0)
            return 0;

        int8_t recv_len = read(serial_port_fd, &recv_buffer, bytes_available);

        for (int i = 0; i < recv_len; i++)
        {
            if (recv_buffer[i] == READ_PROTOCOL)
            {
                if (recv_buffer[i + 1] == TYPE_ACC)
                {
                    int16_t acc_x_buffer;
                    int16_t acc_y_buffer;
                    int16_t acc_z_buffer;

                    double acc_x;
                    double acc_y;
                    double acc_z;

                    memcpy(&acc_x_buffer, recv_buffer + i + 2, 2);
                    memcpy(&acc_y_buffer, recv_buffer + i + 4, 2);
                    memcpy(&acc_z_buffer, recv_buffer + i + 6, 2);

                    acc_x = (acc_x_buffer / 32768.0) * 16.0 * GRAVITY;
                    acc_y = (acc_y_buffer / 32768.0) * 16.0 * GRAVITY;
                    acc_z = (acc_z_buffer / 32768.0) * 16.0 * GRAVITY;

                    imu_msg.linear_acceleration.x = acc_x;
                    imu_msg.linear_acceleration.y = acc_y;
                    imu_msg.linear_acceleration.z = acc_z;

                    uint16_t crc_sum = 0;
                    crc_sum = READ_PROTOCOL + TYPE_ACC + (acc_x_buffer & 0xFF) + (acc_x_buffer >> 8) + (acc_y_buffer & 0xFF) + (acc_y_buffer >> 8) + (acc_z_buffer & 0xFF) + (acc_z_buffer >> 8);

                    if (crc_sum == recv_buffer[i + 10])
                    {
                        // logger.info("CRC OK");
                    }

                    // logger.info("Acc X: %f, Acc Y: %f, Acc Z: %f", acc_x, acc_y, acc_z);
                }
                else if (recv_buffer[i + 1] == TYPE_ANGULAR_VELOCITY)
                {
                    int16_t ang_vel_x_buffer;
                    int16_t ang_vel_y_buffer;
                    int16_t ang_vel_z_buffer;

                    double ang_vel_x;
                    double ang_vel_y;
                    double ang_vel_z;

                    memcpy(&ang_vel_x_buffer, recv_buffer + i + 2, 2);
                    memcpy(&ang_vel_y_buffer, recv_buffer + i + 4, 2);
                    memcpy(&ang_vel_z_buffer, recv_buffer + i + 6, 2);

                    ang_vel_x = (ang_vel_x_buffer / 32768.0) * 2000.0; // degree per second
                    ang_vel_y = (ang_vel_y_buffer / 32768.0) * 2000.0; // degree per second
                    ang_vel_z = (ang_vel_z_buffer / 32768.0) * 2000.0; // degree per second

                    imu_msg.angular_velocity.x = ang_vel_x;
                    imu_msg.angular_velocity.y = ang_vel_y;
                    imu_msg.angular_velocity.z = ang_vel_z;

                    uint16_t crc_sum = 0;
                    crc_sum = READ_PROTOCOL + TYPE_ANGULAR_VELOCITY + (ang_vel_x_buffer & 0xFF) + (ang_vel_x_buffer >> 8) + (ang_vel_y_buffer & 0xFF) + (ang_vel_y_buffer >> 8) + (ang_vel_z_buffer & 0xFF) + (ang_vel_z_buffer >> 8);

                    if (crc_sum == recv_buffer[i + 10])
                    {
                        // logger.info("CRC OK");
                    }

                    // logger.info("Ang Vel X: %f, Ang Vel Y: %f, Ang Vel Z: %f", ang_vel_x, ang_vel_y, ang_vel_z);
                }
                else if (recv_buffer[i + 1] == TYPE_ANGLE)
                {
                    int16_t angle_x_buffer;
                    int16_t angle_y_buffer;
                    int16_t angle_z_buffer;

                    double angle_x;
                    double angle_y;
                    double angle_z;

                    memcpy(&angle_x_buffer, recv_buffer + i + 2, 2);
                    memcpy(&angle_y_buffer, recv_buffer + i + 4, 2);
                    memcpy(&angle_z_buffer, recv_buffer + i + 6, 2);

                    angle_x = (angle_x_buffer / 32768.0) * 180.0;
                    angle_y = (angle_y_buffer / 32768.0) * 180.0;
                    angle_z = (angle_z_buffer / 32768.0) * 180.0;

                    double roll = angle_x * M_PI / 180.0;
                    double pitch = angle_y * M_PI / 180.0;
                    double yaw = angle_z * M_PI / 180.0;

                    double cy = cos(yaw * 0.5);
                    double sy = sin(yaw * 0.5);
                    double cp = cos(pitch * 0.5);
                    double sp = sin(pitch * 0.5);
                    double cr = cos(roll * 0.5);
                    double sr = sin(roll * 0.5);

                    double qw = cy * cp * cr + sy * sp * sr;
                    double qx = cy * cp * sr - sy * sp * cr;
                    double qy = sy * cp * sr + cy * sp * cr;
                    double qz = sy * cp * cr - cy * sp * sr;

                    imu_msg.orientation.w = qw;
                    imu_msg.orientation.x = qx;
                    imu_msg.orientation.y = qy;
                    imu_msg.orientation.z = qz;

                    uint16_t crc_sum = 0;
                    crc_sum = READ_PROTOCOL + TYPE_ANGLE + (angle_x_buffer & 0xFF) + (angle_x_buffer >> 8) + (angle_y_buffer & 0xFF) + (angle_y_buffer >> 8) + (angle_z_buffer & 0xFF) + (angle_z_buffer >> 8);

                    if (crc_sum == recv_buffer[i + 10])
                    {
                        // logger.info("CRC OK");
                    }

                    // logger.info("Roll: %f, Pitch: %f, Yaw: %f", roll, pitch, yaw);
                }
                else if (recv_buffer[i + 1] == TYPE_MAGNETIC)
                {
                    int16_t mag_x;
                    int16_t mag_y;
                    int16_t mag_z;

                    memcpy(&mag_x, recv_buffer + i + 2, 2);
                    memcpy(&mag_y, recv_buffer + i + 4, 2);
                    memcpy(&mag_z, recv_buffer + i + 6, 2);

                    uint16_t crc_sum = 0;
                    crc_sum = READ_PROTOCOL + TYPE_MAGNETIC + (mag_x & 0xFF) + (mag_x >> 8) + (mag_y & 0xFF) + (mag_y >> 8) + (mag_z & 0xFF) + (mag_z >> 8);

                    if (crc_sum == recv_buffer[i + 10])
                    {
                        // logger.info("CRC OK");
                    }

                    // logger.info("Mag X: %d, Mag Y: %d, Mag Z: %d", mag_x, mag_y, mag_z);
                }
            }
        }

        pub_imu->publish(imu_msg);

        return 0;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node_imu = std::make_shared<SerialIMU>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node_imu);
    executor.spin();

    return 0;
}