#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <soem/ethercat.h>

#define EC_TIMEOUTMON 5000

#define EL1809_ID 0x07113052
#define EL3104_ID 0x0c203052
#define EL3204_ID 0x0c843052
#define EL5152_ID 0x14203052
#define EL6751_ID 0x1a5f3052
#define EL2008_ID 0x07d83052
#define EL6021_ID 0x17853052

/**
 * Scan and save to EEPROM the CANopen slaves configuration
 */
int scan_CANopen_Slaves(uint16_t slave)
{
    uint8_t _f002_1[2] = {0x01, 0x00};
    ec_SDOwrite(slave, 0xf002, 0x01, FALSE, sizeof(_f002_1), &_f002_1, EC_TIMEOUTRXM);

    printf("Wait...\n");

    while (1)
    {
        uint8_t _f002_2;
        int _f002_2_s = sizeof(_f002_2);
        int _f002_2_wkc = ec_SDOread(slave, 0xf002, 0x02, FALSE, &_f002_2_s, &_f002_2, EC_TIMEOUTRXM);
        if (_f002_2_wkc > 0)
        {
            printf("f002_2: %d\n", _f002_2);

            if (_f002_2 <= 3)
                break;
        }

        usleep(10000);
    }

    // Untuk lanjutannya lihat di 90xx

    // Save ke EEPROM
    uint32_t _1010 = 0x65766173;
    ec_SDOwrite(slave, 0x1010, 0x01, TRUE, sizeof(_1010), &_1010, EC_TIMEOUTRXM);

    while (ec_iserror())
    {
        printf("%s\n", ec_elist2string());
    }

    return 1;
}

int remove_CAN_eeprom(uint16_t slave)
{
    uint32_t _1011 = 0x64616F6C; // Delete
    ec_SDOwrite(slave, 0x1011, 0x01, TRUE, sizeof(_1011), &_1011, EC_TIMEOUTRXM);

    while (EcatError)
        printf("%s", ec_elist2string());

    return 1;
}

int init_CAN_Startup(uint16_t slave)
{

    while (EcatError)
        printf("%s", ec_elist2string());

    return 1;
}

class Beckhoff : public rclcpp::Node
{
public:
    rclcpp::TimerBase::SharedPtr tim_50hz;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_test;

    // Configs
    // =======================================================
    std::string if_name;

    // Vars
    // =======================================================
    uint16_t slave_canopen_id = -1;
    int expectedWKC = 0;
    uint8 IOmap[4096]; // I/O map for PDOs

    Beckhoff() : Node("beckhoff")
    {
        this->declare_parameter("if_name", "eth0");
        this->get_parameter("if_name", if_name);

        // if (init_beckhoff() > 0)
        // {
        //     RCLCPP_ERROR(this->get_logger(), "Failed to initialize Beckhoff");
        //     return;
        // }

        //----Timer
        tim_50hz = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&Beckhoff::callback_tim_50hz, this));

        //----Publisher
        pub_test = this->create_publisher<std_msgs::msg::String>("beckhoff", 1);
    }

    void callback_tim_50hz()
    {
        static int count = 0;
        std::string message = "Beckhoff! " + std::to_string(count);
        // RCLCPP_INFO(this->get_logger(), "%s", message.c_str());

        std_msgs::msg::String msg;
        msg.data = message;
        pub_test->publish(msg);

        count++;
    }

    int8_t init_beckhoff()
    {
        if (ec_init(if_name.c_str()))
        {
            if (ec_config_init(FALSE) > 0)
            {
                while (EcatError)
                    printf("%s", ec_elist2string());

                printf("%d slaves found and configured.\n", ec_slavecount);

                for (uint8_t slave = 1; slave <= ec_slavecount; slave++)
                {
                    switch (ec_slave[slave].eep_id)
                    {
                    case EL6751_ID:
                        slave_canopen_id = slave;
                        printf("Slave Canopen ID: %d\n", slave_canopen_id);
                        break;
                    }
                }

                if (slave_canopen_id == -1)
                {
                    return 4;
                }

                expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
                printf("Calculated workcounter %d\n", expectedWKC);

                // ============= CONFIGURE STARTUP STATE =============
                // ec_slave[slave_canopen_id].PO2SOconfig = &scan_CANopen_Slaves;
                // ec_slave[slave_canopen_id].PO2SOconfig = &remove_CAN_eeprom;
                ec_slave[slave_canopen_id].PO2SOconfig = &init_CAN_Startup;
                // ===================================================

                ec_config_map(&IOmap);
                ec_configdc();

                // Set ke SO
                // Set ke O

                return 0;
            }
            else
            {
                return 2;
            }
        }
        else
        {
            return 1;
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node_beckhoff = std::make_shared<Beckhoff>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node_beckhoff);
    executor.spin();

    return 0;
}