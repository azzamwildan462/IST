#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ros2_utils/help_logger.hpp"

#include <soem/ethercat.h>

#define EC_TIMEOUTMON 5000

#define EL1809_ID 0x07113052
#define EL3104_ID 0x0c203052
#define EL3204_ID 0x0c843052
#define EL5152_ID 0x14203052
#define EL6751_ID 0x1a5f3052
#define EL2008_ID 0x07d83052
#define EL6021_ID 0x17853052

HelpLogger logger;

/**
 * Scan and save to EEPROM the CANopen slaves configuration
 */
int scan_CANopen_Slaves(uint16_t slave)
{
    uint8_t _f002_1[2] = {0x01, 0x00};
    ec_SDOwrite(slave, 0xf002, 0x01, FALSE, sizeof(_f002_1), &_f002_1, EC_TIMEOUTRXM);

    logger.info("Wait...");

    while (1)
    {
        uint8_t _f002_2;
        static uint8_t prev_f002_2;
        int _f002_2_s = sizeof(_f002_2);
        int _f002_2_wkc = ec_SDOread(slave, 0xf002, 0x02, FALSE, &_f002_2_s, &_f002_2, EC_TIMEOUTRXM);
        if (_f002_2_wkc > 0)
        {
            if (prev_f002_2 != _f002_2)
            {
                if (100 == _f002_2)
                    logger.info("f002_2: %d", _f002_2);
                else if (130 == _f002_2)
                    logger.info("f002_2: %d", _f002_2);
                else if (180 == _f002_2)
                    logger.info("f002_2: %d", _f002_2);
            }

            if (_f002_2 <= 3)
                break;
        }

        prev_f002_2 = _f002_2;

        usleep(10000);
    }

    uint8_t _9000[200] = {0x00};
    int _9000_s = sizeof(_9000);
    int _9000_wkc = ec_SDOread(slave, 0x9000, 0x00, TRUE, &_9000_s, &_9000, 10 * EC_TIMEOUTRXM);
    if (_9000_wkc > 0 && _9000_s > 2)
    {
        logger.info("CO slave0: ");
        for (int k = 0; k < _9000_s; k++)
        {
            logger.info("0x%x ", _9000[k]);
        }
        logger.info("");
    }

    uint8_t _9010[200] = {0x00};
    int _9010_s = sizeof(_9010);
    int _9010_wkc = ec_SDOread(slave, 0x9010, 0x00, TRUE, &_9010_s, &_9010, 10 * EC_TIMEOUTRXM);
    if (_9010_wkc > 0 && _9010_s > 2)
    {
        logger.info("CO slave1: ");
        for (int k = 0; k < _9010_s; k++)
        {
            logger.info("0x%x ", _9010[k]);
        }
        logger.info("");
    }

    uint8_t _9020[200] = {0x00};
    int _9020_s = sizeof(_9020);
    int _9020_wkc = ec_SDOread(slave, 0x9020, 0x00, TRUE, &_9020_s, &_9020, 10 * EC_TIMEOUTRXM);
    if (_9020_wkc > 0 && _9020_s > 2)
    {
        logger.info("CO slave2: ");
        for (int k = 0; k < _9020_s; k++)
        {
            logger.info("0x%x ", _9020[k]);
        }
        logger.info("");
    }

    uint8_t _9030[200] = {0x00};
    int _9030_s = sizeof(_9030);
    int _9030_wkc = ec_SDOread(slave, 0x9030, 0x00, TRUE, &_9030_s, &_9030, 10 * EC_TIMEOUTRXM);
    if (_9030_wkc > 0 && _9030_s > 2)
    {
        logger.info("CO slave3: ");
        for (int k = 0; k < _9030_s; k++)
        {
            logger.info("0x%x ", _9030[k]);
        }
        logger.info("");
    }

    uint8_t _9040[200] = {0x00};
    int _9040_s = sizeof(_9040);
    int _9040_wkc = ec_SDOread(slave, 0x9040, 0x00, TRUE, &_9040_s, &_9040, 10 * EC_TIMEOUTRXM);
    if (_9040_wkc > 0 && _9040_s > 2)
    {
        logger.info("CO slave4: ");
        for (int k = 0; k < _9040_s; k++)
        {
            logger.info("0x%x ", _9040[k]);
        }
        logger.info("");
    }

    // Save ke EEPROM
    uint32_t _1010 = 0x65766173;
    ec_SDOwrite(slave, 0x1010, 0x01, TRUE, sizeof(_1010), &_1010, EC_TIMEOUTRXM);

    while (ec_iserror())
    {
        logger.warn("%s", ec_elist2string());
    }

    return 1;
}

int remove_CAN_eeprom(uint16_t slave)
{
    uint32_t _1011 = 0x64616F6C; // Delete
    ec_SDOwrite(slave, 0x1011, 0x01, TRUE, sizeof(_1011), &_1011, EC_TIMEOUTRXM);

    while (EcatError)
        logger.warn("%s", ec_elist2string());

    return 1;
}

int init_CAN_Startup(uint16_t slave)
{
    uint8_t _8003[68] = {
        0x08, 0x00,
        0x02, 0x1A,
        0x00,
        0x01, 0x00,
        0x00,

        0x02, 0x1A,
        0x01,
        0x04, 0x00,
        0x20, 0x00, 0x64, 0x60,

        0x02, 0x1A,
        0x02,
        0x04, 0x00,
        0x20, 0x00, 0x69, 0x60,

        0x02, 0x1A,
        0x00,
        0x01, 0x00,
        0x02,

        0x02, 0x16,
        0x00,
        0x01, 0x00,
        0x00,

        0x02, 0x16,
        0x01,
        0x04, 0x00,
        0x20, 0x00, 0xff, 0x60,

        0x02, 0x16,
        0x02,
        0x04, 0x00,
        0x10, 0x00, 0x71, 0x60,

        0x02, 0x16,
        0x00,
        0x01, 0x00,
        0x02,

        0x60, 0x60,
        0x00,
        0x01, 0x00,
        0x03};
    ec_SDOwrite(slave, 0x8003, 0x00, TRUE, sizeof(_8003), &_8003, EC_TIMEOUTRXM);

    while (EcatError)
        logger.warn("%s", ec_elist2string());

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
    int po2so_config = 0;

    // Vars
    // =======================================================
    uint16_t slave_canopen_id = 255;
    int expectedWKC = 0;
    uint8 IOmap[4096]; // I/O map for PDOs

    Beckhoff() : Node("beckhoff")
    {
        this->declare_parameter("if_name", "eth0");
        this->get_parameter("if_name", if_name);

        this->declare_parameter("po2so_config", 0);
        this->get_parameter("po2so_config", po2so_config);

        if (!logger.init())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize logger");
            rclcpp::shutdown();
        }

        if (init_beckhoff() > 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize Beckhoff");
            rclcpp::shutdown();
        }

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
                    logger.warn("%s", ec_elist2string());

                logger.info("%d slaves found and configured.", ec_slavecount);

                for (uint8_t slave = 1; slave <= ec_slavecount; slave++)
                {
                    switch (ec_slave[slave].eep_id)
                    {
                    case EL6751_ID:
                        slave_canopen_id = slave;
                        logger.info("Slave Canopen ID: %d", slave_canopen_id);
                        break;
                    }
                }

                if (slave_canopen_id == 255)
                {
                    logger.warn("No slave found with Canopen ID");
                    return 4;
                }

                expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
                logger.info("Calculated workcounter %d", expectedWKC);

                // ============= CONFIGURE STARTUP STATE =============
                if (1 == po2so_config)
                {
                    ec_slave[slave_canopen_id].PO2SOconfig = &scan_CANopen_Slaves;
                }
                else if (2 == po2so_config)
                {
                    ec_slave[slave_canopen_id].PO2SOconfig = &remove_CAN_eeprom;
                }
                else if (3 == po2so_config)
                {
                    ec_slave[slave_canopen_id].PO2SOconfig = &init_CAN_Startup;
                }
                // ===================================================

                ec_config_map(&IOmap);
                ec_configdc();

                ec_slave[0].state = EC_STATE_SAFE_OP;
                ec_writestate(0);

                while (ec_iserror())
                {
                    logger.warn("%s", ec_elist2string());
                }

                ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 3);
                if (ec_slave[0].state != EC_STATE_SAFE_OP)
                {
                    logger.warn("Not all slaves reached safe operational state.");
                    ec_readstate();
                    for (int i = 1; i <= ec_slavecount; i++)
                    {
                        if (ec_slave[i].state != EC_STATE_SAFE_OP)
                        {
                            logger.warn("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s", i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                        }
                    }
                }

                ec_readstate();
                ec_slave[0].state = EC_STATE_OPERATIONAL;
                ec_writestate(0);

                if (ec_slave[0].state == EC_STATE_OPERATIONAL)
                {
                    logger.info("Operational state reached for all slaves.");

                    ec_send_processdata();
                    ec_receive_processdata(EC_TIMEOUTRET);
                }

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