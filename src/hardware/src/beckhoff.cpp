#include "rclcpp/rclcpp.hpp"
#include "ros2_utils/global_definitions.hpp"
#include "ros2_utils/help_logger.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/string.hpp"

#include <soem/ethercat.h>

#define EC_TIMEOUTMON 5000

#define DO_SWITCH_THROTTLE 0b10000
#define DO_TRANSMISSION_NEUTRAL 0b100
#define DO_TRANSMISSION_REVERSE 0b001
#define DO_TRANSMISSION_FORWARD 0b010

#define EL6751_ID 0x1a5f3052 // CANopen
#define EL2889_ID 0x0b493052 // Digital output
#define EL3068_ID 0x0bfc3052 // Analog input
#define EL4004_ID 0x0fa43052 // Analog output
#define JIAYU_ID 0x00000001  // Motor driver

#define ANALOG_OUT_SCALER 3276.8f
#define ANALOG_INPUT_SCALER 0.0003051757812f
#define LSB_VALUE 0.0196078431f

PACKED_BEGIN
typedef struct
{
    uint16_t data;
} digital_out_t;
PACKED_END

PACKED_BEGIN
typedef struct
{
    uint16_t header_1;
    int16_t data_1;
    uint16_t header_2;
    int16_t data_2;
    uint16_t header_3;
    int16_t data_3;
    uint16_t header_4;
    int16_t data_4;
    uint16_t header_5;
    int16_t data_5;
    uint16_t header_6;
    int16_t data_6;
    uint16_t header_7;
    int16_t data_7;
    uint16_t header_8;
    int16_t data_8;
} analog_input_t;
PACKED_END

PACKED_BEGIN
typedef struct
{
    int16_t data_1;
    int16_t data_2;
    int16_t data_3;
    int16_t data_4;
} analog_output_t;
PACKED_END

PACKED_BEGIN
typedef struct
{
    uint16_t control_word;
    int16_t target_torque;
    int32_t target_position;
    uint32_t homing_acceleration;
    uint16_t touch_probe_function;
    int32_t target_velocity;
    int8_t modes_of_operation;
} if_brake_output_t;
PACKED_END

PACKED_BEGIN
typedef struct
{
    uint16_t error_code;
    uint16_t status_word;
    int32_t position_actual_value;
    int32_t velocity_actual_value;
    int8_t modes_of_operation_display;
    int16_t torque_actual_value;
    uint16_t touch_probe_status;
    int32_t touch_probe_pos1_pos_value;
    int32_t touch_probe_pos2_pos_value;
    uint32_t digital_inputs;
} if_brake_input_t;
PACKED_END

HelpLogger logger;

/**
 * Scan and save to EEPROM the CANopen slaves configuration
 */
int scan_CANopen_Slaves(uint16_t slave)
{
    // uint8_t _f002_1[2] = {0x01, 0x00};
    // ec_SDOwrite(slave, 0xf002, 0x01, FALSE, sizeof(_f002_1), &_f002_1, EC_TIMEOUTRXM);
    uint8_t _f002_1[2] = {0x01, 0x04};
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
    uint32_t _f921 = 0x65766173;
    ec_SDOwrite(slave, 0xf921, 0x01, TRUE, sizeof(_f921), &_f921, EC_TIMEOUTRXM);

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

int init_Brake_Driver(uint16_t slave)
{
    uint16_t _1c12_zero[2] = {0x00, 0x00};
    uint16_t _1c13_zero[2] = {0x00, 0x00};
    uint16_t _1c12_val[2] = {0x01, 0x1600};
    uint16_t _1c13_val[2] = {0x01, 0x1a00};

    uint16_t ret_val = 0;
    ret_val += ec_SDOwrite(slave, 0x1c12, 0x00, TRUE, sizeof(_1c12_zero), _1c12_zero, EC_TIMEOUTRXM);
    ret_val += ec_SDOwrite(slave, 0x1c13, 0x00, TRUE, sizeof(_1c13_zero), _1c13_zero, EC_TIMEOUTRXM);
    ret_val += ec_SDOwrite(slave, 0x1c12, 0x00, TRUE, sizeof(_1c12_val), _1c12_val, EC_TIMEOUTRXM);
    ret_val += ec_SDOwrite(slave, 0x1c13, 0x00, TRUE, sizeof(_1c13_val), _1c13_val, EC_TIMEOUTRXM);

    return 1;
}

class Beckhoff : public rclcpp::Node
{
public:
    rclcpp::TimerBase::SharedPtr tim_50hz;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_sensors;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_error_code;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_analog_input;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_master_actuator;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_master_local_fsm;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_master_global_fsm;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_transmission_master;

    // Configs
    // =======================================================
    std::string if_name;
    int po2so_config = 0;
    float speed_to_volt = 0.7;
    float dac_velocity_minimum = 0.4;
    float dac_velocity_maximum = 1.2;

    // DAta raw beckhoff
    // =======================================================
    digital_out_t *digital_out;
    analog_input_t *analog_input;
    analog_output_t *analog_output;
    if_brake_output_t *if_brake_output;
    if_brake_input_t *if_brake_input;

    // Vars
    // =======================================================
    uint16_t slave_canopen_id = 255;
    uint16_t brake_slave_id = 255;
    int expectedWKC = 0;
    uint8 IOmap[10240]; // I/O map for PDOs

    int16_t transmission_master = 0;  // Ini di-trigger manual dari joystick untuk testing
    float master_target_volt_hat = 0; // turunan dari voltage
    float master_target_steering = 0; // rad
    int16_t master_local_fsm = 0;
    int16_t master_global_fsm = 0;
    float lpf_velocity_target_voltage = 0.0;
    float buffer_dac_velocity = 0.0;

    int16_t error_code = 0;

    float fb_steering_angle = 0.3;
    float fb_throttle_velocity_volt = 0.0;
    uint8_t fb_foot_brake_switch = 0;
    uint8_t fb_foot_velocity_switch = 0;

    uint32_t counter_beckhoff_disconnect = 0;
    uint8_t auto_transmission = 0;

    Beckhoff()
        : Node("beckhoff")
    {
        this->declare_parameter("if_name", "eth0");
        this->get_parameter("if_name", if_name);

        this->declare_parameter("po2so_config", 0);
        this->get_parameter("po2so_config", po2so_config);

        this->declare_parameter("speed_to_volt", 0.7);
        this->get_parameter("speed_to_volt", speed_to_volt);

        this->declare_parameter("dac_velocity_minimum", 0.4);
        this->get_parameter("dac_velocity_minimum", dac_velocity_minimum);

        this->declare_parameter("dac_velocity_maximum", 1.2);
        this->get_parameter("dac_velocity_maximum", dac_velocity_maximum);

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
        tim_50hz = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&Beckhoff::callback_tim_50hz, this));

        //----Publisher
        pub_sensors = this->create_publisher<std_msgs::msg::Float32MultiArray>("/beckhoff/sensors", 1);
        pub_error_code = this->create_publisher<std_msgs::msg::Int16>("/beckhoff/error_code", 1);
        pub_analog_input = this->create_publisher<std_msgs::msg::Float32MultiArray>("/beckhoff/analog_input", 1);

        //----Subscriber
        sub_master_actuator = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/master/actuator", 1, std::bind(&Beckhoff::callback_sub_master_actuator, this, std::placeholders::_1));
        sub_master_local_fsm = this->create_subscription<std_msgs::msg::Int16>(
            "/master/local_fsm", 1, std::bind(&Beckhoff::callback_sub_master_local_fsm, this, std::placeholders::_1));
        sub_master_global_fsm = this->create_subscription<std_msgs::msg::Int16>(
            "/master/global_fsm", 1, std::bind(&Beckhoff::callback_sub_master_global_fsm, this, std::placeholders::_1));
        sub_transmission_master = this->create_subscription<std_msgs::msg::Int16>(
            "/master/transmission", 1, std::bind(&Beckhoff::callback_sub_transmission_master, this, std::placeholders::_1));
    }

    void callback_sub_transmission_master(const std_msgs::msg::Int16::SharedPtr msg)
    {
        transmission_master = msg->data;
    }

    void callback_sub_master_local_fsm(const std_msgs::msg::Int16::SharedPtr msg)
    {
        master_local_fsm = msg->data;
    }

    void callback_sub_master_global_fsm(const std_msgs::msg::Int16::SharedPtr msg)
    {
        master_global_fsm = msg->data;
    }

    void callback_sub_master_actuator(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        static uint16_t counter_zero_velocity = 0;
        static uint16_t counter_plus_velocity = 0;
        master_target_volt_hat = msg->data[0];
        master_target_steering = msg->data[1];

        if (buffer_dac_velocity < 0.45)
        {
            counter_zero_velocity++;
            counter_plus_velocity = 0;
        }
        else
        {
            counter_zero_velocity = 0;
            counter_plus_velocity++;
        }

        if (counter_zero_velocity > 1)
        {
            auto_transmission = 1;
        }
        else if (counter_plus_velocity > 0)
        {
            auto_transmission = 3;
        }
    }

    void callback_tim_50hz()
    {
        ec_send_processdata();
        int wkc = ec_receive_processdata(EC_TIMEOUTRET);

        uint8_t _6060 = 9;
        (void)ec_SDOwrite(brake_slave_id, 0x6060, 0x00, FALSE, sizeof(_6060), &_6060, EC_TIMEOUTRXM);

        if (wkc >= expectedWKC)
        {
            counter_beckhoff_disconnect = 0;
            // test_analog_output();
            // test_digital_output();
            test_analog_input();

            if (transmission_master > 0)
            {
                // Netral
                if (transmission_master == 1)
                {
                    digital_out->data &= ~DO_SWITCH_THROTTLE;
                    digital_out->data &= ~DO_TRANSMISSION_FORWARD;
                    digital_out->data &= ~DO_TRANSMISSION_REVERSE;
                    digital_out->data |= DO_TRANSMISSION_NEUTRAL;
                }
                // Forward
                else if (transmission_master == 3)
                {
                    digital_out->data &= ~DO_TRANSMISSION_NEUTRAL;
                    digital_out->data &= ~DO_TRANSMISSION_REVERSE;
                    digital_out->data |= DO_TRANSMISSION_FORWARD;

                    if (auto_transmission == 3)
                        digital_out->data |= DO_SWITCH_THROTTLE;
                    else if (auto_transmission == 1)
                        digital_out->data &= ~DO_SWITCH_THROTTLE;
                }
                // Reverse
                else if (transmission_master == 5)
                {
                    digital_out->data &= ~DO_TRANSMISSION_FORWARD;
                    digital_out->data &= ~DO_TRANSMISSION_NEUTRAL;
                    digital_out->data |= DO_TRANSMISSION_REVERSE;

                    if (auto_transmission == 3)
                        digital_out->data |= DO_SWITCH_THROTTLE;
                    else if (auto_transmission == 1)
                        digital_out->data &= ~DO_SWITCH_THROTTLE;
                }
            }
            else
            {
                // Netral
                if (auto_transmission == 1)
                {
                    digital_out->data &= ~DO_SWITCH_THROTTLE;
                    digital_out->data &= ~DO_TRANSMISSION_FORWARD;
                    digital_out->data &= ~DO_TRANSMISSION_REVERSE;
                    digital_out->data |= DO_TRANSMISSION_NEUTRAL;
                }
                // Forward
                else if (auto_transmission == 3)
                {
                    digital_out->data &= ~DO_TRANSMISSION_NEUTRAL;
                    digital_out->data &= ~DO_TRANSMISSION_REVERSE;
                    digital_out->data |= DO_SWITCH_THROTTLE;
                    digital_out->data |= DO_TRANSMISSION_FORWARD;
                }
            }

            // ===================================================================================

            fb_throttle_velocity_volt = (float)analog_input->data_2 * ANALOG_INPUT_SCALER;

            // Ketika throttle ditekan, maka ikut throttle
            if (fb_throttle_velocity_volt > 0.5)
            {
                buffer_dac_velocity = fb_throttle_velocity_volt;
            }
            else
            {
                buffer_dac_velocity += master_target_volt_hat;
            }

            if (buffer_dac_velocity <= dac_velocity_minimum)
                buffer_dac_velocity = dac_velocity_minimum;
            else if (buffer_dac_velocity >= dac_velocity_maximum)
                buffer_dac_velocity = dac_velocity_maximum;

            float dac_velocity_send = 0.40;

            // Aktifkan relay dulu lalu beri throttle
            if (auto_transmission == 3)
            {
                dac_velocity_send = buffer_dac_velocity;
            }

            analog_output->data_2 = (int16_t)(dac_velocity_send * ANALOG_OUT_SCALER);
            logger.info("INFO: %.2f || %.2f %.2f %d", fb_throttle_velocity_volt, buffer_dac_velocity, dac_velocity_send, digital_out->data);

            // ====================================================================================

            error_code = 0;

            transmit_all();
        }
        else
        {
            counter_beckhoff_disconnect++;
            error_code = 1;
        }

        if (counter_beckhoff_disconnect > 100)
        {
            logger.error("Computer LAN Wiring error");
            rclcpp::shutdown();
        }

        std_msgs::msg::Int16 msg_error_code;
        msg_error_code.data = error_code;
        pub_error_code->publish(msg_error_code);
    }

    void transmit_all()
    {
        std_msgs::msg::Float32MultiArray msg_sensors;
        msg_sensors.data.push_back(fb_steering_angle);
        pub_sensors->publish(msg_sensors);

        std_msgs::msg::Float32MultiArray msg_analog_input;
        msg_analog_input.data.push_back((float)analog_input->data_1 * ANALOG_INPUT_SCALER);
        msg_analog_input.data.push_back((float)analog_input->data_2 * ANALOG_INPUT_SCALER);
        msg_analog_input.data.push_back((float)analog_input->data_3 * ANALOG_INPUT_SCALER);
        msg_analog_input.data.push_back((float)analog_input->data_4 * ANALOG_INPUT_SCALER);
        msg_analog_input.data.push_back((float)analog_input->data_5 * ANALOG_INPUT_SCALER);
        msg_analog_input.data.push_back((float)analog_input->data_6 * ANALOG_INPUT_SCALER);
        msg_analog_input.data.push_back((float)analog_input->data_7 * ANALOG_INPUT_SCALER);
        msg_analog_input.data.push_back((float)analog_input->data_8 * ANALOG_INPUT_SCALER);
        pub_analog_input->publish(msg_analog_input);
    }

    void test_digital_output()
    {
        static uint16_t status_blink = 0b1010101010101010;
        static uint16_t cntr = 0;

        if (cntr++ > 100)
        {
            cntr = 0;
            digital_out->data = status_blink;
            status_blink = ~status_blink;
        }
    }

    void test_analog_output()
    {
        float voltase_target = 1.23;
        analog_output->data_1 = (int16_t)(voltase_target * ANALOG_OUT_SCALER);
        voltase_target = 2.34;
        analog_output->data_2 = (int16_t)(voltase_target * ANALOG_OUT_SCALER);
        voltase_target = 3.45;
        analog_output->data_3 = (int16_t)(voltase_target * ANALOG_OUT_SCALER);
        voltase_target = 4.56;
        analog_output->data_4 = (int16_t)(voltase_target * ANALOG_OUT_SCALER);
    }

    void test_analog_input()
    {
        float adc_1 = (float)analog_input->data_1 * ANALOG_INPUT_SCALER;
        (void)adc_1;
    }

    uint8_t set_watchdog(uint16_t slave, uint16_t watchdog_time)
    {
        uint16_t multiplier;
        int wc = ec_FPRD(ec_slave[slave].configadr, 0x400, 4, &multiplier, EC_TIMEOUTRXM);

        if (wc > 0)
        {
            logger.info("Watchdog time multiplier: %d", multiplier);
            double sf = (1.0 / 25.0) * (double)(multiplier + 2) / 1000.0;
            uint16_t wv = (uint16_t)((double)watchdog_time / sf);

            wc = ec_FPWR(ec_slave[slave].configadr, 0x420, 4, &wv, EC_TIMEOUTRXM);

            if (wc > 0)
            {
                logger.info("Watchdog time set to %d ms", watchdog_time);
            }
            else
            {
                logger.error("Failed to set watchdog time\n");
                return 0;
            }
        }
        else
        {
            logger.error("Failed to read watchdog time multiplier\n");
            return 0;
        }

        while (EcatError)
        {
            logger.warn("%s", ec_elist2string());
            return 0;
        }

        return 1;
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
                    case JIAYU_ID:
                        brake_slave_id = slave;
                        logger.info("Brake Driver ID: %d", brake_slave_id);
                        break;
                    }
                }

                if (brake_slave_id == 255)
                {
                    logger.warn("No slave found with brake driver ID");
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

                if (3 == po2so_config)
                {
                    ec_slave[brake_slave_id].PO2SOconfig = &init_Brake_Driver;
                }
                // =============== CONFIGURE WATCHDOG ================
                for (uint8_t slave = 1; slave <= ec_slavecount; slave++)
                {
                    logger.info("Setting watchdog for slave %d", slave);
                    set_watchdog(slave, 100);
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

                    for (uint8_t slave = 1; slave <= ec_slavecount; slave++)
                    {
                        switch (ec_slave[slave].eep_id)
                        {
                        case EL2889_ID:
                            digital_out = (digital_out_t *)ec_slave[slave].outputs;
                            logger.info("EL2889 Configured");
                            break;

                        case EL3068_ID:
                            analog_input = (analog_input_t *)ec_slave[slave].inputs;
                            logger.info("EL3068 Configured");
                            break;

                        case EL4004_ID:
                            analog_output = (analog_output_t *)ec_slave[slave].outputs;
                            logger.info("EL4004 Configured");
                            break;

                        case JIAYU_ID:
                            if_brake_input = (if_brake_input_t *)ec_slave[slave].inputs;
                            if_brake_output = (if_brake_output_t *)ec_slave[slave].outputs;
                            logger.info("JIAYU Found & Configured");
                            break;
                        }
                    }
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