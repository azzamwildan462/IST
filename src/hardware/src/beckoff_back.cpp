#include "rclcpp/rclcpp.hpp"
#include "ros2_utils/global_definitions.hpp"
#include "ros2_utils/help_logger.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

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
#define EL1889_ID 0x0b4f3052 // Digital input
#define JIAYU_ID 0x00000001  // Motor driver

#define ANALOG_OUT_SCALER 3276.8f
#define ANALOG_INPUT_SCALER 0.0003051757812f
#define LSB_VALUE 0.0196078431f

#define ADDRESS_CONTROL_WORD 0x6040
#define ADDRESS_STATUS_WORD 0x6041
#define ADDRESS_MODES_OF_OPERATION 0x6060
#define ADDRESS_ACTUAL_VELOCITY 0x6069
#define ADDRESS_ACTUAL_TORQUE 0x6077
#define ADDRESS_TARGET_VELOCITY 0x60FF
#define ADDRESS_CURRENT_POSITION 0x6063

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

PACKED_BEGIN
typedef struct
{
    uint16_t data;
} digital_in_t;
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
    // uint16_t _1c12_val[2] = { 0x01, 0x1600 };
    // uint16_t _1c13_val[2] = { 0x01, 0x1a00 };

    uint16_t ret_val = 0;
    ret_val += ec_SDOwrite(slave, 0x1c12, 0x00, TRUE, sizeof(_1c12_zero), _1c12_zero, EC_TIMEOUTRXM);
    ret_val += ec_SDOwrite(slave, 0x1c13, 0x00, TRUE, sizeof(_1c13_zero), _1c13_zero, EC_TIMEOUTRXM);
    // ret_val += ec_SDOwrite(slave, 0x1c12, 0x00, TRUE, sizeof(_1c12_val), _1c12_val, EC_TIMEOUTRXM);
    // ret_val += ec_SDOwrite(slave, 0x1c13, 0x00, TRUE, sizeof(_1c13_val), _1c13_val, EC_TIMEOUTRXM);

    return 1;
}

class Beckhoff : public rclcpp::Node
{
public:
    rclcpp::TimerBase::SharedPtr tim_routine;
    rclcpp::TimerBase::SharedPtr tim_control_brake;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_sensors;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_error_code;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_analog_input;
    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr pub_digital_input;
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
    digital_in_t *digital_in;

    // Vars
    // =======================================================
    uint16_t slave_canopen_id = 255;
    uint16_t brake_slave_id = 255;
    int expectedWKC = 0;
    uint8 IOmap[10240]; // I/O map for PDOs

    int16_t transmission_master = 0;  // Ini di-trigger manual dari joystick untuk testing
    float master_target_volt_hat = 0; // turunan dari voltage
    float master_target_steering = 0; // rad // Ini di-handle CANbus_HAL
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
    uint8_t accelerator_switch = 0;

    uint8_t fsm_brake_driver = 0;

    uint16_t control_word = 0;
    uint16_t status_word = 0;
    int status_word_size = sizeof(status_word);
    int32_t output_velocity_brake = 0;
    int16_t current_torque_brake = 0;
    int current_torque_brake_size = sizeof(current_torque_brake);
    int8_t brake_mode = 9; // 8: Position, 9: Velocity
    int32_t local_brake_position = 0;
    int32_t current_brake_position = 0;
    int32_t prev_brake_position = 0;
    int current_brake_position_size = sizeof(current_brake_position);
    uint8_t status_braking = 0;
    uint8_t prev_status_braking = 0;
    uint16_t counter_unbrake = 0;

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
        tim_routine = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&Beckhoff::callback_tim_routine, this));
        tim_control_brake = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Beckhoff::callback_tim_control_brake, this));

        //----Publisher
        pub_sensors = this->create_publisher<std_msgs::msg::Float32MultiArray>("/beckhoff/sensors", 1);
        pub_error_code = this->create_publisher<std_msgs::msg::Int16>("/beckhoff/error_code", 1);
        pub_analog_input = this->create_publisher<std_msgs::msg::Float32MultiArray>("/beckhoff/analog_input", 1);
        pub_digital_input = this->create_publisher<std_msgs::msg::UInt8MultiArray>("/beckhoff/digital_input", 1);

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

        if (buffer_dac_velocity < 0.41)
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
            accelerator_switch = 1;
        }
        else if (counter_plus_velocity > 0)
        {
            accelerator_switch = 3;
        }
    }

    void callback_tim_control_brake()
    {
        if (brake_slave_id == 255)
        {
            return;
        }
        if (brake_jiayu() > 0)
        {
            logger.warn("Brake Jiayu error");
        }
    }

    void callback_tim_routine()
    {
        if (brake_slave_id != 255 && fsm_brake_driver == 3)
        {
            (void)ec_SDOread(brake_slave_id, ADDRESS_STATUS_WORD, 0x00, FALSE, &status_word_size, &status_word, EC_TIMEOUTRXM);
            (void)ec_SDOread(brake_slave_id, ADDRESS_ACTUAL_TORQUE, 0x00, FALSE, &current_torque_brake_size, &current_torque_brake, EC_TIMEOUTRXM);
            (void)ec_SDOwrite(brake_slave_id, ADDRESS_MODES_OF_OPERATION, 0x00, FALSE, sizeof(brake_mode), &brake_mode, EC_TIMEOUTRXM);
            (void)ec_SDOwrite(brake_slave_id, ADDRESS_TARGET_VELOCITY, 0x00, FALSE, sizeof(output_velocity_brake), &output_velocity_brake, EC_TIMEOUTRXM);
            (void)ec_SDOread(brake_slave_id, ADDRESS_CURRENT_POSITION, 0x00, FALSE, &current_brake_position_size, &current_brake_position, EC_TIMEOUTRXM);
        }
        else if (brake_slave_id != 255 && fsm_brake_driver != 3)
        {
            return;
        }

        ec_send_processdata();
        int wkc = ec_receive_processdata(EC_TIMEOUTRET);

        if (wkc >= expectedWKC)
        {
            counter_beckhoff_disconnect = 0;
            // test_analog_output();
            // test_digital_output();
            // test_analog_input();

            // ===================================================================================

            // Ketika brake
            if (master_target_volt_hat < -1.0)
            {
                status_braking = 1;
                counter_unbrake = 0;
                static const float brake_minimum = -0.1;
                static const float brake_maximum = -3.0;
                static const float brake_torq_minimum = 0.0;
                static const float brake_torq_maximum = 7.0;
                static const float brake_max_velocity = 1500.0;

                float current_torq = 0;
                if (brake_slave_id != 255)
                    current_torq = (float)current_torque_brake;

                float normalized_brake = (master_target_volt_hat - brake_minimum) / (brake_maximum - brake_minimum);
                float normalized_torq = (current_torq - brake_torq_minimum) / (brake_torq_maximum - brake_torq_minimum);

                // float error_brake = normalized_brake - normalized_torq;
                // int32_t output_velocity = (int32_t)(error_brake * brake_max_velocity);

                // logger.info("Brake: (%d) %.2f %.2f %.2f %.2f %d -> %d", local_brake_position, master_target_volt_hat, current_torq, normalized_brake, normalized_torq, output_velocity_brake, output_velocity);

                static float p = 0;
                static float i = 0;
                static float d = 0;

                static const float kp = 400;
                static const float ki = 0;
                static const float kd = 0;

                float target_torq = normalized_brake * brake_torq_maximum;
                float error = target_torq - current_torq;

                p = kp * error;
                i += ki * error;

                if (i > 10)
                    i = 10;
                else if (i < -10)
                    i = -10;

                float output_velocity = (int32_t)(p + i);

                if (output_velocity > brake_max_velocity)
                    output_velocity = brake_max_velocity;
                else if (output_velocity < -brake_max_velocity)
                    output_velocity = -brake_max_velocity;

                // logger.info("Brake: %.2f %.2f %.2f %d -> %.2f", current_torq, target_torq, error, output_velocity_brake, output_velocity);

                if (brake_slave_id != 255)
                    output_velocity_brake = output_velocity;
            }
            else
            {
                status_braking = 0;
                static const float unbrake_torq_minimum = 0.0;
                static const float unbrake_torq_maximum = 7.0;
                static const float unbrake_max_velocity = -2000.0;

                float current_torq = 0;
                if (brake_slave_id != 255)
                    current_torq = (float)current_torque_brake;

                float normalized_torq = (current_torq - unbrake_torq_minimum) / (unbrake_torq_maximum - unbrake_torq_minimum);

                int32_t output_velocity = (int32_t)(normalized_torq * unbrake_max_velocity);

                if (fabs(current_torq) < 2.0)
                {
                    counter_unbrake++;
                }
                else
                {
                    counter_unbrake -= 20;

                    if ((int16_t)counter_unbrake < 0)
                        counter_unbrake = 0;
                }

                if (counter_unbrake > 1000)
                {
                    counter_unbrake = 1000;
                }

                // logger.info("UnBrake: (%d %d) (%d) %.2f %.2f %.2f %d -> %d -> %.2f", counter_unbrake, status_braking, local_brake_position, master_target_volt_hat, current_torq, normalized_torq, output_velocity_brake, output_velocity, buffer_dac_velocity);

                if (brake_slave_id != 255)
                    output_velocity_brake = output_velocity;
            }

            // ===================================================================================

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

                    // if (fb_throttle_velocity_volt > 0.435)
                    //     digital_out->data |= DO_SWITCH_THROTTLE;
                    // else if (fb_throttle_velocity_volt > 0.415 && fb_throttle_velocity_volt < 0.435)
                    //     digital_out->data &= ~DO_SWITCH_THROTTLE;
                    // else
                    // {
                    // }
                    if (accelerator_switch == 3)
                        digital_out->data |= DO_SWITCH_THROTTLE;
                    else if (accelerator_switch == 1)
                        digital_out->data &= ~DO_SWITCH_THROTTLE;
                }
                // Reverse
                else if (transmission_master == 5)
                {
                    digital_out->data &= ~DO_TRANSMISSION_FORWARD;
                    digital_out->data &= ~DO_TRANSMISSION_NEUTRAL;
                    digital_out->data |= DO_TRANSMISSION_REVERSE;

                    // if (fb_throttle_velocity_volt > 0.435)
                    //     digital_out->data |= DO_SWITCH_THROTTLE;
                    // else if (fb_throttle_velocity_volt > 0.415 && fb_throttle_velocity_volt < 0.435)
                    //     digital_out->data &= ~DO_SWITCH_THROTTLE;
                    // else
                    // {
                    // }
                    if (accelerator_switch == 3)
                        digital_out->data |= DO_SWITCH_THROTTLE;
                    else if (accelerator_switch == 1)
                        digital_out->data &= ~DO_SWITCH_THROTTLE;
                }
            }
            else
            {
                // Netral
                if (accelerator_switch == 1)
                {
                    digital_out->data &= ~DO_SWITCH_THROTTLE;
                    digital_out->data &= ~DO_TRANSMISSION_FORWARD;
                    digital_out->data |= DO_TRANSMISSION_NEUTRAL;
                    digital_out->data &= ~DO_TRANSMISSION_REVERSE;
                }
                // Forward
                else if (accelerator_switch == 3)
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
            if (fb_throttle_velocity_volt > 0.415)
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
            if (accelerator_switch == 3)
            {
                dac_velocity_send = buffer_dac_velocity;
            }

            analog_output->data_2 = (int16_t)(dac_velocity_send * ANALOG_OUT_SCALER);
            // logger.info("INFO: %.2f || %.2f %.2f %d", fb_throttle_velocity_volt, buffer_dac_velocity, dac_velocity_send, digital_out->data);

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

        int32_t diff_brake_position = current_brake_position - prev_brake_position;
        if (diff_brake_position > 20000 || diff_brake_position < -20000)
        {
            logger.error("Brake position error");
            diff_brake_position = 0;
        }

        local_brake_position += diff_brake_position;

        prev_brake_position = current_brake_position;

        std_msgs::msg::Int16 msg_error_code;
        msg_error_code.data = error_code;
        pub_error_code->publish(msg_error_code);
    }

    int8_t brake_jiayu()
    {
        // uint16_t sword = if_brake_input->status_word;
        (void)ec_SDOread(brake_slave_id, ADDRESS_STATUS_WORD, 0x00, FALSE, &status_word_size, &status_word, EC_TIMEOUTRXM);
        uint16_t sword = status_word;
        static uint16_t last_sword = sword;

        if (0 == sword && 0 < last_sword)
        {
            fsm_brake_driver = 0;
            return 1;
        }

        last_sword = sword;

        if (3 == fsm_brake_driver)
        {
            if (status_braking == 0)
            {
                control_word = 7;
            }
            else if (status_braking == 1)
            {
                control_word = 15;
            }
            // logger.info("BRAKING SWORD: %d CWORD: %d", status_word, control_word);

            (void)ec_SDOwrite(brake_slave_id, ADDRESS_CONTROL_WORD, 0x00, FALSE, sizeof(control_word), &control_word, EC_TIMEOUTRXM);

            return 0;
        }

        if (33 == sword)
        {
            control_word = 7;
        }
        else if (35 == sword || 55 == sword)
        {
            control_word = 15;
        }
        else if (5216 == sword || 1088 == sword)
        {
            // if_brake_output->control_word = 0x06;
            control_word = 6;
            fsm_brake_driver = 1;
        }
        else if (5153 == sword || 1057 == sword || 4129 == sword)
        {
            // if_brake_output->control_word = 0x07;
            control_word = 7;
            fsm_brake_driver = 2;
        }
        else if (5155 == sword || 1059 == sword || 4131 == sword)
        {
            // if_brake_output->control_word = 0x0f;
            control_word = 15;
            fsm_brake_driver = 3;
        }
        else
        {
            // if_brake_output->control_word = 0x06;
            control_word = 7;
            fsm_brake_driver = 0;
        }

        (void)ec_SDOwrite(brake_slave_id, ADDRESS_MODES_OF_OPERATION, 0x00, FALSE, sizeof(brake_mode), &brake_mode, EC_TIMEOUTRXM);
        (void)ec_SDOwrite(brake_slave_id, ADDRESS_CONTROL_WORD, 0x00, FALSE, sizeof(control_word), &control_word, EC_TIMEOUTRXM);
        logger.info("SWORD: %d CWORD: %d", status_word, control_word);

        return 0;
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

        // std_msgs::msg::UInt8MultiArray msg_digital_input;
        // msg_digital_input.data.push_back(digital_in->data & 0xFF);
        // msg_digital_input.data.push_back((digital_in->data >> 8) & 0xFF);
        // pub_digital_input->publish(msg_digital_input);

        std_msgs::msg::UInt8MultiArray msg_digital_input;
        msg_digital_input.data.push_back(0);
        msg_digital_input.data.push_back(0);
        pub_digital_input->publish(msg_digital_input);
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
                    // return 4;
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
                else if (4 == po2so_config)
                {
                    if (brake_slave_id != 255)
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
                            // if_brake_input = (if_brake_input_t*)ec_slave[slave].inputs;
                            // if_brake_output = (if_brake_output_t*)ec_slave[slave].outputs;
                            logger.info("JIAYU Found & Configured");
                            break;

                        case EL1889_ID:
                            digital_in = (digital_in_t *)ec_slave[slave].inputs;
                            logger.info("EL1889 Configured");
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

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node_beckhoff);
    executor.spin();

    return 0;
}