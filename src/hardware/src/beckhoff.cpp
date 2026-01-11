// asd
#include "rclcpp/rclcpp.hpp"
#include "ros2_utils/global_definitions.hpp"
#include "ros2_utils/help_logger.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

#include <soem/ethercat.h>

#define EC_TIMEOUTMON 5000

#define DO_SWITCH_THROTTLE 0b10000
#define DO_TRANSMISSION_NEUTRAL 0b100
#define DO_TRANSMISSION_REVERSE 0b001
#define DO_TRANSMISSION_FORWARD 0b010
#define DO_BUZZER_TOWING 0b1000
#define DO_LAMPU_BAWAAN 0b100000000
#define DO_LAMPU_BELAKANG_MERAH 0b1000000000000
#define DO_LAMPU_BELAKANG_KUNING 0b10000000000000
#define DO_LAMPU_BELAKANG_HIJAU 0b100000000000000
#define DO_BUZZER_BELAKANG 0b1000000000000000
#define DO_SIRINE 0b1000000000
#define DO_LAMPU_REM_BELAKANG 0b100000000000
#define DO_SEIN_KIRI 0b100000
#define DO_SEIN_KANAN 0b1000000

#define EL6751_ID 0x1a5f3052 // CANopen
#define EL2889_ID 0x0b493052 // Digital output
#define EL3068_ID 0x0bfc3052 // Analog input
#define EL4004_ID 0x0fa43052 // Analog output
#define EL4104_ID 0x10083052 // Analog output
#define EL1889_ID 0x07613052 // Digital input
#define JIAYU_ID 0x00000001  // Motor driver
#define SLAVE_ID_DI_1 0x07
#define SLAVE_ID_DI_2 0x08
#define EPS_MRI_IST_ID 0x00006969

#define EPS_ADDRES_KP_TORQUE 0x2000
#define EPS_ADDRES_KI_TORQUE 0x2001
#define EPS_ADDRES_KD_TORQUE 0x2002
#define EPS_ADDRES_KP_POS 0x2003
#define EPS_ADDRES_KI_POS 0x2004
#define EPS_ADDRES_KD_POS 0x2005
#define EPS_ADDRES_KP_VEL 0x2006
#define EPS_ADDRES_KI_VEL 0x2007
#define EPS_ADDRES_KD_VEL 0x2008
#define EPS_ADDRESS_TOR_OFFSET 0x2009
#define EPS_ADDRESS_MIN_TOR_VAL 0x200A
#define EPS_ADDRESS_VOLTAGE_SCALE 0x200B
#define EPS_ADDRESS_CURRENT_OFFSET 0x200C
#define EPS_ADDRESS_CURRENT_SCALE 0x200D
#define EPS_ADDRESS_TOR_LPF_CUTOFF_HZ 0x200E
#define EPS_ADDRESS_CUR_LPF_CUTOFF_HZ 0x200F
#define EPS_ADDRESS_CURRENT_LIMIT 0x2010
#define EPS_ADDRESS_POS_LIMIT 0x2011
#define EPS_ADDRESS_VEL_ACCEL 0x2012
#define EPS_ADDRESS_PID_POS_OUT_LIM 0x2013
#define EPS_ADDRESS_PID_VEL_OUT_LIM 0x2014
#define EPS_ADDRESS_PID_TOR_OUT_LIM 0x2015
#define EPS_ADDRESS_DEADBAND_PWM 0x2016
#define EPS_ADDRESS_LIMIT_PERUBAHAN_PWM 0x2017

#define ANALOG_OUT_SCALER 3276.8f
#define ANALOG_INPUT_SCALER 0.0003051757812f
#define LSB_VALUE 0.0196078431f

#define ADDRESS_CONTROL_WORD 0x6040
#define ADDRESS_STATUS_WORD 0x6041
#define ADDRESS_MODES_OF_OPERATION 0x6060
#define ADDRESS_ACTUAL_VELOCITY 0x6069
#define ADDRESS_ACTUAL_TORQUE 0x6077
#define ADDRESS_TARGET_VELOCITY 0x60FF
#define ADDRESS_CURRENT_POSITION 0x6064

#define IN_TR_FORWARD 0b01
#define IN_TR_REVERSE 0b10
#define IN_BRAKE_ACTIVE 0b100
#define IN_EPS_nFAULT 0b1000
#define IN_MASK_BUMPER 0b11110000
// #define IN_START_OP3 0b100000000
// #define IN_STOP_OP3 0b1000000000
// #define IN_START_GAS_MANUAL 0b10000000000
#define IN_LS_BRAKE 0b1000000000000
#define IN_TRIM_KECEPATAN 0b10000000000000
#define IN_SELECTOR_SIRINE 0b10000000000000
#define IN_SELECTOR_DISABLE_SIRINE 0b100000000000000
#define IN_START_OP3_HANDLER 0b1000000000
#define IN_STOP_OP3_HANDLER 0b100000000
#define IN_LAMPU_HAZARD 0b100000000000
#define IN_LAMPU_TESTER 0b10000000000

#define IN_START_OP3 (0b100000 << 16)
#define IN_STOP_OP3 (0b100 << 16)
#define IN_START_GAS_MANUAL (0b1000 << 16)
#define IN_MANUAL_MUNDUR (0b100 << 16)
#define IN_MANUAL_MAJU (0b10 << 16)
#define IN_NEXT_TERMINAL (0b10000 << 16)
#define IN_SYSTEM_FULL_ENABLE (0b01 << 16)

#define EMERGENCY_LIDAR_DEPAN_DETECTED 0b010
#define EMERGENCY_CAMERA_OBS_DETECTED 0b100
#define EMERGENCY_GYRO_ANOMALY_DETECTED 0b1000
#define EMERGENCY_ICP_SCORE_TERLALU_BESAR 0b10000
#define EMERGENCY_ICP_TRANSLATE_TERLALU_BESAR 0b100000
#define EMERGENCY_STOP_KARENA_OBSTACLE 0b1000000
#define EMERGENCY_ALL_LIDAR_DETECTED 0b10000000
#define EMERGENCY_GANDENGAN_LEPAS 0b100000000
#define STATUS_TOWING_CONNECTED 0b01

PACKED_BEGIN
typedef struct PACKED
{
    float tar_pos_rad;
    float tar_vel_rad_s;
    uint16_t tar_pwm1;
    uint16_t tar_pwm2;
    uint8_t cmd_eps_mode;
} eps_output_t;
PACKED_END

PACKED_BEGIN
typedef struct PACKED
{
    float vsupply;
    float i_motor;
    float actual_torque_mv;
    float actual_pos_rad;
    float actual_vel_rad_s;
    float internal_temp;
    float ntc_temp;
    float ntc_volt;
    float vref_stm;
    uint16_t pwm1;
    uint16_t pwm2;
    uint8_t eps_mode;
    uint32_t eps_err_code;
} eps_input_t;
PACKED_END

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
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_fb_eps_mode;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_eps_encoder;
    rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr pub_eps_err_code;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_master_actuator;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_master_local_fsm;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_master_global_fsm;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_transmission_master;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_slam_status;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_master_camera_obs;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_master_lidar_obs;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_car_battery;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr sub_master_status_emergency;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_status_handrem;

    // Configs
    // =======================================================
    std::string if_name;
    int po2so_config = 0;
    float speed_to_volt = 0.7;
    float dac_velocity_minimum = 0.4;
    float dac_velocity_maximum = 1.2;
    int brake_idle_position = 0;
    int towing_berapa = 2;
    std::vector<double> k_pid_eps_torq_vel_pos;
    std::vector<double> k_eps_const;
    bool bypass_handrem_hw = false;
    bool disable_brake = false;
    float max_change_rad_eps_tar = 0.0026;

    // DAta raw beckhoff
    // =======================================================
    digital_out_t *digital_out;
    analog_input_t *analog_input;
    analog_output_t *analog_output;
    if_brake_output_t *if_brake_output;
    if_brake_input_t *if_brake_input;
    digital_in_t *digital_in1;
    digital_in_t *digital_in2;
    eps_output_t *eps_output;
    eps_input_t *eps_input;

    // Vars
    // =======================================================
    uint8_t status_handrem_dibawah = 0;
    uint16_t slave_canopen_id = 255;
    uint16_t brake_slave_id = 255;
    uint16_t eps_slave_id = 255;
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
    uint8_t fsm_brake_calibration = 0;

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
    int32_t brake_maximum_position = 0;
    float master_camera_obs = 0;
    float master_lidar_obs = 0;

    uint8_t master_slam_status = 0;

    rclcpp::Time last_time_update_can_mobil;
    rclcpp::Time current_time;
    bool status_mobil_connected = false;

    int16_t master_status_emergency = 0; // Status emergency dari master

    std::thread thread_routine;

    uint16_t digital_in1_data_kirim = 0;

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

        this->declare_parameter("brake_idle_position", 0);
        this->get_parameter("brake_idle_position", brake_idle_position);

        this->declare_parameter("towing_berapa", 2);
        this->get_parameter("towing_berapa", towing_berapa);

        this->declare_parameter("bypass_handrem_hw", false);
        this->get_parameter("bypass_handrem_hw", bypass_handrem_hw);

        this->declare_parameter("disable_brake", false);
        this->get_parameter("disable_brake", disable_brake);

        this->declare_parameter("max_change_rad_eps_tar", 0.0026);
        this->get_parameter("max_change_rad_eps_tar", max_change_rad_eps_tar);

        this->declare_parameter<std::vector<double>>("k_pid_eps_torq_vel_pos", {8.0, 0.0, 0.0, 100.0, 10.0, 0.0, 2.9, 0.0029, 73.03});
        this->get_parameter("k_pid_eps_torq_vel_pos", k_pid_eps_torq_vel_pos);

        this->declare_parameter<std::vector<double>>("k_eps_const", {1225.0, 100.0, 20.86168798, 1788.0, 70.67, 0.1, 0.1, 8.0, 1.28, 30.0, 200.0, 5600.0, 5600.0, 69.0, 7.0});
        this->get_parameter("k_eps_const", k_eps_const);

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
        // thread_routine = std::thread(std::bind(&Beckhoff::callback_routine_multi_thread, this), this);
        tim_routine = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&Beckhoff::callback_tim_routine, this));
        tim_control_brake = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Beckhoff::callback_tim_control_brake, this)); // SEMENTARA KARENA PERLU DIBENAHI SECARA MEKANIK REM NYA

        //----Publisher
        pub_sensors = this->create_publisher<std_msgs::msg::Float32MultiArray>("/beckhoff/sensors", 1);
        pub_error_code = this->create_publisher<std_msgs::msg::Int16>("/beckhoff/error_code", 1);
        pub_analog_input = this->create_publisher<std_msgs::msg::Float32MultiArray>("/beckhoff/analog_input", 1);
        pub_digital_input = this->create_publisher<std_msgs::msg::UInt8MultiArray>("/beckhoff/digital_input", 1);

        if (towing_berapa != 2)
        {
            pub_eps_encoder = this->create_publisher<std_msgs::msg::Float32>("/can/eps_encoder", 1);
            pub_fb_eps_mode = this->create_publisher<std_msgs::msg::UInt8>("/can/eps_mode", 1);
            pub_eps_err_code = this->create_publisher<std_msgs::msg::UInt32>("/can/eps_err_code", 1);
        }

        //----Subscriber
        sub_master_actuator = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/master/actuator", 1, std::bind(&Beckhoff::callback_sub_master_actuator, this, std::placeholders::_1));
        sub_master_local_fsm = this->create_subscription<std_msgs::msg::Int16>(
            "/master/local_fsm", 1, std::bind(&Beckhoff::callback_sub_master_local_fsm, this, std::placeholders::_1));
        sub_master_global_fsm = this->create_subscription<std_msgs::msg::Int16>(
            "/master/global_fsm", 1, std::bind(&Beckhoff::callback_sub_master_global_fsm, this, std::placeholders::_1));
        sub_transmission_master = this->create_subscription<std_msgs::msg::Int16>(
            "/master/transmission", 1, std::bind(&Beckhoff::callback_sub_transmission_master, this, std::placeholders::_1));
        sub_slam_status = this->create_subscription<std_msgs::msg::UInt8>(
            "/master/slam_status", 1, std::bind(&Beckhoff::callback_sub_slam_status, this, std::placeholders::_1));
        sub_master_camera_obs = this->create_subscription<std_msgs::msg::Float32>(
            "/master/camera_obs_emergency", 1, std::bind(&Beckhoff::callback_sub_master_camera_obs, this, std::placeholders::_1));
        sub_master_lidar_obs = this->create_subscription<std_msgs::msg::Float32>(
            "/master/obs_find", 1, std::bind(&Beckhoff::callback_sub_master_lidar_obs, this, std::placeholders::_1));
        sub_car_battery = this->create_subscription<std_msgs::msg::Int16>(
            "/can/battery", 1, std::bind(&Beckhoff::callback_sub_car_battery, this, std::placeholders::_1));
        sub_status_handrem = this->create_subscription<std_msgs::msg::UInt8>(
            "/can/handrem", 1, std::bind(&Beckhoff::callback_sub_status_handrem, this, std::placeholders::_1));
        sub_master_status_emergency = this->create_subscription<std_msgs::msg::Int16>(
            "/master/status_emergency", 1, std::bind(&Beckhoff::callback_sub_master_status_emergency, this, std::placeholders::_1));
    }

    float EPS_limit_current(float tar_pwm)
    {
        static uint16_t counter_current_aman = 0;
        static uint16_t counter_current_tidak_aman = 0;

        if (eps_input->i_motor > 8.0)
        {
            counter_current_aman = 0;
            counter_current_tidak_aman++;
        }

        if (eps_input->i_motor < 6.0)
        {
            counter_current_aman++;
            counter_current_tidak_aman = 0;
        }

        if (counter_current_aman > 1000)
        {
            counter_current_aman = 1000;
        }
        if (counter_current_tidak_aman > 1000)
        {
            counter_current_tidak_aman = 1000;
        }
    }

    void EPS_set_pwm(float tar_pwm)
    {
        if (tar_pwm > 0)
        {
            eps_output->tar_pwm1 = (uint16_t)tar_pwm;
            eps_output->tar_pwm2 = (uint16_t)0;
        }
        else if (tar_pwm < 0)
        {
            eps_output->tar_pwm1 = (uint16_t)0;
            eps_output->tar_pwm2 = (uint16_t)tar_pwm;
        }
        else
        {
            eps_output->tar_pwm1 = (uint16_t)0;
            eps_output->tar_pwm2 = (uint16_t)0;
        }
    }

    void EPS_pid_kecepatan(float target_kecepatan)
    {
        static rclcpp::Time last_time_kontrol_kecepatan = rclcpp::Clock(RCL_SYSTEM_TIME).now();

        static float integral = 0;

        current_time = rclcpp::Clock(RCL_SYSTEM_TIME).now();
        rclcpp::Duration dt_last_time_dipanggil = current_time - last_time_kontrol_kecepatan;
        if (dt_last_time_dipanggil.seconds() > 2)
        {
            integral = 0;
        }

        float error = target_kecepatan - eps_input->actual_vel_rad_s;
        float p = k_pid_eps_torq_vel_pos[3] * error;
        integral += k_pid_eps_torq_vel_pos[4] * error;

        if (integral > 3.14)
            integral = 3.14;
        else if (integral < -3.14)
            integral = -3.14;

        float output = p + integral;

        if (output > k_eps_const[11])
            output = k_eps_const[11];
        else if (output < -k_eps_const[11])
            output = -k_eps_const[11];

        EPS_set_pwm(output);

        last_time_kontrol_kecepatan = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    }

    void EPS_pid_posisi(float target_pos_rad)
    {
        static rclcpp::Time last_time_kontrol_posisi = rclcpp::Clock(RCL_SYSTEM_TIME).now();

        while (target_pos_rad > M_PI)
            target_pos_rad -= 2 * M_PI;
        while (target_pos_rad < -M_PI)
            target_pos_rad += 2 * M_PI;

        if (target_pos_rad > 1.00)
            target_pos_rad = 1.00;
        else if (target_pos_rad < -1.00)
            target_pos_rad = -1.00;

        static float integral = 0;

        current_time = rclcpp::Clock(RCL_SYSTEM_TIME).now();
        rclcpp::Duration dt_last_time_dipanggil = current_time - last_time_kontrol_posisi;
        if (dt_last_time_dipanggil.seconds() > 2)
        {
            integral = 0;
        }

        float error = target_pos_rad - eps_input->actual_pos_rad;
        float p = k_pid_eps_torq_vel_pos[6] * error;
        integral += k_pid_eps_torq_vel_pos[7] * error;

        if (integral > 3.14)
            integral = 3.14;
        else if (integral < -3.14)
            integral = -3.14;

        float output = p + integral;

        if (output > k_eps_const[10])
            output = k_eps_const[10];
        else if (output < -k_eps_const[10])
            output = -k_eps_const[10];

        EPS_pid_kecepatan(output);

        last_time_kontrol_posisi = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    }

    void callback_routine_multi_thread()
    {
        while (rclcpp::ok())
        {
            callback_tim_routine();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    void callback_sub_master_status_emergency(const std_msgs::msg::Int16::SharedPtr msg)
    {
        master_status_emergency = msg->data;
    }

    void callback_sub_status_handrem(const std_msgs::msg::UInt8::SharedPtr msg)
    {
        status_handrem_dibawah = msg->data;
    }

    void callback_sub_car_battery(const std_msgs::msg::Int16::SharedPtr msg)
    {
        if (msg->data > 5)
            last_time_update_can_mobil = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    }

    void callback_sub_master_lidar_obs(const std_msgs::msg::Float32::SharedPtr msg)
    {
        master_lidar_obs = msg->data;
    }

    void callback_sub_master_camera_obs(const std_msgs::msg::Float32::SharedPtr msg)
    {
        master_camera_obs = msg->data;
    }

    void callback_sub_slam_status(const std_msgs::msg::UInt8::SharedPtr msg)
    {
        master_slam_status = msg->data;
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

        // logger.info("asd: %.2f %d %d -> %d", buffer_dac_velocity, counter_zero_velocity, counter_plus_velocity, accelerator_switch);
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
        if (init_brake_jiayu_fsm() > 0)
        {
            logger.warn("Brake Jiayu error");
        }
    }

    void callback_tim_routine()
    {
        if (brake_slave_id != 255 && fsm_brake_driver == 3 && fsm_brake_calibration == 4)
        {
            int32_t output_velocity_brake_send = -output_velocity_brake;
            (void)ec_SDOread(brake_slave_id, ADDRESS_STATUS_WORD, 0x00, FALSE, &status_word_size, &status_word, EC_TIMEOUTRXM);
            (void)ec_SDOread(brake_slave_id, ADDRESS_ACTUAL_TORQUE, 0x00, FALSE, &current_torque_brake_size, &current_torque_brake, EC_TIMEOUTRXM);
            (void)ec_SDOwrite(brake_slave_id, ADDRESS_MODES_OF_OPERATION, 0x00, FALSE, sizeof(brake_mode), &brake_mode, EC_TIMEOUTRXM);
            (void)ec_SDOwrite(brake_slave_id, ADDRESS_TARGET_VELOCITY, 0x00, FALSE, sizeof(output_velocity_brake), &output_velocity_brake_send, EC_TIMEOUTRXM);
            (void)ec_SDOread(brake_slave_id, ADDRESS_CURRENT_POSITION, 0x00, FALSE, &current_brake_position_size, &current_brake_position, EC_TIMEOUTRXM);
        }
        else if (brake_slave_id != 255 && fsm_brake_driver != 3)
        {
            digital_out->data &= ~DO_LAMPU_BELAKANG_HIJAU;
            digital_out->data &= ~DO_LAMPU_BELAKANG_KUNING;
            digital_out->data |= DO_LAMPU_BELAKANG_MERAH;

            ec_send_processdata();
            int wkc = ec_receive_processdata(EC_TIMEOUTRET);
            return;
        }

        ec_send_processdata();
        int wkc = ec_receive_processdata(EC_TIMEOUTRET);

        // test_perlahan_lahan();
        // return;

        if (wkc >= expectedWKC)
        {

            digital_in1_data_kirim = digital_in1->data;

            // Tes lesting
            if ((digital_in1_data_kirim & IN_LAMPU_HAZARD) == IN_LAMPU_HAZARD)
            {
                digital_out->data |= DO_SEIN_KANAN;
                digital_out->data |= DO_SEIN_KIRI;
            }
            else
            {
                digital_out->data &= ~(DO_SEIN_KANAN | DO_SEIN_KIRI);
            }

            // BYpass
            if ((((digital_in1->data & IN_BRAKE_ACTIVE) == IN_BRAKE_ACTIVE)))
            {
                digital_out->data |= DO_LAMPU_REM_BELAKANG;
            }
            else
            {
                digital_out->data &= ~DO_LAMPU_REM_BELAKANG;
            }

            counter_beckhoff_disconnect = 0;
            current_time = rclcpp::Clock(RCL_SYSTEM_TIME).now();
            rclcpp::Duration dt_can_mobil = current_time - last_time_update_can_mobil;

            if (dt_can_mobil.seconds() < 5)
            {
                status_mobil_connected = true;
            }
            else
            {
                status_mobil_connected = false;
            }

            // ===================================================================================

            // Ketika brake siap
            if (brake_slave_id != 255 && fsm_brake_calibration == 4)
            {
                if (master_target_volt_hat < -1.0 - __FLT_EPSILON__ /*&& fabsf(buffer_dac_velocity - dac_velocity_minimum) < 0.1*/)
                {
                    (void)brake_control_position(brake_maximum_position, 10000);
                }
                else
                {
                    (void)brake_control_position(brake_idle_position, 10000);
                }
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

            // Kontrol Lampu belakang
            static uint16_t counter_kedip_lampu_kuning = 0;
            static uint16_t counter_kedip_lampu_hijau = 0;
            // Setir bermasalah atau EPS fault
            if (!status_mobil_connected)
            {
                digital_out->data &= ~DO_LAMPU_BELAKANG_HIJAU;
                digital_out->data &= ~DO_LAMPU_BELAKANG_KUNING;
                digital_out->data &= ~DO_LAMPU_BELAKANG_MERAH;
            }
            else if (master_global_fsm == 0 || master_global_fsm == 1 || (towing_berapa == 2 && (digital_in1->data & IN_EPS_nFAULT) == 0))
            {
                digital_out->data &= ~DO_LAMPU_BELAKANG_HIJAU;
                digital_out->data &= ~DO_LAMPU_BELAKANG_KUNING;
                digital_out->data |= DO_LAMPU_BELAKANG_MERAH;
                counter_kedip_lampu_kuning = 0;
                counter_kedip_lampu_hijau = 0;
            }
            else if (master_global_fsm == 7 || master_global_fsm == 8)
            {
                digital_out->data &= ~DO_LAMPU_BELAKANG_HIJAU;
                digital_out->data &= ~DO_LAMPU_BELAKANG_MERAH;
                digital_out->data |= DO_LAMPU_BELAKANG_KUNING;
            }
            else if (master_global_fsm == 3)
            {
                if (counter_kedip_lampu_kuning > 40)
                {
                    if (master_local_fsm == 1)
                    {
                        digital_out->data &= ~DO_LAMPU_BELAKANG_HIJAU;
                        digital_out->data &= ~DO_LAMPU_BELAKANG_MERAH;
                        digital_out->data |= DO_LAMPU_BELAKANG_KUNING;
                    }
                    else
                    {
                        digital_out->data &= ~DO_LAMPU_BELAKANG_HIJAU;
                        digital_out->data |= DO_LAMPU_BELAKANG_MERAH;
                        digital_out->data &= ~DO_LAMPU_BELAKANG_KUNING;
                    }
                    counter_kedip_lampu_kuning = 0;
                }
                else if (counter_kedip_lampu_kuning > 20)
                {
                    digital_out->data &= ~DO_LAMPU_BELAKANG_HIJAU;
                    digital_out->data &= ~DO_LAMPU_BELAKANG_MERAH;
                    digital_out->data &= ~DO_LAMPU_BELAKANG_KUNING;
                    digital_out->data &= ~DO_LAMPU_BELAKANG_HIJAU;
                    digital_out->data &= ~DO_LAMPU_BELAKANG_MERAH;
                    digital_out->data &= ~DO_LAMPU_BELAKANG_KUNING;
                }
                counter_kedip_lampu_kuning++;
                counter_kedip_lampu_hijau = 0;
            }
            else
            {
                // Jika posisi sudah benar
                if ((master_slam_status & 0b1001) == 0b1001)
                {
                    digital_out->data &= ~DO_LAMPU_BELAKANG_KUNING;
                    digital_out->data &= ~DO_LAMPU_BELAKANG_MERAH;
                    digital_out->data |= DO_LAMPU_BELAKANG_HIJAU;
                    counter_kedip_lampu_kuning = 0;
                    counter_kedip_lampu_hijau = 0;
                }
                // Jika map sudah ada tapi posisi belum benar
                else if ((master_slam_status & 0b01) == 0b01)
                {
                    if (counter_kedip_lampu_hijau > 40)
                    {
                        digital_out->data &= ~DO_LAMPU_BELAKANG_KUNING;
                        digital_out->data &= ~DO_LAMPU_BELAKANG_MERAH;
                        digital_out->data |= DO_LAMPU_BELAKANG_HIJAU;
                        counter_kedip_lampu_hijau = 0;
                    }
                    else if (counter_kedip_lampu_hijau > 20)
                    {
                        digital_out->data &= ~DO_LAMPU_BELAKANG_KUNING;
                        digital_out->data &= ~DO_LAMPU_BELAKANG_MERAH;
                        digital_out->data &= ~DO_LAMPU_BELAKANG_HIJAU;
                    }
                    counter_kedip_lampu_hijau++;
                    counter_kedip_lampu_kuning = 0;
                }
                else
                {
                    digital_out->data &= ~DO_LAMPU_BELAKANG_HIJAU;
                    digital_out->data &= ~DO_LAMPU_BELAKANG_MERAH;
                    digital_out->data |= DO_LAMPU_BELAKANG_KUNING;
                    counter_kedip_lampu_hijau = 0;
                }
            }

            if (master_global_fsm == 3 && master_local_fsm == 1)
            {
                if (towing_berapa == 2)
                {
                    if ((digital_in1->data & IN_SELECTOR_DISABLE_SIRINE) == IN_SELECTOR_DISABLE_SIRINE)
                        digital_out->data |= DO_SIRINE;
                    else
                        digital_out->data &= ~DO_SIRINE;
                }
                else
                {
                    if ((digital_in1->data & IN_SELECTOR_DISABLE_SIRINE) == 0)
                        digital_out->data |= DO_SIRINE;
                    else
                        digital_out->data &= ~DO_SIRINE;
                }
            }
            else
            {
                digital_out->data &= ~DO_SIRINE;
            }

            if ((digital_in1->data & IN_SELECTOR_SIRINE) == IN_SELECTOR_SIRINE)
            {
                digital_out->data |= DO_SIRINE;
            }

            static uint16_t counter = 0;
            static uint16_t target_delay = 20;
            uint8_t obs_all_aktif = ((master_status_emergency & EMERGENCY_ALL_LIDAR_DETECTED) == EMERGENCY_ALL_LIDAR_DETECTED);
            uint8_t obs_depan_aktif = ((master_status_emergency & EMERGENCY_LIDAR_DEPAN_DETECTED) == EMERGENCY_LIDAR_DEPAN_DETECTED);
            uint8_t icp_terlalu_besar = ((master_status_emergency & EMERGENCY_ICP_SCORE_TERLALU_BESAR) == EMERGENCY_ICP_SCORE_TERLALU_BESAR);
            uint8_t gandengan_lepas = ((master_status_emergency & EMERGENCY_GANDENGAN_LEPAS) == EMERGENCY_GANDENGAN_LEPAS);
            // logger.info("EMERGENCY STATUS: %d %d %d %d", obs_all_aktif, obs_depan_aktif, icp_terlalu_besar, gandengan_lepas);
            if ((obs_all_aktif + obs_depan_aktif + icp_terlalu_besar + gandengan_lepas) > 0 && status_mobil_connected)
            {
                if (obs_all_aktif && obs_depan_aktif && icp_terlalu_besar)
                {
                    target_delay = 10;
                }
                else if (obs_all_aktif && icp_terlalu_besar)
                {
                    target_delay = 20;
                }
                else if (obs_all_aktif && obs_depan_aktif)
                {
                    target_delay = 20;
                }
                else if (icp_terlalu_besar && obs_depan_aktif)
                {
                    target_delay = 20;
                }
                else if (obs_all_aktif)
                {
                    target_delay = 40;
                }
                else if (obs_depan_aktif)
                {
                    target_delay = 60;
                }
                else if (icp_terlalu_besar)
                {
                    target_delay = 80;
                }
                else if (gandengan_lepas)
                {
                    target_delay = 100;
                }

                if (counter > target_delay)
                {
                    digital_out->data |= DO_BUZZER_BELAKANG;
                    counter = 0;
                }
                else if (counter > (target_delay >> 0x01))
                {
                    digital_out->data &= ~DO_BUZZER_BELAKANG;
                }
                counter++;
            }
            else
            {
                digital_out->data &= ~DO_BUZZER_BELAKANG;
            }

            if (status_mobil_connected)
                digital_out->data |= DO_LAMPU_BAWAAN;
            else
                digital_out->data &= ~DO_LAMPU_BAWAAN;

            // ===================================================================================
            if (towing_berapa == 2)
            {
                fb_throttle_velocity_volt = (float)analog_input->data_2 * ANALOG_INPUT_SCALER;
            }
            else
            {
                fb_throttle_velocity_volt = (float)analog_input->data_1 * ANALOG_INPUT_SCALER;
            }

            // Ketika handrem dibawah
            if (status_handrem_dibawah == 1 || bypass_handrem_hw)
            {
                // Ketika throttle ditekan, maka ikut throttle
                if (fb_throttle_velocity_volt > 0.415)
                {
                    buffer_dac_velocity = fb_throttle_velocity_volt;
                }
                else
                {
                    buffer_dac_velocity += master_target_volt_hat;
                }
            }
            else if (status_handrem_dibawah == 0)
            {
                buffer_dac_velocity = 0.0;
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

            if (brake_slave_id != 255 && fsm_brake_calibration == 4 && abs(brake_idle_position - local_brake_position) < 50)
                analog_output->data_1 = (int16_t)(dac_velocity_send * ANALOG_OUT_SCALER);
            else if (brake_slave_id == 255)
                analog_output->data_1 = (int16_t)(dac_velocity_send * ANALOG_OUT_SCALER);

            // JIKA koENek EPS
            if (eps_slave_id != 255 && towing_berapa != 2)
            {
                // NOrmal
                // eps_output->tar_pos_rad = master_target_steering;
                // if (master_global_fsm == 3 || master_global_fsm == 5 || master_global_fsm == 6)
                // {
                //     eps_output->cmd_eps_mode = 2;
                // }
                // else
                // {
                //     eps_output->cmd_eps_mode = 1;
                // }

                eps_output->tar_pos_rad = master_target_steering;
                if (master_global_fsm == 3 || master_global_fsm == 5 || master_global_fsm == 6)
                {
                    static float buffer_eps_tar_pos_rad = eps_output->tar_pos_rad;

                    float d_tar_pos_rad = eps_output->tar_pos_rad - buffer_eps_tar_pos_rad;
                    if (d_tar_pos_rad > max_change_rad_eps_tar)
                    {
                        d_tar_pos_rad = max_change_rad_eps_tar;
                    }
                    if (d_tar_pos_rad < -max_change_rad_eps_tar)
                    {
                        d_tar_pos_rad = -max_change_rad_eps_tar;
                    }

                    buffer_eps_tar_pos_rad += d_tar_pos_rad;

                    eps_output->cmd_eps_mode = 3;
                    EPS_pid_posisi(buffer_eps_tar_pos_rad);
                }
                else
                {
                    eps_output->cmd_eps_mode = 1;
                    EPS_pid_posisi(eps_input->actual_pos_rad);
                }

                // logger.info("EPS Target: %.2f %d", eps_output->tar_pos_rad, eps_output->cmd_eps_mode);
            }

            static uint16_t pembagi_cntr_log_beckhoff = 0;
            if (pembagi_cntr_log_beckhoff++ >= 10)
            {
                pembagi_cntr_log_beckhoff = 0;
                logger.info("HW: %d %d || %.2f %.2f || %d ||  %d %d || %.2f %.2f || %d %.2f || %d",
                            digital_in1->data,
                            digital_in2->data,
                            (float)analog_input->data_1 * ANALOG_INPUT_SCALER,
                            (float)analog_input->data_2 * ANALOG_INPUT_SCALER,
                            digital_out->data,
                            local_brake_position,
                            status_handrem_dibawah,
                            master_target_volt_hat,
                            master_target_steering,
                            accelerator_switch,
                            buffer_dac_velocity,
                            status_mobil_connected);
            }

            if (eps_slave_id != 255)
            {
                digital_in1_data_kirim &= ~IN_EPS_nFAULT; // Reset
                if (eps_input->eps_err_code == 0)
                {
                    digital_in1_data_kirim |= IN_EPS_nFAULT;
                }
                else
                {
                    digital_in1_data_kirim &= ~IN_EPS_nFAULT;
                }

                if (eps_input->actual_pos_rad >= 0.1)
                {
                    digital_out->data |= DO_SEIN_KIRI;
                }
                else if (eps_input->actual_pos_rad <= -0.1)
                {
                    digital_out->data |= DO_SEIN_KANAN;
                }

                static uint16_t pembagi_cntr_log_eps = 0;
                if (pembagi_cntr_log_eps++ >= 10)
                {
                    pembagi_cntr_log_eps = 0;
                    logger.info("EPS: %.2f %.2f || %.2f %.2f %.2f || %.2f %.2f %.2f || %d %d || %d %d || %.4f %d",
                                eps_input->vsupply,
                                eps_input->i_motor,
                                eps_input->actual_torque_mv,
                                eps_input->actual_pos_rad,
                                eps_input->actual_vel_rad_s,
                                eps_input->internal_temp,
                                eps_input->ntc_temp,
                                eps_input->ntc_volt,
                                eps_input->eps_mode,
                                eps_input->eps_err_code,
                                eps_input->pwm1,
                                eps_input->pwm2,
                                eps_output->tar_pos_rad,
                                eps_output->cmd_eps_mode);
                }
            }

            // logger.info("INFO: %d %.2f || %.2f %.2f %d || %d", accelerator_switch, fb_throttle_velocity_volt, buffer_dac_velocity, dac_velocity_send, (digital_out->data & DO_SWITCH_THROTTLE), status_handrem_dibawah);

            // ====================================================================================

            error_code = 0;

            transmit_all();
        }
        else
        {
            logger.info("WKC error %d || %d", wkc, expectedWKC);
            counter_beckhoff_disconnect++;
            error_code = 1;
        }

        if (counter_beckhoff_disconnect > 100)
        {
            error_code = 5;
            logger.error("Computer LAN Wiring error");
            rclcpp::shutdown();
        }

        int32_t diff_brake_position = current_brake_position - prev_brake_position;
        if (diff_brake_position > 1000000000 || diff_brake_position < -1000000000)
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

    void test_perlahan_lahan()
    {

        float adc_1 = (float)analog_input->data_1 * ANALOG_INPUT_SCALER;

        logger.info("adc %.2f %d %d %d %d %d %d %d %d %d || %d %d %d %d %d %d || %d %d || %d",
                    adc_1,
                    analog_input->data_1,
                    (digital_in1->data & IN_START_OP3_HANDLER),
                    (digital_in1->data & IN_STOP_OP3_HANDLER),
                    (digital_in1->data & IN_LS_BRAKE),
                    (digital_in1->data & IN_BRAKE_ACTIVE),
                    (digital_in1->data & IN_SELECTOR_SIRINE),
                    (digital_in1->data & IN_SELECTOR_DISABLE_SIRINE),
                    (digital_in1->data & IN_TR_FORWARD),
                    (digital_in1->data & IN_TR_REVERSE),
                    (digital_in2->data & (IN_SYSTEM_FULL_ENABLE >> 16)),
                    (digital_in2->data & (IN_START_OP3 >> 16)),
                    (digital_in2->data & (IN_STOP_OP3 >> 16)),
                    (digital_in2->data & (IN_MANUAL_MAJU >> 16)),
                    (digital_in2->data & (IN_MANUAL_MUNDUR >> 16)),
                    (digital_in2->data & (IN_NEXT_TERMINAL >> 16)),
                    digital_in1->data, digital_in2->data,
                    status_handrem_dibawah);

        // JIKA koENek EPS
        if (eps_slave_id != 255 && towing_berapa != 2)
        {
            eps_output->tar_pos_rad = master_target_steering;
            if (master_global_fsm == 3 || master_global_fsm == 5 || master_global_fsm == 6)
            {
                eps_output->cmd_eps_mode = 2;
            }
            else
            {
                eps_output->cmd_eps_mode = 1;
            }
            logger.info("EPS Target: %.2f %d", eps_output->tar_pos_rad, eps_output->cmd_eps_mode);
        }

        static uint16_t counter_blink = 0;

        if (counter_blink > 700)
        {
            counter_blink = 0;
            digital_out->data |= DO_TRANSMISSION_NEUTRAL;
            // digital_out->data &= ~DO_TRANSMISSION_FORWARD;
            // digital_out->data &= ~DO_BUZZER_TOWING;
            digital_out->data &= ~DO_BUZZER_BELAKANG;

            digital_out->data &= ~DO_LAMPU_BELAKANG_HIJAU;
            digital_out->data |= DO_LAMPU_BELAKANG_KUNING;
            digital_out->data &= ~DO_LAMPU_BELAKANG_MERAH;

            digital_out->data &= ~DO_SWITCH_THROTTLE;
            digital_out->data &= ~DO_SIRINE;
            digital_out->data &= ~DO_LAMPU_BAWAAN;

            // analog_output_4104->data_1 = (int16_t)(0.01 * ANALOG_OUT_SCALER);
        }
        else if (counter_blink > 350)
        {
            // digital_out->data &= ~DO_TRANSMISSION_NEUTRAL;
            // digital_out->data |= DO_TRANSMISSION_FORWARD;
            // digital_out->data |= DO_BUZZER_TOWING;
            digital_out->data |= DO_BUZZER_BELAKANG;

            digital_out->data |= DO_LAMPU_BELAKANG_HIJAU;
            digital_out->data &= ~DO_LAMPU_BELAKANG_KUNING;
            digital_out->data |= DO_LAMPU_BELAKANG_MERAH;

            digital_out->data |= DO_SWITCH_THROTTLE;
            digital_out->data |= DO_SIRINE;
            digital_out->data |= DO_LAMPU_BAWAAN;
        }
        analog_output->data_1 = (int16_t)(2.0 * ANALOG_OUT_SCALER);
        analog_output->data_3 = (int16_t)(0b11);
        counter_blink++;
        logger.info("cntr %d %d %d", counter_blink, (digital_out->data & DO_TRANSMISSION_NEUTRAL), analog_output->data_1);

        return;
    }

    int8_t brake_control_position(float target, float max_velocities)
    {
        static float kp = 0.3;
        static const float toleransi_error = 100;

        float error = local_brake_position - target;

        float proportional = error * kp;

        float output = proportional;

        if (output > max_velocities)
            output = max_velocities;
        else if (output < -max_velocities)
            output = -max_velocities;

        output_velocity_brake = (int)output;

        // logger.info("Brake: %d %d %d %d %d", local_brake_position, target, output_velocity_brake, error, output);

        if (fabs(error) < toleransi_error)
            return 1;
        else
            return 0;
    }

    int8_t init_brake_jiayu_fsm()
    {
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
            static uint8_t counter_ls_brake_active = 0;
            static uint8_t counter_brake_active = 0;
            static bool is_idle_calibration = false;
            static bool is_maximum_calibration = false;

            // logger.info("fsm_brake_calibration: (%d) (%d %d) (%d %d) %d %d %d %d %d", sword, is_idle_calibration, is_maximum_calibration, (digital_in1->data & IN_LS_BRAKE), (digital_in1->data & IN_BRAKE_ACTIVE), fsm_brake_calibration, brake_idle_position, brake_maximum_position, local_brake_position, current_brake_position);

            switch (fsm_brake_calibration)
            {
            case 0:
                counter_ls_brake_active = 0;
                counter_brake_active = 0;
                fsm_brake_calibration = 1;
                break;

            case 1:
                output_velocity_brake = -10000;

                if ((digital_in1->data & IN_LS_BRAKE) == IN_LS_BRAKE)
                {
                    local_brake_position = 0;
                    is_idle_calibration = true;
                    logger.warn("Brake idle calibration");
                }

                if ((digital_in1->data & IN_BRAKE_ACTIVE) == IN_BRAKE_ACTIVE)
                {
                    if (is_idle_calibration)
                    {
                        brake_maximum_position = local_brake_position;
                        is_maximum_calibration = true;
                        logger.warn("Brake maximum calibration on %d", local_brake_position);
                    }
                }

                if (is_idle_calibration && is_maximum_calibration)
                {
                    fsm_brake_calibration = 2;
                }

                break;

            case 2:
                output_velocity_brake = 10000;

                if ((digital_in1->data & IN_LS_BRAKE) == IN_LS_BRAKE)
                {
                    logger.warn("Brake idle sens again");
                    fsm_brake_calibration = 3;
                }

                break;

            case 3:
                if (brake_control_position(brake_idle_position, 1500) == 1)
                {
                    fsm_brake_calibration = 4;
                }
                break;
            }

            if (fsm_brake_calibration != 4)
            {
                int32_t output_velocity_brake_send = -output_velocity_brake;
                (void)ec_SDOwrite(brake_slave_id, ADDRESS_CONTROL_WORD, 0x00, FALSE, sizeof(control_word), &control_word, EC_TIMEOUTRXM);
                (void)ec_SDOread(brake_slave_id, ADDRESS_ACTUAL_TORQUE, 0x00, FALSE, &current_torque_brake_size, &current_torque_brake, EC_TIMEOUTRXM);
                (void)ec_SDOwrite(brake_slave_id, ADDRESS_MODES_OF_OPERATION, 0x00, FALSE, sizeof(brake_mode), &brake_mode, EC_TIMEOUTRXM);
                (void)ec_SDOwrite(brake_slave_id, ADDRESS_TARGET_VELOCITY, 0x00, FALSE, sizeof(output_velocity_brake), &output_velocity_brake_send, EC_TIMEOUTRXM);
                (void)ec_SDOread(brake_slave_id, ADDRESS_CURRENT_POSITION, 0x00, FALSE, &current_brake_position_size, &current_brake_position, EC_TIMEOUTRXM);

                int32_t diff_brake_position = current_brake_position - prev_brake_position;
                if (diff_brake_position > 1000000 || diff_brake_position < -1000000)
                {
                    logger.error("Brake position error");
                    diff_brake_position = 0;
                }

                local_brake_position += diff_brake_position;

                prev_brake_position = current_brake_position;
            }

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
        else if (5155 == sword || 1059 == sword || 4131 == sword || 1079 == sword)
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

        std_msgs::msg::UInt8MultiArray msg_digital_input;
        msg_digital_input.data.push_back(digital_in1_data_kirim & 0xFF);
        msg_digital_input.data.push_back((digital_in1_data_kirim >> 8) & 0xFF);
        msg_digital_input.data.push_back((digital_in2->data >> 0) & 0xFF);
        msg_digital_input.data.push_back((digital_in2->data >> 8) & 0xFF);
        pub_digital_input->publish(msg_digital_input);

        if (towing_berapa != 2 && eps_slave_id != 255)
        {
            std_msgs::msg::Float32 msg_eps_encoder;
            fb_steering_angle = (float)eps_input->actual_pos_rad;
            msg_eps_encoder.data = fb_steering_angle;
            pub_eps_encoder->publish(msg_eps_encoder);

            std_msgs::msg::UInt8 msg_fb_eps_mode;
            msg_fb_eps_mode.data = (uint8_t)eps_input->eps_mode;
            pub_fb_eps_mode->publish(msg_fb_eps_mode);

            std_msgs::msg::UInt32 msg_fb_eps_err_code;
            msg_fb_eps_err_code.data = (uint32_t)eps_input->eps_err_code;
            pub_eps_err_code->publish(msg_fb_eps_err_code);
        }
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
                    set_watchdog(slave, 700);
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

                    static uint8_t counter_slave_1889_ditemukan = 0;

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
                        case EL4104_ID:
                            analog_output = (analog_output_t *)ec_slave[slave].outputs;
                            logger.info("EL4004 Configured");
                            break;

                        case JIAYU_ID:
                            if (disable_brake == false)
                            {
                                brake_slave_id = slave; // asdasdasd
                            }
                            else
                            {
                                brake_slave_id = 255; // asdasdasd
                            }
                            logger.info("Brake Driver ID: %d", brake_slave_id);
                            break;

                        case EL1889_ID:
                            if (counter_slave_1889_ditemukan == 0)
                            {
                                digital_in1 = (digital_in_t *)ec_slave[slave].inputs;
                                logger.info("EL1889(1) Configured on slave %d", slave);
                                counter_slave_1889_ditemukan++;
                            }
                            else if (counter_slave_1889_ditemukan == 1)
                            {
                                digital_in2 = (digital_in_t *)ec_slave[slave].inputs;
                                logger.info("EL1889(2) Configured on slave %d", slave);
                                counter_slave_1889_ditemukan++;
                            }

                            break;

                        case EPS_MRI_IST_ID:
                            eps_slave_id = slave;
                            eps_output = (eps_output_t *)ec_slave[slave].outputs;
                            eps_input = (eps_input_t *)ec_slave[slave].inputs;
                            logger.info("EPS MRI IST Configured on slave %d", slave);

                            {
                                float float_k_pid_eps_torq_vel_pos[9] = {
                                    (float)k_pid_eps_torq_vel_pos[0], (float)k_pid_eps_torq_vel_pos[1], (float)k_pid_eps_torq_vel_pos[2],
                                    (float)k_pid_eps_torq_vel_pos[3], (float)k_pid_eps_torq_vel_pos[4], (float)k_pid_eps_torq_vel_pos[5],
                                    (float)k_pid_eps_torq_vel_pos[6], (float)k_pid_eps_torq_vel_pos[7], (float)k_pid_eps_torq_vel_pos[8]};
                                (void)ec_SDOwrite(eps_slave_id, EPS_ADDRES_KP_TORQUE, 0x00, FALSE, sizeof(float_k_pid_eps_torq_vel_pos[0]), &float_k_pid_eps_torq_vel_pos[0], EC_TIMEOUTRXM);
                                (void)ec_SDOwrite(eps_slave_id, EPS_ADDRES_KI_TORQUE, 0x00, FALSE, sizeof(float_k_pid_eps_torq_vel_pos[1]), &float_k_pid_eps_torq_vel_pos[1], EC_TIMEOUTRXM);
                                (void)ec_SDOwrite(eps_slave_id, EPS_ADDRES_KD_TORQUE, 0x00, FALSE, sizeof(float_k_pid_eps_torq_vel_pos[2]), &float_k_pid_eps_torq_vel_pos[2], EC_TIMEOUTRXM);

                                (void)ec_SDOwrite(eps_slave_id, EPS_ADDRES_KP_POS, 0x00, FALSE, sizeof(float_k_pid_eps_torq_vel_pos[6]), &float_k_pid_eps_torq_vel_pos[6], EC_TIMEOUTRXM);
                                (void)ec_SDOwrite(eps_slave_id, EPS_ADDRES_KI_POS, 0x00, FALSE, sizeof(float_k_pid_eps_torq_vel_pos[7]), &float_k_pid_eps_torq_vel_pos[7], EC_TIMEOUTRXM);
                                (void)ec_SDOwrite(eps_slave_id, EPS_ADDRES_KD_POS, 0x00, FALSE, sizeof(float_k_pid_eps_torq_vel_pos[8]), &float_k_pid_eps_torq_vel_pos[8], EC_TIMEOUTRXM);

                                (void)ec_SDOwrite(eps_slave_id, EPS_ADDRES_KP_VEL, 0x00, FALSE, sizeof(float_k_pid_eps_torq_vel_pos[3]), &float_k_pid_eps_torq_vel_pos[3], EC_TIMEOUTRXM);
                                (void)ec_SDOwrite(eps_slave_id, EPS_ADDRES_KI_VEL, 0x00, FALSE, sizeof(float_k_pid_eps_torq_vel_pos[4]), &float_k_pid_eps_torq_vel_pos[4], EC_TIMEOUTRXM);
                                (void)ec_SDOwrite(eps_slave_id, EPS_ADDRES_KD_VEL, 0x00, FALSE, sizeof(float_k_pid_eps_torq_vel_pos[5]), &float_k_pid_eps_torq_vel_pos[5], EC_TIMEOUTRXM);

                                logger.info("EPS PID parameters set %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f",
                                            float_k_pid_eps_torq_vel_pos[0], float_k_pid_eps_torq_vel_pos[1], float_k_pid_eps_torq_vel_pos[2],
                                            float_k_pid_eps_torq_vel_pos[3], float_k_pid_eps_torq_vel_pos[4], float_k_pid_eps_torq_vel_pos[5],
                                            float_k_pid_eps_torq_vel_pos[6], float_k_pid_eps_torq_vel_pos[7], float_k_pid_eps_torq_vel_pos[8]);
                            }

                            {
                                float float_k_eps_const[15] = {
                                    (float)k_eps_const[0], (float)k_eps_const[1], (float)k_eps_const[2], (float)k_eps_const[3],
                                    (float)k_eps_const[4], (float)k_eps_const[5], (float)k_eps_const[6], (float)k_eps_const[7],
                                    (float)k_eps_const[8], (float)k_eps_const[9], (float)k_eps_const[10], (float)k_eps_const[11],
                                    (float)k_eps_const[12], (float)k_eps_const[13], (float)k_eps_const[14]};

                                (void)ec_SDOwrite(eps_slave_id, EPS_ADDRESS_TOR_OFFSET, 0x00, FALSE, sizeof(float_k_eps_const[0]), &float_k_eps_const[0], EC_TIMEOUTRXM);
                                (void)ec_SDOwrite(eps_slave_id, EPS_ADDRESS_MIN_TOR_VAL, 0x00, FALSE, sizeof(float_k_eps_const[1]), &float_k_eps_const[1], EC_TIMEOUTRXM);
                                (void)ec_SDOwrite(eps_slave_id, EPS_ADDRESS_VOLTAGE_SCALE, 0x00, FALSE, sizeof(float_k_eps_const[2]), &float_k_eps_const[2], EC_TIMEOUTRXM);
                                (void)ec_SDOwrite(eps_slave_id, EPS_ADDRESS_CURRENT_OFFSET, 0x00, FALSE, sizeof(float_k_eps_const[3]), &float_k_eps_const[3], EC_TIMEOUTRXM);
                                (void)ec_SDOwrite(eps_slave_id, EPS_ADDRESS_CURRENT_SCALE, 0x00, FALSE, sizeof(float_k_eps_const[4]), &float_k_eps_const[4], EC_TIMEOUTRXM);
                                (void)ec_SDOwrite(eps_slave_id, EPS_ADDRESS_TOR_LPF_CUTOFF_HZ, 0x00, FALSE, sizeof(float_k_eps_const[5]), &float_k_eps_const[5], EC_TIMEOUTRXM);
                                (void)ec_SDOwrite(eps_slave_id, EPS_ADDRESS_CUR_LPF_CUTOFF_HZ, 0x00, FALSE, sizeof(float_k_eps_const[6]), &float_k_eps_const[6], EC_TIMEOUTRXM);
                                (void)ec_SDOwrite(eps_slave_id, EPS_ADDRESS_CURRENT_LIMIT, 0x00, FALSE, sizeof(float_k_eps_const[7]), &float_k_eps_const[7], EC_TIMEOUTRXM);
                                (void)ec_SDOwrite(eps_slave_id, EPS_ADDRESS_POS_LIMIT, 0x00, FALSE, sizeof(float_k_eps_const[8]), &float_k_eps_const[8], EC_TIMEOUTRXM);
                                (void)ec_SDOwrite(eps_slave_id, EPS_ADDRESS_VEL_ACCEL, 0x00, FALSE, sizeof(float_k_eps_const[9]), &float_k_eps_const[9], EC_TIMEOUTRXM);
                                (void)ec_SDOwrite(eps_slave_id, EPS_ADDRESS_PID_POS_OUT_LIM, 0x00, FALSE, sizeof(float_k_eps_const[10]), &float_k_eps_const[10], EC_TIMEOUTRXM);
                                (void)ec_SDOwrite(eps_slave_id, EPS_ADDRESS_PID_VEL_OUT_LIM, 0x00, FALSE, sizeof(float_k_eps_const[11]), &float_k_eps_const[11], EC_TIMEOUTRXM);
                                (void)ec_SDOwrite(eps_slave_id, EPS_ADDRESS_PID_TOR_OUT_LIM, 0x00, FALSE, sizeof(float_k_eps_const[12]), &float_k_eps_const[12], EC_TIMEOUTRXM);
                                (void)ec_SDOwrite(eps_slave_id, EPS_ADDRESS_DEADBAND_PWM, 0x00, FALSE, sizeof(float_k_eps_const[13]), &float_k_eps_const[13], EC_TIMEOUTRXM);
                                (void)ec_SDOwrite(eps_slave_id, EPS_ADDRESS_LIMIT_PERUBAHAN_PWM, 0x00, FALSE, sizeof(float_k_eps_const[14]), &float_k_eps_const[14], EC_TIMEOUTRXM);

                                logger.info("EPS Constant parameters set %.2f %.2f %.2f  %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
                                            float_k_eps_const[0],
                                            float_k_eps_const[1], float_k_eps_const[2], float_k_eps_const[3],
                                            float_k_eps_const[4], float_k_eps_const[5], float_k_eps_const[6], float_k_eps_const[7],
                                            float_k_eps_const[8], float_k_eps_const[9], float_k_eps_const[10], float_k_eps_const[11],
                                            float_k_eps_const[12], float_k_eps_const[13], float_k_eps_const[14]);
                            }

                            break;
                        }
                    }
                }

                if (brake_slave_id == 255)
                {
                    logger.warn("No slave found with brake driver ID");
                    // return 4;
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
