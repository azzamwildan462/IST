#include "wr_ls_udp2/wr_ls1207de_parser.h"
#include "wr_ls_udp2/wr_ls_sensor_frame.h"
#include <rclcpp/rclcpp.hpp>
#include <stdio.h>

namespace wr_ls_udp
{
CWrLs1207DEParser::CWrLs1207DEParser(rclcpp::Node::SharedPtr node) : mNode(node),
    CParserBase(),
    fRangeMin(0.01),
    fRangeMax(20.0),
    fTimeIncrement(-1.0),
    fFrame_id("laser_link")
{
    // Do Nothing...
}

CWrLs1207DEParser::~CWrLs1207DEParser()
{
    // Do Nothing...
}

int CWrLs1207DEParser::Parse(char *data, size_t data_length, WrLsConfig &config, sensor_msgs::msg::LaserScan &msg)
{
    CWrLsSensFrame *pSensFrame = new CWrLsSensFrame();
    if(!pSensFrame->InitFromSensBuff(data, data_length))
    {
        RCLCPP_INFO(mNode->get_logger(),"Invalid frame data!");
        //printf("Invalid frame data!\r\n");
        return ExitSuccess;
    }

    int dataCount = pSensFrame->GetSensDataCount();

    msg.header.frame_id = fFrame_id;
    rclcpp::Time start_time = mNode->now();
    unsigned short scanning_freq = 1000 / 36 * 100; /*For dev borad, the device will send data every 36ms*/
    msg.scan_time = 1.0 / (scanning_freq / 100.0);
    RCLCPP_DEBUG(mNode->get_logger(),"scanning freq: %d, scan_time: %f", scanning_freq, msg.scan_time);


    fTimeIncrement = 0.000033;
    msg.time_increment = fTimeIncrement;
    RCLCPP_DEBUG(mNode->get_logger(),"time_increment: %f", msg.time_increment);

    int starting_angle = 0xFFF92230; /*This value is from Sick Tim*/
    msg.angle_min = (starting_angle / 10000.0) / 180.0 * M_PI - M_PI / 2;
    RCLCPP_DEBUG(mNode->get_logger(),"starting_angle: %d, angle_min: %f", starting_angle, msg.angle_min);

    unsigned short angular_step_width = 0xD05; /*3333: This value is from Sick Tim*/
    msg.angle_increment = (angular_step_width / 10000.0) / 180.0 * M_PI;

    msg.angle_max = msg.angle_min + (dataCount - 1) * msg.angle_increment;
    //RCLCPP_WARN(mNode->get_logger(),"\033[32m AngleMax: %.3f \033[0m",msg.angle_max);


    int index_min = 0;
    while (msg.angle_min + msg.angle_increment < config.min_ang)
    {
        msg.angle_min += msg.angle_increment;
        index_min++;
    }
    RCLCPP_DEBUG(mNode->get_logger(),"index_min: %d, angle_min: %f", index_min, msg.angle_min);

    int index_max = dataCount - 1;
    while (msg.angle_max - msg.angle_increment > config.max_ang)
    {
        msg.angle_max -= msg.angle_increment;
        index_max--;
    }
    RCLCPP_DEBUG(mNode->get_logger(),"index_max: %i, angle_max: %f", index_max, msg.angle_max);

    msg.ranges.resize(index_max - index_min + 1);
    msg.ranges.assign(index_max - index_min + 1, std::numeric_limits<double>::infinity());
    RCLCPP_DEBUG(mNode->get_logger(),"Fill sensor data. index_min = %d, index_max = %d.", index_min, index_max);

    int check_all_num = 0;
    int check_fault_cnt = 0;
    int check_fault_cnt2 = 0;	
    for (int j = index_min; j <= index_max; ++j)
    {
        check_all_num++;

        if(config.debug_mode)
        {
            if((j - index_min + 1) % 48 == 0)
            {
                printf("\n");
            }
        }

        unsigned short range = pSensFrame->GetSensDataOfIndex(j);
        if(0x01 == range)
        {
            check_fault_cnt++;
        }

        float meter_value = range / 1000.0;
        if(meter_value > fRangeMin && meter_value < fRangeMax)
        {
            if(std::abs(std::abs((j - index_min) * msg.angle_increment / M_PI * 180 + msg.angle_min / M_PI * 180) - 90) < 5 ||
               std::abs((j - index_min) * msg.angle_increment / M_PI * 180 + msg.angle_min / M_PI * 180) - 105 > 0 ||
               std::abs(std::abs((j - index_min) * msg.angle_increment / M_PI * 180 + msg.angle_min / M_PI * 180) - 120) < 10 && (meter_value < 1.2) )
            {
                msg.ranges[j - index_min] = meter_value / 100;
            }
            else 
            {
                msg.ranges[j - index_min] = meter_value;
            }
            
        }
		else
		{
			check_fault_cnt2++;
		}

        if(config.debug_mode)
        {
            printf("%.2f ", msg.ranges[j - index_min]);
        }
    }
    if(config.debug_mode)
    {
        printf("\n");
    }

    if((check_all_num == check_fault_cnt) || (check_all_num == check_fault_cnt2))
    {
        return ExitError;
    }

    if(config.intensity && data_length == dataCount * 4)
    {
        msg.intensities.resize(index_max - index_min + 1);
        for (int j = index_min; j <= index_max; ++j)
        {
            unsigned short intensity = pSensFrame->GetSensIntensityOfIndex(j);

            if(intensity > 55000)
            {
                intensity = 600;
            }

            if(intensity > 5000)
            {
                intensity = 200 + (intensity - 5000) / 1200;

            }
            else
            {
                intensity = intensity / 25;
            }

            msg.intensities[j - index_min] = intensity;
        }
    }

    msg.range_min = fRangeMin;
    msg.range_max = fRangeMax;
    msg.header.stamp = mNode->now();

    float expected_time_increment = msg.scan_time * msg.angle_increment / (2.0 * M_PI);
    if (fabs(expected_time_increment - msg.time_increment) > 0.00001)
    {
        rclcpp::Clock  debug_clock(RCL_STEADY_TIME);
        RCLCPP_DEBUG_THROTTLE(mNode->get_logger(),debug_clock,60,
                           "The time_increment, scan_time and angle_increment values reported by the scanner are inconsistent! "
                           "Expected time_increment: %.9f, reported time_increment: %.9f. "
                           "Perhaps you should set the parameter time_increment to the expected value. This message will print every 60 seconds.",
                           expected_time_increment,
                           msg.time_increment);
    }

    if(pSensFrame)
    {
        delete pSensFrame;
    }

    return ExitSuccess;
}

void CWrLs1207DEParser::SetRangeMin(float minRange)
{
    fRangeMin = minRange;
}

void CWrLs1207DEParser::SetRangeMax(float maxRange)
{
    fRangeMax = maxRange;
}

void CWrLs1207DEParser::SetTimeIncrement(float time)
{
    fTimeIncrement = time;
}

void CWrLs1207DEParser::SetFrameId(std::string frame_id)
{
    fFrame_id = frame_id;
}

} /*namespace wr_ls_udp*/
