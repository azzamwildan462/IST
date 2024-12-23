#ifndef WR_LS1207DE_PARSER__
#define WR_LS1207DE_PARSER__

#include "wr_ls_udp2/parser_base.h"
#include "wr_ls_udp2/wr_ls_common.h"
#include <rclcpp/rclcpp.hpp>

namespace wr_ls_udp
{
class CWrLs1207DEParser : public CParserBase
{
public:
    CWrLs1207DEParser(rclcpp::Node::SharedPtr node);
    virtual ~CWrLs1207DEParser();

    virtual int Parse(char *data, size_t data_length, WrLsConfig &config, sensor_msgs::msg::LaserScan &msg);

    void SetRangeMin(float minRange);
    void SetRangeMax(float maxRange);
    void SetTimeIncrement(float time);
    void SetFrameId(std::string str);

private:
    float fRangeMin;
    float fRangeMax;
    float fTimeIncrement;
    std::string fFrame_id;
    rclcpp::Node::SharedPtr mNode;
    
};
} /*namespace wr_ls_udp*/

#endif /*WR_LS1207DE_PARSER__*/
