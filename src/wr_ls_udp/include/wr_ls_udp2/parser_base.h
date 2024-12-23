#ifndef PARSER_BASE__
#define PARSER_BASE__

//#include "wr_ls_udp/WrLsConfig.h"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace wr_ls_udp
{
enum ExitCode
{
    ExitSuccess = 0,
    ExitError   = 1,
    ExitFatal   = 2
};
struct WrLsConfig
{
    double  min_ang;
    double  max_ang;
    bool    intensity;
    int     skip;
    double  time_offset;
    bool    auto_reboot;
    bool    debug_mode; 
};

class CParserBase
{
public:
    CParserBase();
    virtual ~CParserBase();

    virtual int Parse(char *data,
                      size_t data_length,
                      WrLsConfig &config,
                      sensor_msgs::msg::LaserScan &msg) = 0;
};
} /*namespace wr_ls_udp*/

#endif /*PARSER_BASE__*/
