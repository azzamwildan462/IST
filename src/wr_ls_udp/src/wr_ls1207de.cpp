#include "wr_ls_udp2/wr_ls_common_udp.h"
#include "wr_ls_udp2/wr_ls1207de_parser.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("wr_ls_udp_node",rclcpp::NodeOptions(rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)));

    /*Check whether hostname is provided*/
    bool isTcpConnection = false;
    std::string strHostName;
    std::string strPort;

    node->get_parameter_or<std::string>("hostname", strHostName, "192.168.0.10");
    isTcpConnection = true;
    node->get_parameter_or<std::string>("port",strPort, "2112");

    /*Get configured time limit*/
    int iTimeLimit = 5;
    node->get_parameter_or<int>("timelimit",iTimeLimit, 5);

    bool isDataSubscribed = false;
    node->get_parameter_or<bool>("subscribe_datagram",isDataSubscribed, false);

    int iDeviceNumber = 0;
    node->get_parameter_or<int>("device_number",iDeviceNumber, 0);

    /*Create and initialize parser*/
    wr_ls_udp::CWrLs1207DEParser *pParser = new wr_ls_udp::CWrLs1207DEParser(node);

    double param;
    std::string frame_id;

    if(node->get_parameter_or<double>("range_min",param, 0.1))
    {
        RCLCPP_INFO(node->get_logger(),"range_min: %f", param);
        pParser->SetRangeMin(param);
    }

    if(node->get_parameter_or<double>("range_max",param, 20))
    {
        RCLCPP_INFO(node->get_logger(),"range_max: %f", param);
        pParser->SetRangeMax(param);
    }

    if(node->get_parameter_or<double>("time_increment",param, 0.000061722))
    {
        RCLCPP_INFO(node->get_logger(),"time_increment: %f", param);
        pParser->SetTimeIncrement(param);
    }

    if(node->get_parameter_or<std::string>("frame_id",frame_id, "laser_link"))
    {
        RCLCPP_INFO(node->get_logger(),"frame_id: %s", frame_id.c_str());
        pParser->SetFrameId(frame_id);
    }

    wr_ls_udp::CWrLsCommon * pWrLs = NULL;
    int result = wr_ls_udp::ExitError;
    while(rclcpp::ok())
    {
        if(pWrLs != NULL)
        {
            delete pWrLs;
        }

        pWrLs = new wr_ls_udp::CWrLsCommonUdp(strHostName, strPort, iTimeLimit, pParser, node);
        result = pWrLs->Init();

        /*Device has been initliazed successfully*/
        while(rclcpp::ok() && (result == wr_ls_udp::ExitSuccess))
        {
            rclcpp::spin_some(node);
            result = pWrLs->LoopOnce();
        }

        if(result == wr_ls_udp::ExitFatal)
        {
            return result;
        }
    }

    if(pWrLs != NULL)
    {
        delete pWrLs;
    }

    if(pParser != NULL)
    {
        delete pParser;
    }

    return result;
}
