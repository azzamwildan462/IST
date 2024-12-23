#ifndef WR_LS_COMMON_UDP__
#define WR_LS_COMMON_UDP__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "wr_ls_udp2/wr_ls_common.h"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

namespace wr_ls_udp
{
class CWrLsCommonUdp : public CWrLsCommon
{
public:
    CWrLsCommonUdp(const std::string &hostname, const std::string &port, int &timelimit, CParserBase *parser, std::shared_ptr<rclcpp::Node> nh);
    virtual ~CWrLsCommonUdp();

protected:
    /*Override functions*/
    virtual int InitDevice();
    virtual int CloseDevice();

    virtual int SendDeviceReq(const char *req, std::vector<unsigned char> *resp);

    virtual int GetDataGram(unsigned char *receiveBuffer, int bufferSize, int *length);

    int SendUdpData2Device(char *buf, int length);
private:
    int                          mSocket;
    size_t                       mBytesReceived;
    std::string                  mHostName;
    std::string                  mPort;
    int                          mTimeLimit;
    struct sockaddr_in           remoteServAddr;

    rclcpp::Node::SharedPtr      mNode_;

};

} /*namespace wr_ls_udp*/

#endif /*WR_LS_COMMON_TCP__*/
