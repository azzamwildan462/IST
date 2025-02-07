#include "hardware/jhctech_291.h"

uint8_t AsciiToHex_Single(uint8_t Ascii_Byte)
{
    return (Ascii_Byte >> 4 == 0B0011 ? Ascii_Byte & 0B00001111 : (Ascii_Byte & 0B00001111) + 0B1001);
}
uint8_t HexToAscii_Single(uint8_t Hex_Byte)
{
    return Hex_Byte >> 3 == 0B0000 ? Hex_Byte | 0B00110000 : ((Hex_Byte & 0B0110) == 0B0000 ? Hex_Byte | 0B00110000 : (Hex_Byte - 0B1 & 0B0111) | 0B01000000);
}

int jhctech_Open(char *comportPath)
{
    struct termios options;
    int fd;

    if (comportPath == NULL)
    {
        return -1;
    }

    /* 打开串口 */
    fd = open(comportPath, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0)
    {
        return -1;
    }
    else
    {
        fcntl(fd, F_SETFL, 0);
    }

    // 获取原有串口配置
    if (tcgetattr(fd, &options) < 0)
    {
        close(fd);
        return -1;
    }

    // 修改控制模式，保证程序不会占用串口
    options.c_cflag |= CLOCAL;

    // 修改控制模式，能够从串口读取数据
    options.c_cflag |= CREAD;

    // 设置数据位
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    // 设置奇偶校验位
    options.c_cflag &= ~PARENB;
    // options.c_iflag &= ~INPCK;
    options.c_iflag &= ~(INPCK | BRKINT | ICRNL | ISTRIP | IXON);

    // 设置停止位
    options.c_cflag &= ~CSTOPB;

    // 设置最少字符和等待时间
    options.c_cc[VMIN] = 1;  // 读数据的最小字节数
    options.c_cc[VTIME] = 0; // 等待第1个数据，单位是10s

    // 修改输出模式，原始数据输出
    options.c_oflag &= ~OPOST;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    // 设置波特率
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);

    // 清空终端未完成的数据
    tcflush(fd, TCIFLUSH);

    // 设置新属性
    if (tcsetattr(fd, TCSANOW, &options) < 0)
    {
        close(fd);
        return -1;
    }

    return fd;
}

int jhctech_Close(int fd)
{
    return close(fd);
}

long jhctech_GetComMessage(int fd, uint8_t *msg, int len)
{
    int ret;
    uint8_t buffer[1024] = {0};
    ret = read(fd, buffer, len);
    if (ret < 1)
    {
        return -1;
    }
    memset(msg, '\0', len);
    memcpy(msg, buffer, ret);

    return ret;
}
long jhctech_CheckId(int Id, int pattern, int mask)
{
    int ret, i, id_len;
    for (i = 0; i < 4 * 8; i++)
    {
        if (((mask >> i) & 1) == 1)
        {
            // filter false is not need the message
            if (((Id >> i) & 1) != ((pattern >> i) & 1))
            {
                break;
            }
        }
    }
    if (i == 32)
    {
        return 0;
    }

    return -1;
}

/*
    [in]msg
    [out]res
    [in]pattern
    [in]mask
*/

/*
    [out]can
    [int]Message
    [in]pattern
    [in]mask
        when mask has 1 ,must compare pattern and can_id ,if has a compare is not the same ,refuse
      to receive the com_can message
*/
long jhctech_Receive(Canbus_msg **can, uint8_t *Message)
{

    uint8_t buffer[1024] = {0};
    Canbus_msg *tail = NULL;

    if ((*can) != NULL)
    {
        free(*can);
    }
    memcpy(buffer, Message, sizeof(buffer));
    for (int i = 0; i < 1024;)
    {
        uint32_t ID = 0;
        int len = 0;
        // 判断开头格式表达数据类型和开始
        if (buffer[i] == 't')
        {
            // 判断是否有结束标志
            if (buffer[i + 6] == 0x0D)
            {
                for (int j = 0; j < 3; j++)
                {
                    ID = ID << 4;
                    ID = ID + AsciiToHex_Single(buffer[i + j + 2]);
                }
                if (tail != NULL)
                {
                    tail->Next = (Canbus_msg *)malloc(sizeof(Canbus_msg));
                    tail = tail->Next;
                    tail->Next = NULL;
                }
                else
                {
                    tail = (Canbus_msg *)malloc(sizeof(Canbus_msg));
                    (*can) = tail;
                    tail->Next = NULL;
                }
                tail->canStart = 't';
                tail->dataLength = 0;
            }
            else
            {
                // 判断是否有结束标志
                if (buffer[i + 2 * AsciiToHex_Single(buffer[i + 5]) + 6] != 0x0D)
                {
                    i++;
                    continue;
                }
                for (int j = 0; j < 3; j++)
                {
                    ID = ID << 4;
                    ID = ID + AsciiToHex_Single(buffer[i + j + 2]);
                }

                if (tail != NULL)
                {
                    tail->Next = (Canbus_msg *)malloc(sizeof(Canbus_msg));
                    tail = tail->Next;
                    tail->Next = NULL;
                }
                else
                {
                    tail = (Canbus_msg *)malloc(sizeof(Canbus_msg));
                    (*can) = tail;
                    tail->Next = NULL;
                }
                tail->canStart = 't';

                tail->dataLength = AsciiToHex_Single(buffer[i + 5]);
            }
            tail->canPort = AsciiToHex_Single(buffer[i + 1]);
            tail->canId = 0;
            tail->canId = ID;
            tail->cr = 0x0D;

            // 接收数据
            for (int j = 0; j < tail->dataLength; j++)
            {
                tail->data[j] = AsciiToHex_Single(buffer[i + 2 * j + 6]);
                tail->data[j] = ((tail->data[j]) << 4) + AsciiToHex_Single(buffer[i + 2 * j + 6 + 1]);
            }

            i = i + 7 + tail->dataLength;
        }
        else if (buffer[i] == 'T')
        {

            // 判断是否有结束标志
            if (buffer[i + 11] == 0x0D)
            {
                for (int j = 0; j < 8; j++)
                {
                    ID = ID << 4;
                    ID = ID + AsciiToHex_Single(buffer[i + j + 2]);
                }

                if (tail != NULL)
                {
                    tail->Next = (Canbus_msg *)malloc(sizeof(Canbus_msg));
                    tail = tail->Next;
                    tail->Next = NULL;
                }
                else
                {
                    tail = (Canbus_msg *)malloc(sizeof(Canbus_msg));
                    (*can) = tail;
                    tail->Next = NULL;
                }
                tail->canStart = 'T';
                tail->dataLength = 0;
            }
            else
            {
                // 判断是否有结束标志
                if (buffer[i + 2 * AsciiToHex_Single(buffer[i + 10]) + 11] != 0x0D)
                {
                    i++;
                    continue;
                }
                for (int j = 0; j < 8; j++)
                {
                    ID = ID << 4;
                    ID = ID + AsciiToHex_Single(buffer[i + j + 2]);
                }
                if (tail != NULL)
                {
                    tail->Next = (Canbus_msg *)malloc(sizeof(Canbus_msg));
                    tail = tail->Next;
                    tail->Next = NULL;
                }
                else
                {
                    tail = (Canbus_msg *)malloc(sizeof(Canbus_msg));
                    tail->Next = NULL;
                    (*can) = tail;
                }
                tail->canStart = 'T';

                tail->dataLength = AsciiToHex_Single(buffer[i + 10]);
            }
            tail->canPort = AsciiToHex_Single(buffer[i + 1]);
            tail->canId = 0;
            tail->canId = ID;
            tail->cr = 0x0D;

            for (int j = 0; j < tail->dataLength; j++)
            {
                tail->data[j] = AsciiToHex_Single(buffer[i + 2 * j + 11]);
                tail->data[j] = ((tail->data[j]) << 4) + AsciiToHex_Single(buffer[i + 2 * j + 11 + 1]);
            }
            i = i + 11 + tail->dataLength;
        }
        else
        {
            i++;
        }
    }
    return 0;
}

long Serial_SendData(int fd, const uint8_t *data, int len)
{
    int ret = 0;
    ret = write(fd, data, len);

    if (ret != len)
    {
        return -1;
    }

    return ret;
}
long jhctech_SendDataFrame(int fd, char canStart, uint8_t canPort, uint32_t canId, uint8_t *data, int data_len)
{
    uint8_t msg[1024] = {0};
    int len_send = 0;
    int ret = 0;
    // 判断数据长度是否正确
    if (data_len < 0 || data_len > 8)
    {
        return -1;
    }
    msg[0] = canStart;
    msg[1] = (canPort & 0xff) + 0x30;
    // 判断数据类型是标准帧还是扩展帧
    if (canStart == 't')
    {
        // 将id转化为scaii作为发送数据存储
        for (int i = 0; i < 3; i++)
        {
            msg[2 + i] = HexToAscii_Single(canId >> ((2 - i) * 4) & 0xf);
        }
        msg[5] = HexToAscii_Single(data_len);
        // 将数据转化为scaii作为发送数据存储
        for (int i = 0; i < data_len; i++)
        {
            msg[6 + 2 * i] = HexToAscii_Single((data[i] >> 4) & 0xf);
            msg[6 + 2 * i + 1] = HexToAscii_Single(data[i] & 0xf);
        }
        msg[5 + 2 * data_len + 1] = 0x0D;
        len_send = 2 * data_len + 7;
    }
    // 解析扩展帧
    else if (canStart == 'T')
    {
        for (int i = 0; i < 8; i++)
        {
            msg[2 + i] = HexToAscii_Single(canId >> ((7 - i) * 4) & 0xf);
        }
        msg[10] = HexToAscii_Single(data_len);
        for (int i = 0; i < data_len; i++)
        {
            msg[11 + 2 * i] = HexToAscii_Single((data[i] >> 4) & 0xf);
            msg[11 + 2 * i + 1] = HexToAscii_Single(data[i] & 0xf);
        }
        msg[10 + 2 * data_len + 1] = 0x0D;
        len_send = 2 * data_len + 12;
    }
    else
    {
        return -1;
    }

    // 发送数据
    ret = write(fd, msg, len_send);

    if (ret != len_send)
    {
        return -1;
    }

    return ret;
}
long jhctech_SendRemoteFrame(int fd, char canStart, uint8_t canPort, uint32_t canId, uint8_t dlc)
{
    uint8_t msg[1024] = {0};
    int len_send = 0;
    int ret = 0;

    msg[0] = canStart;
    msg[1] = (canPort & 0xff) + 0x30;
    // 判断数据类型是标准帧还是扩展帧
    if (canStart == 'r')
    {
        // 将id转化为scaii作为发送数据存储
        for (int i = 0; i < 3; i++)
        {
            msg[2 + i] = HexToAscii_Single(canId >> ((2 - i) * 4) & 0xf);
        }
        msg[5] = HexToAscii_Single(dlc);

        msg[6] = 0x0D;
        len_send = 7;
    }
    else if (canStart == 'R')
    {
        for (int i = 0; i < 8; i++)
        {
            msg[2 + i] = HexToAscii_Single(canId >> ((7 - i) * 4) & 0xf);
        }
        msg[10] = HexToAscii_Single(dlc);

        msg[11] = 0x0D;
        len_send = 12;
    }
    else
    {
        return -1;
    }
    // 发送数据
    ret = write(fd, msg, len_send);

    if (ret != len_send)
    {
        return -1;
    }

    return 0;
}

// GPIO control
int jhctech_ReadCom(int fd)
{
    uint8_t send_data[3] = {0x47, 0x31, 0x0D};
    uint8_t rec[5] = {0};
    int ret = Serial_SendData(fd, send_data, 3);
    if (3 != ret)
    {
        return -1;
    }
    ret = jhctech_GetComMessage(fd, rec, 5);
    if (ret != 5)
    {
        return -2;
    }
    if (rec[0] != 0x47 || rec[4] != 0x0D)
    {
        return -3;
    }
    return ((AsciiToHex_Single(rec[2])) << 4) + AsciiToHex_Single(rec[3]);
}
int jhctech_SetGpioForAll(int fd, uint8_t level)
{
    uint8_t control[5] = {0x47, 0x30, 0x00, 0x00, 0x0D};
    control[2] = HexToAscii_Single((level >> 4) & 0xf);
    control[3] = HexToAscii_Single(level & 0xf);

    int ret = Serial_SendData(fd, control, 5);
    if (5 != ret)
    {
        return -1;
    }
    return 0;
}
int jhctech_SetGpioForOne(int fd, int pin, int level)
{
    uint8_t control[5] = {0x67, 0x30};
    control[2] = HexToAscii_Single(pin);
    control[3] = HexToAscii_Single(level);
    control[4] = 0x0D;

    int ret = Serial_SendData(fd, control, 5);
    if (5 != ret)
    {
        return -1;
    }
    return 0;
}
int jhctech_ReadOneGpio(int fd, int pin)
{
    uint8_t arr[4] = {0x67, 0x31};
    uint8_t rec[4] = {0};
    int ret;

    arr[2] = HexToAscii_Single(pin);
    arr[3] = 0x0D;
    ret = Serial_SendData(fd, arr, 4);
    if (ret != 4)
    {
        return -1;
    }
    ret = jhctech_GetComMessage(fd, rec, 4);
    if (ret != 4)
    {
        return -2;
    }
    if (rec[0] != 0x67 || rec[3] != 0x0D)
    {
        return -3;
    }
    return rec[2];
}

// BaudRate
int jhctech_GetBaudRate(int fd, uint8_t can_port)
{
    uint8_t setBaudRate[3] = {0x73, 0, 0x0D};
    uint8_t rec[4] = {0};
    setBaudRate[1] = HexToAscii_Single(can_port);
    int ret;

    ret = Serial_SendData(fd, setBaudRate, 3);
    if (ret != 3)
    {
        return -1;
    }
    ret = jhctech_GetComMessage(fd, rec, 4);
    if (ret <= 0)
    {
        return -2;
    }
    if (rec[0] != 0x73 || rec[3] != 0x0D)
    {
        return -3;
    }
    return AsciiToHex_Single(rec[2]);
}
int jhctech_SetBaudRate(int fd, uint8_t can_port, uint8_t baud)
{
    uint8_t arr[4] = {0x53, 0, 0, 0x0D};
    int ret = 0;
    if (baud > 8 || baud < 0)
    {
        return -1;
    }
    arr[1] = HexToAscii_Single(can_port);
    arr[2] = HexToAscii_Single(baud);

    ret = Serial_SendData(fd, arr, 4);
    if (ret != 4)
    {
        return -2;
    }
    return 0;
}

int jhctech_SetFilter(int fd, uint8_t can_port, uint8_t idType, uint8_t index, uint8_t mode, uint32_t id1, uint32_t id2)
{
    uint8_t filter[23] = {0x4D};
    int ret = 0;

    filter[1] = HexToAscii_Single(can_port);

    filter[2] = HexToAscii_Single(idType);

    filter[3] = HexToAscii_Single((index >> 4) & 0xF);
    filter[4] = HexToAscii_Single(index & 0xF);

    filter[5] = HexToAscii_Single(mode);

    for (int i = 0; i < 8; i++)
    {
        filter[i + 6] = HexToAscii_Single((id1 >> (7 - i) * 4) & 0xf);
        filter[i + 6 + 8] = HexToAscii_Single((id2 >> (7 - i) * 4) & 0xf);
    }
    filter[22] = 0x0D;

    ret = Serial_SendData(fd, filter, 23);
    if (ret != 23)
    {
        return -1;
    }
    return 0;
}

// MCU reset
int jhctech_Reset(int fd)
{
    uint8_t reset[2] = {0x41, 0x0D};
    int ret;
    ret = Serial_SendData(fd, reset, 2);
    if (ret == 2)
    {
        return 0;
    }
    return -1;
}
// 获取软件版本号
int jhctech_Version(int fd, int *major, int *minor)
{
    uint8_t arr[2] = {0x56, 0x0D};
    uint8_t startCode[8] = {0x56, 0x65, 0x72, 0x73, 0x69, 0x6f, 0x6e, 0x3a};
    uint8_t rec[11] = {0};
    int ret;

    ret = Serial_SendData(fd, arr, 2);
    if (ret != 2)
    {
        return -1;
    }
    ret = jhctech_GetComMessage(fd, rec, 11);
    if (ret != 11)
    {
        return -2;
    }
    if (strncmp(rec, startCode, 8) == 0)
    {
        *major = AsciiToHex_Single(rec[8]);
        *minor = AsciiToHex_Single(rec[10]);
        return 0;
    }
    return -3;
}
//
uint32_t jhctech_CheckUartError(uint8_t *msg, int len)
{
    uint8_t errorCodeStart[17] = {0x55, 0x41, 0x52, 0x54, 0x20, 0x45, 0x72, 0x72, 0x6F, 0x72, 0x43, 0x6F, 0x64, 0x65, 0x3A, 0x30, 0x78};
    uint32_t errorCode = 0;
    for (int i = 0; i < len - 17 - 8 - 1; i++)
    {
        if (strncmp(msg + i, errorCodeStart, 17) == 0)
        {
            for (int j = 0; j < 8; j++)
            {
                errorCode = (errorCode << 4) + AsciiToHex_Single(msg[i + 17 + j]);
            }
            return errorCode;
        }
    }
    return 0;
}

uint32_t jhctech_CheckCan0Error(uint8_t *msg, int len)
{
    uint8_t errorCodeStart[17] = {0x43, 0x41, 0x4E, 0x30, 0x20, 0x45, 0x72, 0x72, 0x6F, 0x72, 0x43, 0x6F, 0x64, 0x65, 0x3A, 0x30, 0x78};
    uint32_t errorCode = 0;
    for (int i = 0; i < len - 17 - 8 - 1; i++)
    {
        if (strncmp(msg + i, errorCodeStart, 17) == 0)
        {
            for (int j = 0; j < 8; j++)
            {
                errorCode = (errorCode << 4) + AsciiToHex_Single(msg[i + 17 + j]);
            }
            return errorCode;
        }
    }
    return 0;
}

uint32_t jhctech_CheckCan1Error(uint8_t *msg, int len)
{
    uint8_t errorCodeStart[17] = {0x43, 0x41, 0x4E, 0x31, 0x20, 0x45, 0x72, 0x72, 0x6F, 0x72, 0x43, 0x6F, 0x64, 0x65, 0x3A, 0x30, 0x78};
    uint32_t errorCode = 0;
    for (int i = 0; i < len - 17 - 8 - 1; i++)
    {
        if (strncmp(msg + i, errorCodeStart, 17) == 0)
        {
            for (int j = 0; j < 8; j++)
            {
                errorCode = (errorCode << 4) + AsciiToHex_Single(msg[i + 17 + j]);
            }
            return errorCode;
        }
    }
    return 0;
}
