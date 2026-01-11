#ifndef __JHCTECH_1_H_
#define __JHCTECH_1_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <termios.h>
#include <string.h>
#include <stdlib.h>
#include <malloc.h>

    uint8_t AsciiToHex_Single(uint8_t Ascii_Byte);
    uint8_t HexToAscii_Single(uint8_t Hex_Byte);

    /*
        [in]fd
        115200, 8, N, 1
        return
            if the function succeeds ,return the file descriptor ,if the function fails , return value is less then zero.
    */
    int jhctech_Open(char *comportPath);

    /*
        if the function succeeds ,return zero ,else return minus one
    */
    int jhctech_Close(int fd);

    /*
        [in] fd
        [out] msg
        [in] len
        if the function succeeds ,return bytes ,else return -1
    */
    long jhctech_GetComMessage(int fd, uint8_t *msg, int len);

    /*
        [in]Id
        [in]pattern
        [in]mask
            if check pass ,return 0,else return -1.
    */
    long jhctech_CheckId(int Id, int pattern, int mask);

    // the struct is reseive Canbus message
    typedef struct _Canbus_msg
    {
        struct _Canbus_msg *Next;
        uint16_t canStart;
        uint8_t canPort;
        uint32_t canId;
        uint8_t dataLength;
        uint8_t data[8];
        uint8_t cr;
    } Canbus_msg;

    long Serial_SendData(int fd, const uint8_t *data, int len);
    long jhctech_Receive(Canbus_msg **can, uint8_t *Message);
    long jhctech_SendDataFrame(int fd, char canStart, uint8_t canPort, uint32_t canId, uint8_t *data, int data_len);
    long jhctech_SendRemoteFrame(int fd, char canStart, uint8_t canPort, uint32_t canId, uint8_t dlc);

    // GPIO contorl
    // 设置整个串口gpio,返回-1表示错误,0表示正确
    int jhctech_SetGpioForAll(int fd, uint8_t level);
    // 设置串口单个gpio的状态.同上
    int jhctech_SetGpioForOne(int fd, int pin, int level);
    // 获取整个串口的电平状态 , 返回负数表明发生错误,否则返回串口电平状态
    int jhctech_ReadCom(int fd);
    // 读当个gpio状态,返回负数表明发生错误，否则返回对应gpio的电平
    int jhctech_ReadOneGpio(int fd, int pin);

    // Set BaudRate
    /*
        10k	0
        20k	1
        50k	2
        100k	3
        125k	4
        250k	5
        500k	6
        800k	7
        1M	8

    */
    // 获取波特率函数，发生错误返回负数,否则返回对应波特率代码
    int jhctech_GetBaudRate(int fd, uint8_t can_port);
    // 设置波特率,错误则返回负数，否则返回0
    int jhctech_SetBaudRate(int fd, uint8_t can_port, uint8_t baud);

// filter set
// CAN MODE
#define FDCAN_FILTER_RANGE 0
#define FDCAN_FILTER_DUAL 1
#define FDCAN_FILTER_MASK 2
#define FDCAN_FILTER_RANGE_NO_EIDM 3

    typedef struct
    {
        uint32_t IdType;
        uint32_t FilterIndex;
        uint32_t FilterType;
        uint32_t FilterConfig;
        uint32_t FilterID1;
        uint32_t FilterID2;
    } FDCAN_FilterTypeDef;

    // 设置硬件过滤,成功返回0,否则返回-1
    int jhctech_SetFilter(int fd, uint8_t can_port, uint8_t idType, uint8_t index, uint8_t mode, uint32_t id1, uint32_t id2);

    // MCU reset硬件重启,成功则返回0,否则返回-1
    int jhctech_Reset(int fd);

    // 获取软件版本号,成功返回0,版本为major , 次版本为minor,否则返回负数
    int jhctech_Version(int fd, int *major, int *minor);

// UART error code:
#define UART_ERROR_NONE (0x00000000U) /*!< No error                */
#define UART_ERROR_PE (0x00000001U)   /*!< Parity error            */
#define UART_ERROR_NE (0x00000002U)   /*!< Noise error             */
#define UART_ERROR_FE (0x00000004U)   /*!< Frame error             */
#define UART_ERROR_ORE (0x00000008U)  /*!< Overrun error           */
#define UART_ERROR_DMA (0x00000010U)  /*!< DMA transfer error      */
#define UART_ERROR_RTO (0x00000020U)  /*!< Receiver Timeout error  */

// FDCAN error code :
#define FDCAN_ERROR_NONE ((uint32_t)0x00000000U)            /*!< No error                          */
#define FDCAN_ERROR_TIMEOUT ((uint32_t)0x00000001U)         /*!< Timeout error                    */
#define FDCAN_ERROR_NOT_INITIALIZED ((uint32_t)0x00000002U) /*!< Peripheral not initialized     */
#define FDCAN_ERROR_NOT_READY ((uint32_t)0x00000004U)       /*!< Peripheral not ready             */
#define FDCAN_ERROR_NOT_STARTED ((uint32_t)0x00000008U)     /*!< Peripheral not started           */
#define FDCAN_ERROR_NOT_SUPPORTED ((uint32_t)0x00000010U)   /*!< Mode not supported               */
#define FDCAN_ERROR_PARAM ((uint32_t)0x00000020U)           /*!< Parameter error                   */
#define FDCAN_ERROR_PENDING ((uint32_t)0x00000040U)         /*!< Pending operation                 */
#define FDCAN_ERROR_RAM_ACCESS ((uint32_t)0x00000080U)      /*!< Message RAM Access Failure       */
#define FDCAN_ERROR_FIFO_EMPTY ((uint32_t)0x00000100U)      /*!< Put element in full FIFO         */
#define FDCAN_ERROR_FIFO_FULL ((uint32_t)0x00000200U)       /*!< Get element from empty FIFO      */
    // 检查是否发送错误，是则返回错误码
    uint32_t jhctech_CheckUartError(uint8_t *msg, int len);
    uint32_t jhctech_CheckCan0Error(uint8_t *msg, int len);
    uint32_t jhctech_CheckCan1Error(uint8_t *msg, int len);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
