#ifndef __ECB_306_H__
#define __ECB_306_H__

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <string.h>
#include <sys/ioctl.h>

/* gpio */
typedef enum
{
    PIN_SET = 0U,
    PIN_RESET
} GPIO_PinState;

typedef struct OUTPUTPIN_TypeDef
{
    uint8_t OUTPUT0;
    uint8_t OUTPUT1;
    uint8_t OUTPUT2;
    uint8_t OUTPUT3;
    uint8_t OUTPUT4;
    uint8_t OUTPUT5;
    uint8_t OUTPUT6;
    uint8_t OUTPUT7;
    //    uint8_t OUTPUT8;         uint8_t OUTPUT9;
    //    uint8_t OUTPUT10;        uint8_t OUTPUT11;
    //    uint8_t OUTPUT12;        uint8_t OUTPUT13;
    //    uint8_t OUTPUT14;        uint8_t OUTPUT15;
} OUTPUTPIN_TypeDef;

typedef struct INPUTPIN_TypeDef
{
    uint8_t INPUT0;
    uint8_t INPUT1;
    uint8_t INPUT2;
    uint8_t INPUT3;
    uint8_t INPUT4;
    uint8_t INPUT5;
    uint8_t INPUT6;
    uint8_t INPUT7;
    //    uint8_t INPUT8;          uint8_t INPUT9;
    //    uint8_t INPUT10;         uint8_t INPUT11;
    //    uint8_t INPUT12;         uint8_t INPUT13;
    //    uint8_t INPUT14;         uint8_t INPUT15;
} INPUTPIN_TypeDef;

/* can */

/**
 * can ide type
 */
#define CAN_STANDARD_ID ((uint8_t)0x00U) /*!< Standard ID element 11bit*/
#define CAN_EXTENDED_ID ((uint8_t)0x01U) /*!< Extended ID element 29bit*/

/**
 * can rtr type
 */
#define CAN_DATA_FRAME ((uint32_t)0x00U)   /*!< Data frame   */
#define CAN_REMOTE_FRAME ((uint32_t)0x01U) /*!< Remote frame */

typedef enum
{
    CAN1 = 0x31U, //
    CAN2 = 0x32U
} CanPort;

typedef struct CanMessage
{
    CanPort CanNumber;
    uint32_t CanId;
    uint8_t Ide;
    uint8_t Rtr;
    uint8_t Dlc;
    uint8_t Data[8];
} CanMessage;

typedef enum
{
    BaudRate_10k = 0x30U, //
    BaudRate_20k,
    BaudRate_50k,
    BaudRate_100k,
    BaudRate_125k,
    BaudRate_250k,
    BaudRate_500k,
    BaudRate_800k,
    BaudRate_1M
} CanBaudRate;

int jhctech_Serial_Init(char *ComPortPath);
int jhctech_Serial_Close(int ComPort);
/* gpio */
int jhctech_GPIO_Write(int ComPort, OUTPUTPIN_TypeDef OUTPUTPIN);
int jhctech_GPIO_Write_Single(int ComPort, uint8_t PinNum, GPIO_PinState PinState);
int jhctech_GPIO_Read(int ComPort, INPUTPIN_TypeDef *INPUTPIN);
GPIO_PinState jhctech_GPIO_Read_Single(int ComPort, uint8_t PinNum);
/* can */
int jhctech_CANBUS_Send(int ComPort, CanMessage TxMessage);
int jhctech_CANBUS_Receive(int ComPort, CanMessage *RxMessage);
int jhctech_CANBUS_SetBaudRate(int ComPort, CanPort CanNumber, CanBaudRate BaudRate);
CanBaudRate jhctech_CANBUS_GetBaudRate(int ComPort, CanPort CanNumber);

#endif /* __ECB_306_H__ */
