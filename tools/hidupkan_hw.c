#include "ecb_306.h"

int main(int argc, char *argv[])
{
    int ComPort; /* 端口号 */

    OUTPUTPIN_TypeDef OUTPUTPIN;
    INPUTPIN_TypeDef INPUTPIN;

    const char default_path[] = "/dev/ttyACM0";
    char *path;
    // 若无输入参数则使用默认终端设备
    if (argc > 1)
        path = argv[1];
    else
        path = (char *)default_path;

    ComPort = jhctech_Serial_Init(path);

    OUTPUTPIN.OUTPUT0 = PIN_SET;
    OUTPUTPIN.OUTPUT1 = PIN_RESET;
    OUTPUTPIN.OUTPUT2 = PIN_RESET;
    OUTPUTPIN.OUTPUT3 = PIN_RESET;
    OUTPUTPIN.OUTPUT4 = PIN_RESET;
    OUTPUTPIN.OUTPUT5 = PIN_RESET;
    OUTPUTPIN.OUTPUT6 = PIN_RESET;
    OUTPUTPIN.OUTPUT7 = PIN_RESET;
    // OUTPUTPIN.OUTPUT8 = PIN_SET;      OUTPUTPIN.OUTPUT9 = PIN_SET;
    // OUTPUTPIN.OUTPUT10 = PIN_SET;     OUTPUTPIN.OUTPUT11 = PIN_SET;
    // OUTPUTPIN.OUTPUT12 = PIN_SET;     OUTPUTPIN.OUTPUT13 = PIN_SET;
    // OUTPUTPIN.OUTPUT14 = PIN_SET;     OUTPUTPIN.OUTPUT15 = PIN_SET;

    jhctech_GPIO_Write(ComPort, OUTPUTPIN);

    //printf("Now all outputs are low.\r\n");
    //printf("Press Enter key to continue.\r\n");
    //getchar();

    sleep(1);

    OUTPUTPIN.OUTPUT0 = PIN_RESET;
    OUTPUTPIN.OUTPUT1 = PIN_SET;
    // OUTPUTPIN.OUTPUT2 = PIN_SET;
    // OUTPUTPIN.OUTPUT3 = PIN_SET;
    // OUTPUTPIN.OUTPUT4 = PIN_SET;
    // OUTPUTPIN.OUTPUT5 = PIN_SET;
    // OUTPUTPIN.OUTPUT6 = PIN_SET;
    // OUTPUTPIN.OUTPUT7 = PIN_SET;
    // OUTPUTPIN.OUTPUT8 = PIN_RESET;      OUTPUTPIN.OUTPUT9 = PIN_RESET;
    // OUTPUTPIN.OUTPUT10 = PIN_RESET;     OUTPUTPIN.OUTPUT11 = PIN_RESET;
    // OUTPUTPIN.OUTPUT12 = PIN_RESET;     OUTPUTPIN.OUTPUT13 = PIN_RESET;
    // OUTPUTPIN.OUTPUT14 = PIN_RESET;     OUTPUTPIN.OUTPUT15 = PIN_RESET;

    jhctech_GPIO_Write(ComPort, OUTPUTPIN);

    printf("NOW all outputs are high.\r\n");

    usleep(700000);

    OUTPUTPIN.OUTPUT0 = PIN_RESET;
    OUTPUTPIN.OUTPUT1 = PIN_RESET;
    // OUTPUTPIN.OUTPUT2 = PIN_SET;
    // OUTPUTPIN.OUTPUT3 = PIN_SET;
    // OUTPUTPIN.OUTPUT4 = PIN_SET;
    // OUTPUTPIN.OUTPUT5 = PIN_SET;
    // OUTPUTPIN.OUTPUT6 = PIN_SET;
    // OUTPUTPIN.OUTPUT7 = PIN_SET;
    // OUTPUTPIN.OUTPUT8 = PIN_RESET;      OUTPUTPIN.OUTPUT9 = PIN_RESET;
    // OUTPUTPIN.OUTPUT10 = PIN_RESET;     OUTPUTPIN.OUTPUT11 = PIN_RESET;
    // OUTPUTPIN.OUTPUT12 = PIN_RESET;     OUTPUTPIN.OUTPUT13 = PIN_RESET;
    // OUTPUTPIN.OUTPUT14 = PIN_RESET;     OUTPUTPIN.OUTPUT15 = PIN_RESET;

    jhctech_GPIO_Write(ComPort, OUTPUTPIN);

    printf("NOW all outputs are low.\r\n");

    // printf("Press Enter key to continue,Next test single output.\r\n");
    // getchar();

    /* 0x00~0x07 */
    /*
    for (uint8_t i = 0; i < 0x10; i++)
    {
        jhctech_OUTPUT_Single(fd, i, PIN_SET);
        printf("Pin%d is high\r\n",i);
        usleep(200000);   // us
    }

    usleep(500000);

    for (uint8_t i = 0; i < 0x10; i++)
    {
        jhctech_OUTPUT_Single(fd, i, PIN_RESET);
        printf("Pin%d is low\r\n",i);
        usleep(200000);
    }
   */
    // printf("Press Enter key to continue,Next test input.\r\n");
    // getchar();
    // char ch;
    // do
    // {
    //     jhctech_GPIO_Read(ComPort, &INPUTPIN);
    //     printf("INPUTPIN0:%X\r\n", INPUTPIN.INPUT0);
    //     printf("INPUTPIN1:%X\r\n", INPUTPIN.INPUT1);
    //     printf("INPUTPIN2:%X\r\n", INPUTPIN.INPUT2);
    //     printf("INPUTPIN3:%X\r\n", INPUTPIN.INPUT3);
    //     printf("INPUTPIN4:%X\r\n", INPUTPIN.INPUT4);
    //     printf("INPUTPIN5:%X\r\n", INPUTPIN.INPUT5);
    //     printf("INPUTPIN6:%X\r\n", INPUTPIN.INPUT6);
    //     printf("INPUTPIN7:%X\r\n", INPUTPIN.INPUT7);
    //     // printf("INPUTPIN8:%X\r\n", INPUTPIN.INPUT8);
    //     // printf("INPUTPIN9:%X\r\n", INPUTPIN.INPUT9);
    //     // printf("INPUTPIN10:%X\r\n", INPUTPIN.INPUT10);
    //     // printf("INPUTPIN11:%X\r\n", INPUTPIN.INPUT11);
    //     // printf("INPUTPIN12:%X\r\n", INPUTPIN.INPUT12);
    //     // printf("INPUTPIN13:%X\r\n", INPUTPIN.INPUT13);
    //     // printf("INPUTPIN14:%X\r\n", INPUTPIN.INPUT14);
    //     // printf("INPUTPIN15:%X\r\n", INPUTPIN.INPUT15);
    //     printf("Press Enter to continue,Other&Enter will exit.\r\n");
    //     ch = getchar();

    // } while (ch == '\n');

    // /*printf("Press Enter key to continue,Next test single input.\r\n");
    // getchar();

    // for (uint8_t i = 0; i < 0x10; i++)
    // {
    //     printf("INPUTPIN%d:%X\r\n",i, jhctech_INPUT_Read_Single(fd, i));
    //     usleep(200000);
    // }
    // */
    // printf("This test is completed.\r\n");
    // usleep(100000);

    jhctech_Serial_Close(ComPort);

    return 0;
}
