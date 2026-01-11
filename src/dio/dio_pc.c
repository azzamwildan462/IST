#include "ecb_306.h"

int main()
{
    int ComPort;

    OUTPUTPIN_TypeDef OUTPUTPIN;
    INPUTPIN_TypeDef INPUTPIN;

    const char default_path[] = "/dev/ttyACM0";
    char* path;

    path = (char*)default_path;
    ComPort = jhctech_Serial_Init(path);

    while (1) {
        static uint8_t value = PIN_SET;
        OUTPUTPIN.OUTPUT0 = value;
        OUTPUTPIN.OUTPUT1 = !value;
        OUTPUTPIN.OUTPUT2 = value;
        OUTPUTPIN.OUTPUT3 = !value;
        OUTPUTPIN.OUTPUT4 = value;
        OUTPUTPIN.OUTPUT5 = !value;
        OUTPUTPIN.OUTPUT6 = value;
        OUTPUTPIN.OUTPUT7 = !value;
        jhctech_GPIO_Write(ComPort, OUTPUTPIN);

        jhctech_GPIO_Read(ComPort, &INPUTPIN);
        printf("INPUTPIN0:%X\r\n", INPUTPIN.INPUT0);
        printf("INPUTPIN1:%X\r\n", INPUTPIN.INPUT1);
        printf("INPUTPIN2:%X\r\n", INPUTPIN.INPUT2);
        printf("INPUTPIN3:%X\r\n", INPUTPIN.INPUT3);
        printf("INPUTPIN4:%X\r\n", INPUTPIN.INPUT4);
        printf("INPUTPIN5:%X\r\n", INPUTPIN.INPUT5);
        printf("INPUTPIN6:%X\r\n", INPUTPIN.INPUT6);
        printf("INPUTPIN7:%X\r\n", INPUTPIN.INPUT7);

        sleep(1);
        value = !value;
    }

    jhctech_Serial_Close(ComPort);
    return 0;
}
