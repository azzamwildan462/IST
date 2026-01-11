#include "ecb_306.h"
#include "jhctech_291.h"

#define COB_ID_CAR_ENCODER 0x388

int set_can_jhctech_bitrate_id(int ComPort, int bitrate)
{
    int ret_buffer = 0;
    uint8_t baud = 0;
    switch (bitrate)
    {
    case 10000:
        baud = 0;
        break;
    case 20000:
        baud = 1;
        break;
    case 50000:
        baud = 2;
        break;
    case 100000:
        baud = 3;
        break;
    case 125000:
        baud = 4;
        break;
    case 250000:
        baud = 5;
        break;
    case 500000:
        baud = 6;
        break;
    case 800000:
        baud = 7;
        break;
    case 1000000:
        baud = 8;
        break;
    default:
        ret_buffer = -1;
        break;
    }

    ret_buffer = jhctech_SetBaudRate(ComPort, 1, baud);
    sleep(1);
    ret_buffer = jhctech_SetBaudRate(ComPort, 2, baud);

    return ret_buffer;
}

int main(int argc, char *argv[])
{
    printf("INIT AUTO MATI\n");
    uint8_t retry_num = 0;
    int ComPort; /* 端口号 */

    OUTPUTPIN_TypeDef OUTPUTPIN;
    INPUTPIN_TypeDef INPUTPIN;

    const char default_path[] = "/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_2074315D574B-if00";
    char *path;
    // 若无输入参数则使用默认终端设备
    if (argc > 1)
        path = argv[1];
    else
        path = (char *)default_path;

    printf("Opening\n");

    // ComPort = jhctech_Serial_Init(path);
    ComPort = jhctech_Open(path);
    static const int max_retry = 7;

    printf("asdasdasd %d\n", ComPort);
    while (ComPort < 0 && retry_num < max_retry)
    {
        retry_num++;
        printf("Failed to open %s. Retrying in 5 seconds...\n", path);
        sleep(5);
        ComPort = jhctech_Open(path);
    }
    if (retry_num >= max_retry)
    {
        printf("Failed to open %s after multiple attempts. Exiting.\n", path);
        return -1;
    }

    printf("Sukses open port\n");

    int set_baud = set_can_jhctech_bitrate_id(ComPort, 125000);

    printf("baud %d\n", set_baud);
    if (set_baud < 0)
    {
        printf("Failed to set CAN bitrate.\n");
        return -1;
    }

    printf("Sukses open baud\n");

    uint64_t counter_tidak_ada_can_mobil = 0;
    uint8_t status_mobil_hidup = 0;
    while (1)
    {
        // Menunggu membaca can mobil
        static uint8_t can_data_buffer[1024] = {0}; // Raw serial data
        static Canbus_msg *can_data = NULL;

        {
            int ret_buffer = jhctech_GetComMessage(ComPort, can_data_buffer, 1024);
            uint32_t errorCode = 0;

            if (0 != (errorCode = jhctech_CheckUartError(can_data_buffer, ret_buffer)))
            {
                printf("UART errorCode is 0x%08x", errorCode);
            }
            if (0 != (errorCode = jhctech_CheckCan0Error(can_data_buffer, ret_buffer)))
            {
                printf("CAN0 errorCode is 0x%08x", errorCode);
            }
            if (0 != (errorCode = jhctech_CheckCan1Error(can_data_buffer, ret_buffer)))
            {
                printf("CAN1 errorCode is 0x%08x", errorCode);
            }
        }

        int ret_buffer = jhctech_Receive(&can_data, can_data_buffer);
        (void)ret_buffer;

        while (can_data != NULL)
        {
            if (can_data->canId == COB_ID_CAR_ENCODER)
            {
                printf("canid: %x\n", can_data->canId);
                counter_tidak_ada_can_mobil = 0;
                status_mobil_hidup = 1;
                break;
            }
            counter_tidak_ada_can_mobil++;
            can_data = can_data->Next;
        }
        counter_tidak_ada_can_mobil++;

        // Jika 10 detik tidak ada can mobil, keluar dari loop
        if (counter_tidak_ada_can_mobil > 10000)
        {
            break;
        }

        if (status_mobil_hidup == 1)
        {
            break;
        }
    }

    if (status_mobil_hidup == 1)
    {
        printf("Mobil terdeteksi masih hidup, tidak melakukan auto mati.\n");
        return 0;
    }

    printf("Melakukan auto mati...\n");

    OUTPUTPIN.OUTPUT0 = PIN_RESET;
    OUTPUTPIN.OUTPUT1 = PIN_RESET;
    jhctech_GPIO_Write(ComPort, OUTPUTPIN);
    sleep(5);
    printf("Set data low sukses\n");

    OUTPUTPIN.OUTPUT0 = PIN_RESET;
    OUTPUTPIN.OUTPUT1 = PIN_SET;
    jhctech_GPIO_Write(ComPort, OUTPUTPIN);
    usleep(700000);
    printf("Set clk high sukses\n");

    OUTPUTPIN.OUTPUT0 = PIN_RESET;
    OUTPUTPIN.OUTPUT1 = PIN_RESET;
    jhctech_GPIO_Write(ComPort, OUTPUTPIN);
    usleep(100000);
    printf("Set clk low sukses\n");

    jhctech_Serial_Close(ComPort);
    printf("Matikan PC\n");

    // sleep(30);
    // system("sudo poweroff");

    return 0;
}
