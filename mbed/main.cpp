#include "eeprom.h"
#include "mbed.h"
#include <cstdio>

// int sampled_time[5000];

// Serial pc(USBTX, USBRX);

int main(void)
{
    EEPROM eeprom(p9, p10);

    const int stride = 10;
    printf("address,pressure,roll,pitch,yaw\r\n");
    for (int i = 0; i <= 0xFFFF; i+=stride)
    {
        char data_array[10] = {0, };

        int ret = eeprom.read_block_data(i, data_array, 10);

        int time = ((int)data_array[0] << 24) | ((int)data_array[1] << 16) | ((int)data_array[2] << 8) | (int)data_array[3];
        int roll = ((int)data_array[4] << 8) | (int)data_array[5];
        int pitch = ((int)data_array[6] << 8) | (int)data_array[7];
        int yaw = ((int)data_array[8] << 8) | (int)data_array[9];

        if (roll >= (1<<15)) {
            roll -= (1<<16);
        }
        if (pitch >= (1<<15)) {
            pitch -= (1<<16);
        }
        if (yaw >= (1<<15)) {
            yaw -= (1<<16);
        }

        printf("[ %3d ] 0x%04X, %10d, %5.2lf, %5.2lf, %5.2lf \r\n", ret, i, time, (float)roll / 100, (float)pitch / 100, (float)yaw / 100);
        wait_us(10000);
    }
}
