#ifndef __EEPROM_H__
#define __EEPROM_H__

#include "mbed.h"

class EEPROM
{
private:
    I2C i2c_port;
    static const int EEPROM_8BIT_ADDR = 0xA0;
    int eeprom_memory_addr;
    Timer tictoc;
    
public:
    enum EEPROM_errnum {
        NORMAL=0,
        NOT_READY_YET,
        MEM_FULL,
        OUT_OF_RANGE,
        NACK,
        MEM_ALLOC_FAILED=11,
    };

    union WordToByte {
        int16_t word_array[5];
        char    byte_array[10];
    };

    EEPROM(PinName sda, PinName scl);

    int write_8(const int address, char data);
    int write_16(const int address, int data);
    int write_block_data(const int address, char *data_array, const int size);

    int read_8(const int address);
    int read_16(const int address);
    int read_block_data(const int address, char *received_buffer, const int size);

    bool ready(void);
    int init(void);

    int save_to_eeprom(Serial& pc, int32_t pressure, float roll, float pitch, float yaw);

    EEPROM_errnum get_EEPROM_errnum(void);

protected:
    bool check_EEPROM_mem_address(const int address);
    EEPROM_errnum errnum;

}; // EEPROM

#endif
