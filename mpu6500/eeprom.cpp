#include "./eeprom.h"
#include "mbed.h"
#include <cstring> // For memset and memcpy
#include <cstdlib> // For malloc and free

/**
 * @brief Constructor for the EEPROM class.
 * 
 * Initializes the I2C communication port with the specified SDA and SCL pins and sets the I2C frequency to 100kHz.
 * 
 * @param sda PinName for the SDA (Serial Data) line.
 * @param scl PinName for the SCL (Serial Clock) line.
 */
EEPROM::EEPROM(PinName sda, PinName scl) : i2c_port(sda, scl), errnum(NORMAL) {
    i2c_port.frequency(100000); // 100kHz
    eeprom_memory_addr = 0x0000;
    tictoc.start();
}

/**
 * @brief Reads an 8-bit value from the specified EEPROM address.
 * 
 * Verifies if the EEPROM is ready and the specified memory address is valid before attempting to read.
 * Returns the 8-bit value read from the EEPROM at the specified address. In case of error, such as communication failure
 * or invalid address, returns -1.
 * 
 * @param address The memory address to read from.
 * @return int The 8-bit value read, or -1 in case of error.
 */
int EEPROM::read_8(const int address) {
    char command[2];
    char received;
    int ack;
    int read_ack;

    if (!check_EEPROM_mem_address(address))
        return -1;
    // Prepare the command to set the EEPROM read address
    command[0] = (address >> 8) & 0xFF; // High byte of the address
    command[1] = address & 0xFF;        // Low byte of the address
    ack = this->i2c_port.write(this->EEPROM_8BIT_ADDR, command, 2);
    read_ack = this->i2c_port.read(this->EEPROM_8BIT_ADDR, &received, 1);
    if (ack || read_ack) {
        this->errnum = this->NACK;
        #if DEBUG
            printf("[%s] ack: %d, read_ack: %d\r\n", __FUNCTION__, ack, read_ack);
        #endif
        return -1;
    }
    return received;
}

/**
 * @brief Reads a 16-bit value from the specified EEPROM address.
 * 
 * Checks if the EEPROM is ready and if the memory address plus one is valid, then reads a 16-bit value starting
 * from the specified address. Returns -1 in case of error, such as communication failure or invalid address.
 * 
 * @param address The memory address to read from.
 * @return int The 16-bit value read, or -1 in case of error.
 */
int EEPROM::read_16(const int address) {
    char command[2];
    int received;
    int ack;
    int read_ack;

    if (!check_EEPROM_mem_address(address + 1))
        return -1;
    command[0] = (address >> 8) & 0xFF; // High byte of the address
    command[1] = address & 0xFF;        // Low byte of the address
    ack = this->i2c_port.write(this->EEPROM_8BIT_ADDR, command, 2);
    read_ack = this->i2c_port.read(this->EEPROM_8BIT_ADDR, (char *)&received, 2);
    if (ack || read_ack) {
        this->errnum = this->NACK;
        #if DEBUG
            printf("[%s] ack: %d, read_ack: %d\r\n", __FUNCTION__, ack, read_ack);
        #endif
        return -1;
    }
    return received;
}

/**
 * @brief Writes an 8-bit value to the specified EEPROM address.
 * 
 * Verifies if the EEPROM is ready and the specified memory address is valid before attempting to write.
 * On successful write, it returns the number of bytes written (1 in this case). If the write operation fails, 
 * due to an invalid address or the EEPROM not being ready, it updates the error status and returns -1.
 * 
 * @param address The memory address to write to.
 * @param data The 8-bit value to write.
 * @return int Returns the number of bytes written (1) on success, or -1 in case of error.
 */
int EEPROM::write_8(const int address, char data) {
    char command[3];
    int ack;

    if (!check_EEPROM_mem_address(address))
        return -1;
    command[0] = (address >> 8) & 0xFF; // High byte of the address
    command[1] = address & 0xFF;        // Low byte of the address
    command[2] = data;
    ack = this->i2c_port.write(this->EEPROM_8BIT_ADDR, command, 3);
    if (ack) {
        this->errnum = this->NACK;
        #if DEBUG
            printf("[%s] ack: %d\r\n", __FUNCTION__, ack);
        #endif
        return -1;
    }
    return 1;
}


/**
 * @brief Writes a 16-bit integer value to a specified address in the EEPROM.
 * 
 * This method writes a 16-bit (2-byte) integer to the EEPROM, ensuring the data is stored in little-endian format, meaning the least significant byte (LSB) is stored first. 
 * Before writing, it checks if the EEPROM is ready for communication and if the specified address is valid. If the EEPROM is not ready, or if the address is out of range, the method updates the error status and returns -1 to indicate failure. Upon successful write, it resets the error status to NORMAL and returns the number of bytes written (2 in this case).
 * 
 * The function constructs a 4-byte command: the first two bytes specify the address (big-endian format),
 * and the next two bytes contain the data to be written, with the LSB followed by the most significant byte (MSB).
 * 
 * @param address The EEPROM memory address where the data will be written.
 * @param data The 16-bit data to be written to EEPROM.
 * @return int Returns 2 on successful write operation to indicate 2 bytes have been written, or -1 if an error occurs.
 */
int EEPROM::write_16(const int address, const int data) {
    char command[4];
    int ack =  0;

    if (!check_EEPROM_mem_address(address))
        return -1;

    // Prepare the command to write data: address (big-endian) + data (little-endian).
    command[0] = (address >> 8) & 0xFF; // Address high byte
    command[1] = address & 0xFF;        // Address low byte
    command[2] = data & 0xFF;           // Data low byte (LSB)
    command[3] = (data >> 8) & 0xFF;    // Data high byte (MSB)
    ack = this->i2c_port.write(this->EEPROM_8BIT_ADDR, command, 4);
    if (ack) {
        this->errnum = this->NACK;
        #if DEBUG
            printf("[%s] ack: %d\r\n", __FUNCTION__, ack);
        #endif
        return -1;
    }
    this->errnum = this->NORMAL;
    return 2;
}

/**
 * @brief Initializes the EEPROM by clearing its contents.
 * 
 * Writes zeros to all EEPROM addresses to reset its contents. Returns 0 on successful initialization,
 * or -1 in case of an error during the write operations.
 * 
 * @return int Returns 0 on success, or -1 if an error occurs.
 */
int EEPROM::init(void) {
    const int array_size = 128;
    char zero_array[array_size];

    memset(zero_array, 0, sizeof(char) * (array_size));
    int memory_address = 0x0000;
    for (int i = 0; i < 512; i++) {
        wait_ms(20);
        this->write_block_data(memory_address, zero_array, array_size);
        memory_address += array_size;
    }
    return 0;
}

/**
 * @brief Checks if the EEPROM is ready for a read or write operation.
 * 
 * This method attempts to communicate with the EEPROM to determine its readiness. It retries
 * until a successful acknowledgment is received or a timeout occurs, indicating the device cannot be accessed.
 * 
 * @return true if the EEPROM is ready for communication, false otherwise.
 */
// bool EEPROM::ready(void) {
//     char command[2];

//     command[0] = 0;
//     command[1] = 0;
//     int ack = !this->i2c_port.write(this->EEPROM_8BIT_ADDR, command, 2);
//     if (ack) {
//         this->errnum = NOT_READY_YET;
//     }
//     return ack;
// }

/**
 * @brief Verifies that the specified memory address is within the valid range of the EEPROM.
 * 
 * This method checks if the given address is within the EEPROM's addressable memory range to prevent
 * out-of-bounds read or write operations.
 * 
 * @param address The memory address to validate.
 * @return true if the address is within the valid range, false if it is out of range.
 */
bool EEPROM::check_EEPROM_mem_address(const int address) {
    const int addr_upper_bound = 0xFFFF;

    if (address > addr_upper_bound) {
        this->errnum = EEPROM::OUT_OF_RANGE;
        #if DEBUG
            printf("[%s] invalid eeprom memory access", __FUNCTION__);
        #endif
        return false;
    }
    return true;
}

/**
 * @brief Retrieves the current error number of the EEPROM object.
 * 
 * @return EEPROM_errnum The error number indicating the current state or error condition.
 */
EEPROM::EEPROM_errnum EEPROM::get_EEPROM_errnum(void) {
    return this->errnum;
}

/**
 * @brief Reads a block of data from the EEPROM.
 * 
 * Reads a specified number of bytes from a starting address into a provided buffer.
 * Verifies readiness and valid address range before the operation. Returns the number of bytes read on success,
 * or -1 in case of error.
 * 
 * @param address Starting address to read from.
 * @param received_buffer Buffer to store the read data.
 * @param size Number of bytes to read.
 * @return int Number of bytes read, or -1 in case of error.
 */
int EEPROM::read_block_data(const int address, char *received_buffer, const int size) {
    char command[2];
    int ack;
    int read_ack;

    if (size > 128 
            || size <= 0 
            || !check_EEPROM_mem_address(address + size - 1))
        return -1;
    
    command[0] = (address >> 8) & 0xFF; // High byte of the address
    command[1] = address & 0xFF;        // Low byte of the address
    ack = this->i2c_port.write(this->EEPROM_8BIT_ADDR, command, 2);
    read_ack = this->i2c_port.read(this->EEPROM_8BIT_ADDR, (char *)received_buffer, size);
    if (ack || read_ack) {
        this->errnum = this->NACK;
        return -1;
    }
    return (size);
}


/**
 * @brief Writes a block of data to the EEPROM.
 * 
 * Writes a specified number of bytes from a data array to a starting address in the EEPROM.
 * Verifies the EEPROM's readiness and valid address range before proceeding. On successful write, 
 * it returns the number of bytes written. If the write operation fails, due to an invalid address, 
 * the EEPROM not being ready, or memory allocation issues, it returns -1.
 * 
 * @param address Starting address to write to.
 * @param data_array Data array to write.
 * @param size Number of bytes to write.
 * @return int The number of bytes written on success, or -1 in case of error.
 */
int EEPROM::write_block_data(const int address, char *data_array, const int size) {
    char *command;
    int ack;

    if (size > 128 
            || size <= 0 
            || !check_EEPROM_mem_address(address + size - 1))
        return -1;
    
    // size + 2: 16-bit address + data size
    if (!(command = (char *)malloc(sizeof(char) * (size + 2)))) 
    {
        this->errnum = this->MEM_ALLOC_FAILED;
        return -1 ;
    }
    command[0] = (address >> 8) & 0xFF; // High byte of the address
    command[1] = address & 0xFF;        // Low byte of the address
    memcpy(command+2, data_array, size);
    ack = this->i2c_port.write(this->EEPROM_8BIT_ADDR, command, size + 2);
    if (ack) {
        this->errnum = this->NACK;
        free(command);
        return -1;
    }
    free(command);
    return size;
}

 /**
 * @brief Saves sensor data including pressure, roll, pitch, and yaw to EEPROM.
 * 
 * Encodes sensor values into a byte format suitable for EEPROM storage. The 32-bit pressure is split into two 16-bit parts,
 * while the floating-point roll, pitch, and yaw are scaled and converted into 16-bit integers. These are then sequentially
 * stored in the EEPROM. The function manages EEPROM memory addresses to avoid data overlap. If the data write exceeds EEPROM's
 * capacity, it signals an error.
 *
 * Data Layout in EEPROM (each '|' denotes a byte boundary):
 * 
 * | Byte 0-1      | Byte 2-3      | Byte 4-5 | Byte 6-7 | Byte 8-9 |
 * |---------------|---------------|----------|----------|----------|
 * | Pressure High | Pressure Low  | Roll     | Pitch    | Yaw      |
 * 
 * @param pressure 32-bit integer representing the pressure to be saved.
 * @param roll Floating-point representing the roll angle to be saved, scaled and stored as a 16-bit integer.
 * @param pitch Floating-point representing the pitch angle to be saved, scaled and stored as a 16-bit integer.
 * @param yaw Floating-point representing the yaw angle to be saved, scaled and stored as a 16-bit integer.
 * @return int Returns 0 on successful operation, 1 if the next write operation exceeds EEPROM's storage capacity.
 */
int EEPROM::save_to_eeprom(Serial& pc, int32_t pressure, float roll, float pitch, float yaw) {
    // static int eeprom_memory_addr = 0x0000; // Static variable to track the current write position in EEPROM.

    // Check if adding another 10 bytes would exceed the EEPROM's addressable range.
    if (this->eeprom_memory_addr + 10 >= 0x10000) {
        if (this->tictoc.read_ms()) {
            this->tictoc.stop();
            this->tictoc.reset();
            this->errnum = MEM_FULL;
        }
        return -1; // Return an error if there's not enough space left.
    }

    if (this->tictoc.read_us() < 9000) {
        return -1;
    }
    this->tictoc.reset();

    // Convert floating-point values to 16-bit integers by scaling.
    int16_t roll_u16 = static_cast<int16_t>(roll * 100);
    int16_t pitch_u16 = static_cast<int16_t>(pitch * 100);
    int16_t yaw_u16 = static_cast<int16_t>(yaw * 100);

    // Use a union to easily convert the array of 16-bit words to an array of bytes.
    char data_array[10] = {0, };
    data_array[0] = (pressure >> 24) & 0xFF;
    data_array[1] = (pressure >> 16) & 0xFF;
    data_array[2] = (pressure >> 8) & 0xFF;
    data_array[3] = pressure & 0xFF;
    data_array[4] = (roll_u16 >> 8) & 0xFF;
    data_array[5] = roll_u16 & 0xFF;
    data_array[6] = (pitch_u16 >> 8) & 0xFF;
    data_array[7] = pitch_u16 & 0xFF;
    data_array[8] = (yaw_u16 >> 8) & 0xFF;
    data_array[9] = yaw_u16 & 0xFF;

    // page write limitation
    const int PAGE_SIZE = 128;
    const int DATA_SIZE = 10;
    int write_size;

    if ((this->eeprom_memory_addr / PAGE_SIZE) != ((this->eeprom_memory_addr + DATA_SIZE - 1) / PAGE_SIZE)) {
        // detect page boundary -> need to handle.
        int first_data_size = PAGE_SIZE - (this->eeprom_memory_addr % PAGE_SIZE);
        int remain_size = DATA_SIZE - first_data_size;
        int first_ret = 0, second_ret = 0;
        first_ret = this->write_block_data(
                        this->eeprom_memory_addr,
                        data_array,
                        first_data_size);

        int boundary = (this->eeprom_memory_addr / PAGE_SIZE + 1) * PAGE_SIZE;
        if (first_ret >= 0) {
            int loop_count = 0;
            // max time of write cycle is 5ms
            while ((second_ret = this->write_block_data(
                                            boundary,
                                            data_array + first_data_size,
                                            remain_size)) < 0 
                && loop_count++ <= 50) {
                wait_us(100);
            }
        }
        if (second_ret < 0)
            write_size = first_ret;
        else
            write_size = first_ret + second_ret;
    } else {
        write_size = this->write_block_data(this->eeprom_memory_addr,data_array, 10);
    }

    // Increment the EEPROM memory address for the next write operation.
    this->eeprom_memory_addr += 10;
    return 0; // Return 0 to indicate successful operation.
}
