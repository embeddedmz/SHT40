// on i.MX6 compile with : gcc sht40.c -o sht40 -Wall
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#define SHT40_I2C_BUS                "/dev/i2c-1"
#define SHT40_SLAVE_ADDR             0x44
#define SHT40_HIGH_PRECISION_MEASURE 0xFD

uint8_t checkCrc(uint8_t* data, uint16_t offset, uint16_t count, uint8_t checksum)
{
    uint16_t current_byte;
    uint8_t crc_bit;
    uint8_t crc = 0xFF;

    /* calculates 8-Bit checksum with given polynomial */
    for (current_byte = 0; current_byte < count; ++current_byte)
    {
        crc ^= data[offset + current_byte];
        for (crc_bit = 8; crc_bit > 0; --crc_bit)
        {
            if (crc & 0x80)
            {
                crc = (crc << 1) ^ 0x31;
            }
            else
            {
                crc = (crc << 1);
            }
        }
    }
    
    return crc == checksum;
}

int main(int argc, char * argv[])
{
    // Obtenir l’accès au bus i2c
    int fd = open(SHT40_I2C_BUS, O_RDWR);
    if (fd < 0)
    {
        perror(SHT40_I2C_BUS);
        exit(EXIT_FAILURE);
    }

    // Fixer l'adresse de l'esclave avec qui communiquer
    if (ioctl(fd, I2C_SLAVE, SHT40_SLAVE_ADDR) < 0) {
        perror("Slave unreachable");
        exit(EXIT_FAILURE);
    }
    
    // les routines smbus ne permettent pas de communiquer correctement avec ce composant
    // il faut passer par les appels système read/write
    
    uint8_t cmd = SHT40_HIGH_PRECISION_MEASURE;
    if (write(fd, &cmd, 1) != 1) {
        perror("SHT40_HIGH_PRECISION_MEASURE write error");
        exit(EXIT_FAILURE);
    }
    
    usleep(10000); // 10 ms
    
    uint8_t resp[6] = { 0 };
    if (read(fd, resp, 6) != 6) {
        perror("SHT40_HIGH_PRECISION_MEASURE read error");
        exit(EXIT_FAILURE);
    }
    
    close(fd);
    
    /*printf("SHT40 replied with:");
    
    int byteIndex;
    for (byteIndex = 0; byteIndex < 6; ++byteIndex)
    {
        printf(" 0x%x", resp[byteIndex]);
        if (byteIndex < 31)
        {
            printf(" -");
        }
    }
    printf("\n");*/
    
    if (checkCrc(resp, 0, 2, resp[2]))
    {
        uint16_t tempTicks = (resp[0] << 8) + resp[1];
        double temperature = -45 + 175 * tempTicks / 65535.0;
        printf("Temperature = %0.2f °C\n", temperature);
    }
    else
    {
        puts("ERROR - Bad CRC for temperature!");
    }

    if (checkCrc(resp,3, 2, resp[5]))
    {
        uint16_t rhTicks = (resp[3] << 8) + resp[4];
        double rh = -6 + 125 * rhTicks / 65535.0;
        if (rh > 100)
        {
            rh = 100;
        }
        if (rh < 0)
        {
            rh = 0;
        }
        printf("Relative Humidity = %0.2f %%RH\n", rh);
    }
    else
    {
        puts("ERROR - Bad CRC for relative humidity!");
    }
    
    return EXIT_SUCCESS;
}
