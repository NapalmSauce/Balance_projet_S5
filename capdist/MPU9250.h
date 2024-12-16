

#include <stdint.h> 
#include <stdio.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h> //for IOCTL defs
#include <fcntl.h>

#define I2C_ADRESSE_MPU9250 0x68 // adresse du Device I2C MPU capteur

int MPU9250_lire_ID(int fileDescriptor);
