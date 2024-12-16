//Description: interface qui permet de lire des distances à l'aide d'un VL6180x
//broches: 
//      vue de dessus
//       _____________
//      |             |
//      |             |
//      |   _  _  _   |
//      |  |_||_||_|  |
//      |             |
//      |scl sda  gnd |
//      |int gpio vdd |
//      |_____________|
//
//Note:
//Des pull-up de 4.7k sont requises sur scl et sda si celles du BbB n'y sont pas
//Une pull-up de 10k est requise pour gpio pour permettre au VL6180x de bien démarré
//Le vl6180x ne peut être utilisé pendant la première milliseconde qui suit sont alimentation.
//Il faut donc attendre ce temps avant de communiquer avec lui.

#include <stdint.h> 
#include <stdio.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h> //for IOCTL defs
#include <fcntl.h>

#define I2C_ADRESSE_VL6180x 0x29 // adresse du Device I2C VL6180X capteur
#define VL6180X_TUNING_QTY	40

int VL6180x_lire_ID(int fileDescriptor);
int VL6180x_initialiser(int fileDescriptor);
int VL6180x_lireUneDistance(int fileDescriptor, float* Distance);
