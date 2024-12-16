#include <unistd.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/stat.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h> //for IOCTL defs
#include <fcntl.h>

#include "VL6180x.h"
#include "MPU9250.h"

#define I2C_FICHIER_BUS_EXTERNE "/dev/i2c-1" // fichier Linux representant le BUS #1
#define I2C_FICHIER_BUS_INTERNE "/dev/i2c-2" // fichier Linux representant le BUS #2

int open_I2C_bus(char*, int);

int main()
{
  
  
	// Setting the Attributes of the serial port using termios structure 
	struct termios PortSettings, PortSettingsOld;	// Create the structure 
  struct stat theinput;
  
  fstat(STDIN_FILENO,&theinput);
  
  if(S_ISCHR(theinput.st_mode))
  {
    
    tcgetattr(STDIN_FILENO, &PortSettings);	// Get the current attributes of the Serial port 
    PortSettingsOld = PortSettings;
    PortSettings.c_lflag &= ~(ICANON | ECHO | ECHOE);  // Non Cannonical mode, Disable echo, Disable signal 
            PortSettings.c_cc[VMIN] = 0; // Read at least X character(s) 
        PortSettings.c_cc[VTIME] = 0; // Wait 3sec (0 for indefinetly)
    if((tcsetattr(STDIN_FILENO, TCSANOW, &PortSettings)) != 0) // Set the attributes to the termios structure
      fprintf(stderr,"\n  Erreur! configuration des attributs du port serie");
  }
  
	/// ouverture du port de communication pour le capteur MPU9250, bus I2C interne
	int fdPortI2C_Interne = open_I2C_bus(I2C_FICHIER_BUS_INTERNE, I2C_ADRESSE_MPU9250);
	if(fdPortI2C_Interne < 0)
	{
		return -1;
	}
	/// ouverture du port de communication pour le capteur VL6180x, bus I2C externe
	int fdPortI2C_Externe = open_I2C_bus(I2C_FICHIER_BUS_EXTERNE, I2C_ADRESSE_VL6180x);
	if(fdPortI2C_Externe < 0)
	{
		return -1;
	}

//	fprintf(stderr,"ID MPU9250 = %#04X\n", MPU9250_lire_ID(fdPortI2C_Interne)); // no error mngt
//	fprintf(stderr,"ID VL6180x = %#04X\n", VL6180x_lire_ID(fdPortI2C_Externe)); // no error mngt

	if(VL6180x_initialiser(fdPortI2C_Externe) < 0)
	{
		fprintf(stderr,"erreur VL6180x Initialiser\n");
		return -1;
	}

	float distance = 0.0;
	while(1)
	{
    char c;
    if(read(STDIN_FILENO, &c, sizeof(c)))
    {
      if((c == 'L') || (c == 'l'))
      {
        if(VL6180x_lireUneDistance(fdPortI2C_Externe, &distance) < 0 )
        {
          fprintf(stderr,"erreur VL6180x Lire distance\n");
          return -1;
        }
        else
        {
	  char s[4];
//          fprintf(stderr,"d=%2.5f\n", distance);
          snprintf(s,4,"%.2x\n",(unsigned char) (distance * 10));
          write(STDOUT_FILENO,s,3);
        }
      }
      if((c == 'q') || (c == 'Q'))
      {
        break; 
      }
    }
	}

	close(fdPortI2C_Interne);
	close(fdPortI2C_Externe);
  
  
  if(S_ISCHR(theinput.st_mode))
  {
    tcsetattr(STDIN_FILENO, TCSANOW, &PortSettingsOld);
  }
  
	return 0;
}

int open_I2C_bus(char* port, int adresse_I2C)
{
	int fdPortI2C;  // file descriptor I2C

	// Initialisation du port I2C, 
	fdPortI2C = open(port, O_RDWR); // ouverture du 'fichier', création d'un 'file descriptor' vers le port I2C
	if(fdPortI2C == -1)
	{
		fprintf(stderr,"erreur: ouverture du port I2C\n");
		return -1;
	}
	if(ioctl(fdPortI2C, I2C_SLAVE_FORCE, adresse_I2C) < 0)  // Liaison de l'adresse I2C au fichier (file descriptor) du bus I2C
	{ 	// I2C_SLAVE_FORCE if it is already in use by a driver (i2cdetect : UU)
		fprintf(stderr,"erreur: adresse du device I2C\n");
		close(fdPortI2C);
		return -1;
	}
	return fdPortI2C;
}
