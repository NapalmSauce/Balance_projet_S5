#include "MPU9250.h"

int MPU9250_lire_ID(int fdPortI2C)
{
	uint8_t Source = 0x75; // registre d'ID du chip I2C
	uint8_t Destination;
	if(write(fdPortI2C, &Source, 1) != 1)
	{
		fprintf(stderr,"erreur: Écriture I2C MPU9250\n");
		close(fdPortI2C);
		return -1;
	}
	if (read(fdPortI2C, &Destination, 1) != 1)
	{
		fprintf(stderr,"erreur: Lecture I2C MPU9250\n");
		close(fdPortI2C);
		return -1;
	}
	return Destination;
}
