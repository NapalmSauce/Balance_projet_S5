#include "VL6180x.h"

struct VL6180X_Message
{
	uint16_t adresse;
	uint8_t valeur;
};

struct VL6180X_Message VL6180xTuning[VL6180X_TUNING_QTY] =
{
	{.adresse=0x0207, .valeur=0x01}, {.adresse=0x0208, .valeur=0x01}, {.adresse=0x0096, .valeur=0x00}, 
	{.adresse=0x0097, .valeur=0xfd}, {.adresse=0x00e3, .valeur=0x00}, {.adresse=0x00e4, .valeur=0x04},	
	{.adresse=0x00e5, .valeur=0x02}, {.adresse=0x00e6, .valeur=0x01}, {.adresse=0x00e7, .valeur=0x03},	
	{.adresse=0x00f5, .valeur=0x02}, {.adresse=0x00d9, .valeur=0x05}, {.adresse=0x00db, .valeur=0xce},
	{.adresse=0x00dc, .valeur=0x03}, {.adresse=0x00dd, .valeur=0xf8}, {.adresse=0x009f, .valeur=0x00},	
	{.adresse=0x00a3, .valeur=0x3c}, {.adresse=0x00b7, .valeur=0x00}, {.adresse=0x00bb, .valeur=0x3c},	
	{.adresse=0x00b2, .valeur=0x09}, {.adresse=0x00ca, .valeur=0x09}, {.adresse=0x0198, .valeur=0x01},	
	{.adresse=0x01b0, .valeur=0x17}, {.adresse=0x01ad, .valeur=0x00}, {.adresse=0x00ff, .valeur=0x05},
	{.adresse=0x0100, .valeur=0x05}, {.adresse=0x0199,.valeur= 0x05}, {.adresse=0x01a6,.valeur= 0x1b},	
	{.adresse=0x01ac, .valeur=0x3e}, {.adresse=0x01a7, .valeur=0x1f}, {.adresse=0x0030, .valeur=0x00},
	{.adresse=0x0011, .valeur=0x10},// Enables polling for ‘New Sample ready’ when measurement completes
	{.adresse=0x010a, .valeur=0x30},// Set the averaging sample period (compromise between lower noise and increased execution time)
	{.adresse=0x003f, .valeur=0x46},// Sets the light and dark gain (upper nibble). Dark gain should not be changed.
	{.adresse=0x0031, .valeur=0xFF},// sets the # of range measurements after which auto calibration of system is performed 
	{.adresse=0x0040, .valeur=0x63},// Set ALS integration time to 100ms
	{.adresse=0x002e, .valeur=0x01},// perform a single temperature calibration of the ranging sensor 
	{.adresse=0x001b, .valeur=0x09},// *Set default ranging inter-measurement period to 100ms
	{.adresse=0x003e, .valeur=0x31},// *Set default ALS inter-measurement period to 500ms
	{.adresse=0x0014, .valeur=0x24},// *Configures interrupt on ‘New Sample Ready threshold event’
	{.adresse=0x0016, .valeur=0x00} //*change fresh out of set status to 0
};

int VL6180x_ecrire(int fdPortI2C, uint16_t Registre, uint8_t Donnee)
{
	uint8_t message[3];
	message[0] = (uint8_t)(Registre >> 8);
	message[1] = (uint8_t)(Registre & 0xFF);
	message[2] = Donnee;
	if (write(fdPortI2C, message, 3) != 3)
	{
		fprintf(stderr,"erreur: VL6180x_ecrire donnee\n");
		return -1;
	}
	return 0;
}

int VL6180x_lire(int fdPortI2C, uint16_t Registre, uint8_t *Donnee)
{
	uint8_t Commande[2];
	Commande[0] = (uint8_t)(Registre >> 8);
	Commande[1] = (uint8_t)Registre;
	if (write(fdPortI2C, Commande, 2) != 2)
	{
		fprintf(stderr,"erreur: VL6180x_ecrire commande\n");
		return -1;
	}
	if (read(fdPortI2C, Donnee, 1) != 1)
	{
		fprintf(stderr,"erreur: VL6180x_lire donnee\n");
		return -1;
	}
	return 0;
}

int VL6180x_lire_ID(int fdPortI2C)
{
	uint16_t registre = 0x0000;
	uint8_t Identification; // emplacement memoire pour stocker la donnee lue

	VL6180x_lire(fdPortI2C, registre, &Identification);

	return Identification;
}

int VL6180x_lireUneDistance(int fdPortI2C, float *Distance)
{
	uint8_t valeur;
	if (VL6180x_ecrire(fdPortI2C, 0x0018, 0x01) < 0)
	{
		fprintf(stderr,"erreur: VL6180x_lireUneDistance 1\n");
		return -1;
	}
	if(VL6180x_lire(fdPortI2C, 0x004F, &valeur) < 0)
	{
		fprintf(stderr,"erreur: VL6180x_lireUneDistance 2\n");
		return -1;
	}
	while((valeur & 0x7) != 4)
  	{
    	if (VL6180x_lire(fdPortI2C, 0x004F, &valeur) < 0)
    	{
    		fprintf(stderr,"erreur: VL6180x_lireUneDistance 3\n");
      		return -1;
    	}
  	}
  	if (VL6180x_lire(fdPortI2C, 0x62, &valeur) < 0)
  	{
		fprintf(stderr,"erreur: VL6180x_lireUneDistance 4\n");
    	return -1;
	}
	if (VL6180x_ecrire(fdPortI2C, 0x15, 0x07) < 0)
  	{
  	fprintf(stderr,"erreur: VL6180x_lireUneDistance 5\n");
    return -1;
	}
	*Distance = (float)valeur /10.0;
	return 0;
}

int VL6180x_initialiser(int fdPortI2C)
{
	uint8_t index;
	uint8_t valeur;
	if (VL6180x_lire(fdPortI2C, 0x16, &valeur) < 0)
	{
  		fprintf(stderr,"erreur: VL6180x_initialiser 2\n");
    	return -1;
	}
	if (valeur != 1)
	{
		fprintf(stderr,"le VL6180x va être reconfiguré\n");
	}

	for (index = 0; index < VL6180X_TUNING_QTY; index++)
	{
		if (VL6180x_ecrire(fdPortI2C, VL6180xTuning[index].adresse, VL6180xTuning[index].valeur) < 0)
		{
			fprintf(stderr,"erreur: VL6180x_initialise - index %d\n", index);
			return -1;
		}
	}
	return 0;
}
