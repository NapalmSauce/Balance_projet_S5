# 247-616_I2C-Exemples
Lecture sur le bus I2C #2 (/dev/i2c-2) à l'adresse 0x68, du registre 0x75.  Donnée attendue = 0x71 (chip ID)

La fonction IOCTL est utilisé avec l'argument I2C_SLAVE_FORCE, pour forcer le lien du port avec cette adresse même si un driver l'utilise déjà. 
