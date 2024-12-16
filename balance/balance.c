
#define _DEFAULT_SOURCE 
#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <termios.h>
#include <net/if.h>

#define PILOTESERIEUSB_TTY "/dev/ttyUSB0"

// Variables globales pour la communication série et CAN
int serial_fd, can_socket;
int gram_mode = 1;  // 1 pour grammes, 0 pour onces
int operation_mode = 1; // 0 pour arrêt, 1 pour opération

typedef enum
{
  Commandant = 0x120,
    com_mode,
    com_alarm,
    com_conversion, 
  
  Station_Pese = 0x140,
    bal_mode,
    bal_poids,
  
  Centre_tri = 0x150,
    ct_couleur,
    ct_mode,
  
  Gestion_transport = 0x160,
    gt_position,
   gt_statut,
	
}CAN_ID;

// Fonction d'initialisation de la balance
int init_balance() {
    struct termios options;
    serial_fd = open(PILOTESERIEUSB_TTY, O_RDWR | O_NOCTTY);
    if (serial_fd == -1) {
        perror("Erreur d'ouverture de la balance");
        return -1;
    }
    if(fcntl(serial_fd,F_SETFL, O_NONBLOCK) == -1)
    {
      perror("fcntl");
      abort();
    }
    tcgetattr(serial_fd, &options);
    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);
    options.c_cflag |= (CLOCAL | CREAD);
    tcsetattr(serial_fd, TCSANOW, &options);
    return 0;
}

// Fonction pour lire le poids de la balance
float lire_poids_balance() {
    char buffer[32];
    char filtered_buffer[32];
    int j = 0;

    int bytes_read = read(serial_fd, buffer, sizeof(buffer) - 1);
    if (bytes_read > 0) {
        buffer[bytes_read] = '\0';

        for (int i = 0; i < bytes_read; i++) {
            if ((buffer[i] >= '0' && buffer[i] <= '9') || buffer[i] == '.') {
                filtered_buffer[j++] = buffer[i];
            }
        }
        filtered_buffer[j] = '\0';

        return atof(filtered_buffer);
    }
    return -1;
}

// Processus principal

int main() {
  
  float dernier_poids = 0.0;
  char buf;

  // Setting the Attributes of the serial port using termios structure 
  struct termios PortSettings, PortSettingsOld;	// Create the structure 
  struct stat theinput;
  
  fstat(STDIN_FILENO,&theinput);
  
  if(S_ISCHR(theinput.st_mode))
  {
    
    tcgetattr(STDIN_FILENO, &PortSettings);	// Get the current attributes of the Serial port 
    PortSettingsOld = PortSettings;
    PortSettings.c_lflag &= ~(ICANON | ECHO | ECHOE);  // Non Cannonical mode, Disable echo, Disable signal
    
    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK); 
    if((tcsetattr(STDIN_FILENO, TCSANOW, &PortSettings)) != 0) // Set the attributes to the termios structure
      fprintf(stderr,"\n  Erreur! configuration des attributs du port serie");
  }
     
  if(S_ISFIFO(theinput.st_mode))
  {
    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK); 
  }
  
  init_balance();

  while (1) {
    float poids = lire_poids_balance();
    if (poids > 0.0F) {
      if (!gram_mode) {
        poids *= 0.035274; // Conversion en onces
      }
      dernier_poids = poids;
    }
    
    if((read(STDIN_FILENO,&buf,1) == 1))
    {
      if(buf == 'l')
      {
        printf("%.2f\n", dernier_poids);
      }
      if(buf == 'g')
      {
        if(!gram_mode) {
          dernier_poids /= 0.035274; // Conversion en grammes
        }
        gram_mode = 1;
      }
      else if(buf == 'z')
      {
        if (gram_mode) {
          dernier_poids *= 0.035274; // Conversion en onces
        }
        gram_mode = 0;
      }
      if(buf == 'q')
      {
        break;
      }
    }
    usleep(500);
  }
  if(S_ISCHR(theinput.st_mode))
  {
    tcsetattr(STDIN_FILENO, TCSANOW, &PortSettingsOld);
  }
}


