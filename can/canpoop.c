#define _DEFAULT_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <wait.h>
#include <fcntl.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <string.h>

int processus_fils(void);
int processus_pere(void);

  int pipefd[2];
	int fdSocketCAN, i; 
	struct sockaddr_can addr;
	struct ifreq ifr;
  
int main(int argc, char *argv[]) //char **argv)
{
  pid_t pid;
  
  
  if(pipe(pipefd))
  {
    //error
    fprintf(stderr,"Erreur pipe\n");
    return 1;
  }
  
  // Setting the Attributes of the serial port using termios structure 
  struct termios PortSettings, PortSettingsOld;	// Create the structure 
  struct stat theinput;
  
  fstat(STDIN_FILENO,&theinput);
  
  if(S_ISCHR(theinput.st_mode))
  {
    
    tcgetattr(STDIN_FILENO, &PortSettings);	// Get the current attributes of the Serial port 
    PortSettingsOld = PortSettings;
//    PortSettings.c_lflag &= ~(ICANON | ECHO | ECHOE);  // Non Cannonical mode, Disable echo, Disable signal
    
//    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK); 
//    if((tcsetattr(STDIN_FILENO, TCSANOW, &PortSettings)) != 0) // Set the attributes to the termios structure
//      fprintf(stderr,"\n  Erreur! configuration des attributs du port serie");
  }

	/*
	La première étape est de créer un socket. 
	Cette fonction accepte trois paramètres : 
		domaine/famille de protocoles (PF_CAN), 
		type de socket (raw ou datagram) et 
		protocole de socket. 
	la fonction retourne un descripteur de fichier.
	*/
	if ((fdSocketCAN = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("Socket");
		return -1;
	}
	
	/*
	Ensuite, récupérer l'index de l'interface pour le nom de l'interface (can0, can1, vcan0, etc.) 
	que nous souhaitons utiliser. Envoyer un appel de contrôle d'entrée/sortie et 
	passer une structure ifreq contenant le nom de l'interface 
	*/
	if(argc == 2)
		strcpy(ifr.ifr_name, argv[1]);
	else strcpy(ifr.ifr_name, "can0" );

	ioctl(fdSocketCAN, SIOCGIFINDEX, &ifr);
	/* 	Alternativement, zéro comme index d'interface, permet de récupérer les paquets de toutes les interfaces CAN.
	Avec l'index de l'interface, maintenant lier le socket à l'interface CAN
	*/

	/*
	
	*/
	memset(&addr, 0, sizeof(addr));
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	if (bind(fdSocketCAN, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("Bind");
		return -1;
	}
  if(fcntl(fdSocketCAN,F_SETFL, O_NONBLOCK) == -1)
  {
    fprintf(stderr,"Erreur fcntl\n");
    abort();
  }
  
  pid = fork();
  
  if(!pid)
  {
    processus_fils();
    return 0;
  }
  else if(pid > 0)
  {
    processus_pere();
    wait(NULL);
  }
  else
  {
    fprintf(stderr,"Erreur de creation du processus fils");
  }
  
  if(S_ISCHR(theinput.st_mode))
  {
    tcsetattr(STDIN_FILENO, TCSANOW, &PortSettingsOld);
  }
  
  if (close(fdSocketCAN) < 0) {
    perror("Close");
    return -1;
  } 
    
	return 0;
}



int processus_pere(void)
{
	struct can_frame frame_out;
  char mystr[11];
  unsigned char len;
  
  close(pipefd[0]);
  
  while(1)
  {
    /*
    Envoyer une trame CAN, initialiser une structure can_frame et la remplir avec des données. 
    La structure can_frame de base est définie dans include/linux/can.h  
    */
  //   frame.can_dlc = 7;		// nombre d'octets de données
    
    fgets(mystr, 10, stdin);
    
    switch(mystr[0])
    {
    case 'j':
      frame_out.can_id = 0x161;  	// identifiant CAN, exemple: 247 = 0x0F7
      break;
    case 'c':
      frame_out.can_id = 0x151;  	// identifiant CAN, exemple: 247 = 0x0F7
      break;
    case 'u':
      frame_out.can_id = 0x123;  	// identifiant CAN, exemple: 247 = 0x0F7
      break;
    case 'q':
      write(pipefd[1],"q",1);
      return 0;
      
    case 'G':
      frame_out.can_id = 0x153;  	// identifiant CAN, exemple: 247 = 0x0F7
      break;
    
        
        
      break;
    default:
        
      frame_out.can_id = 0x007;  	// identifiant CAN, exemple: 247 = 0x0F7
    
      
    }
    len = strlen(mystr) - 2;
    if(!len)
    {
      continue;
    }
    else if(len > 8)
    {
      len = 8;
    }
    frame_out.can_dlc = len;
    
    memcpy(frame_out.data, &mystr[1], len);

    if(frame_out.can_dlc != 0)
    {
      if (write(fdSocketCAN, &frame_out, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        perror("Write");
        return -1;
      }
    }
  }
  
}

int processus_fils(void)
{
	int nbytes;

  char char_pere;
  struct can_filter rfilter[2] =
  {
    { .can_id = 0x550, .can_mask = 0xFF0},
    { .can_id = 0x480, .can_mask = 0xFF0}
  };
	struct can_frame frame_in;
  
  if(fcntl(pipefd[0],F_SETFL, O_NONBLOCK) == -1)
  {
    fprintf(stderr,"Erreur fcntl\n");
    abort();
  }
  
  close(pipefd[1]);
  
  while(1)
  {
    // appel système read(). Cela bloquera jusqu'à ce qu'une trame soit disponible
    nbytes = read(fdSocketCAN, &frame_in, sizeof(struct can_frame));
    if(read(pipefd[0],&char_pere,1))
    {
      if(char_pere == 'q')
      {
        return 0;
      }
    }

    if (nbytes <= 0) {
      continue;
    }


      fprintf(stderr,"0x%03X [%d] ",frame_in.can_id, frame_in.can_dlc);
      for (i = 0; i < frame_in.can_dlc; i++)
        fprintf(stderr,"%02X ",frame_in.data[i]);
      fprintf(stderr,"\"");
      for (i = 0; i < frame_in.can_dlc; i++)
      {
        if(isprint(frame_in.data[i]))
        {
          fprintf(stderr,"%c",frame_in.data[i]);
        }
        else
        {
          fprintf(stderr,".");
        }
      }
      fprintf(stderr,"\"\n");
 //     write(STDERR_FILENO,'\n',stderr);
    
    //Commandant unite poids
    if(frame_in.can_id == 0x123)
    {
      if(!strncmp((char*)frame_in.data, "Onces", 5))
      {
        putchar('2');
        
      }
      else if(!strncmp((char*)frame_in.data, "Grammes", 7))
      {
        putchar('1');
      }
    }
    
    //Commandant marche/arret
    else if(frame_in.can_id == 0x121)
    {
      
      if(!strncmp((char*)frame_in.data, "Start", 5))
      {
        write(STDOUT_FILENO,"g",1);
      }
      else if(!strncmp((char*)frame_in.data, "Arret", 5))
      {
        write(STDOUT_FILENO,"a",1);
      }
    }
    
    //Usine couleur
    else if(frame_in.can_id == 0x151)
    {
      if(frame_in.data[0] == 'O')
      {
        write(STDOUT_FILENO,"o",1);
      }
      else if(frame_in.data[0] == 'M')
      {
        write(STDOUT_FILENO,"m",1);
      }
      
    }
    
  }
}
