#include <stdio.h>
#include <sys/stat.h>
#include <assert.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include <wait.h>
#include <fcntl.h>   // File Control Definitions
#include <termios.h> // POSIX Terminal Control Definitions 
#include <unistd.h>  // UNIX Standard Definitions 
#include <errno.h>   // ERROR Number Definitions\



//coordonnees balance 107 312 119


#define _GNU_SOURCE

#define BUFSIZE 64
#define DIST_CAPTEUR 45.0F



#define GRID_STEP 3
#define SCANS_X   23
#define SCANS_Y   19
#define ZONE_X_MM (SCANS_X*GRID_STEP)
#define ZONE_Y_MM (SCANS_X*GRID_STEP)


#define HAUTEUR_BRAS 129
#define THRESH_PUCK_MM 18
//#define THRESH_PUCK HAUTEUR_BRAS-THRESH_PUCK_MM
#define THRESH_PUCK 89
#define EPAISSEUR_MIN 1

  FILE * fBras;
  int fdDist_r;
  FILE * fDist_w;
  
  
  int fdBras; 

void soulevebloc(void);
void deposebloc(void);

int move_senseur(int X, int Y, int Z);
int XYtoRota(int X, int Y);


int findInGrid(unsigned char grid[SCANS_X][SCANS_Y], int * xOut, int * yOut);
double XYtoAngle(int X, int Y);

void getcmds(int * x, int * y, int * z);
int peser(int * x, int * y, int * z);

int quadrille(int * x, int * y, int * z);
int relXYZ(int x, int y, int z);
int absXYZ(int x, int y, int z);
int absPolaire(int s, int r, int h);
int relPolaire(int s, int r, int h);
int comBras(char * commande, FILE * fBras, char buf[BUFSIZE]);
char recoisDist(FILE * fDist_w, int fdDist_r);
int cree_enfant(int * fd_rd, int * fd_wr, FILE ** f_rd, FILE ** f_wr, char * path);
FILE * configFileBras(int * fd);

int main(int argC, char * argV[])
{
  char buf[BUFSIZE] = "\0";

  int x = 150;
  int y = 0;
  int z = 120;
  
  char c = 0;
  

  cree_enfant(&fdDist_r,NULL, NULL, &fDist_w, "/home/debian/projet_s5/capdist/capteurDeDistance");
  fBras = configFileBras(&fdBras);
  
  
  struct termios PortSettings, PortSettingsOld;	// Create the structure 
  struct stat theinput;
  
  fstat(STDIN_FILENO,&theinput);
  
  if(S_ISCHR(theinput.st_mode))
  {
    
    tcgetattr(STDIN_FILENO, &PortSettings);	// Get the current attributes of the Serial port 
    PortSettingsOld = PortSettings;
    PortSettings.c_lflag &= ~(ICANON | ECHO | ECHOE);  // Non Cannonical mode, Disable echo, Disable signal
    
    if((tcsetattr(STDIN_FILENO, TCSANOW, &PortSettings)) != 0) // Set the attributes to the termios structure
      fprintf(stderr,"\n  Erreur! configuration des attributs du port serie");
  }
  
  sleep(3);
  
	tcflush(fdBras, TCIFLUSH);  // Discards old data in the rx buffer
 
  while(1)
  {
    while((c != 'm') && (c != 'o'))
    {
      c = getchar();
      if(c == 'r')
      {
        absPolaire(150,90,130);
      }
    }
    peser(&x,&y,&z);
    if(c == 'm')
    {
      depotmetal();
    }
    
    else if(c == 'o')
    {
      depotrebuts();
    }
    c = 0;
  }
  
  putc('q',fDist_w);
  fprintf(stderr,"lol\n");
  fclose(fBras);
  wait(NULL);
  
  if(S_ISCHR(theinput.st_mode))
  {
    tcsetattr(STDIN_FILENO, TCSANOW, &PortSettingsOld);
  }
}


FILE * configFileBras(int * fdout)

{

// device port série à utiliser 
  const char *portTTY = "/dev/ttyACM0";

  int fd; // File Descriptor
  
  FILE * file;
	
	fprintf(stderr,"\n Ouverture Port Serie");

	// Opening the Serial Port 
	fd = open(portTTY, O_RDWR | O_NOCTTY);  
							// O_RDWR   - Read/Write access to serial port 
							// O_NOCTTY - No terminal will control the process
							// Open in blocking mode,read will wait 
	if(fd == -1) // Error Checking
	{
		fprintf(stderr,"\n Erreur! ouverture de %s ", portTTY);
		return NULL;
	}
	fprintf(stderr,"\n Ouverture de %s reussit ", portTTY);

	// Setting the Attributes of the serial port using termios structure 
	struct termios SerialPortSettings;	// Create the structure 
	tcgetattr(fd, &SerialPortSettings);	// Get the current attributes of the Serial port 
	// Setting the Baud rate
	cfsetispeed(&SerialPortSettings, B115200); // Set Read Speed  
	cfsetospeed(&SerialPortSettings, B115200); // Set Write Speed  
	// 8N1 Mode 
	SerialPortSettings.c_cflag &= ~PARENB;   // Disables the Parity Enable bit(PARENB),So No Parity 
	SerialPortSettings.c_cflag &= ~CSTOPB;   // CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit
	SerialPortSettings.c_cflag &= ~CSIZE;	 // Clears the mask for setting the data size 
	SerialPortSettings.c_cflag |=  CS8;      // Set the data bits = 8  
	SerialPortSettings.c_cflag &= ~CRTSCTS;       // No Hardware flow Control
	SerialPortSettings.c_cflag |= CREAD | CLOCAL; // Enable receiver, Ignore Modem Control lines
	
	SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);          // Disable XON/XOFF flow control both i/p and o/p

	SerialPortSettings.c_lflag &= ~(/*ICANON |*/ ECHO | ECHOE | ISIG);  // Non Cannonical mode, Disable echo, Disable signal  

	SerialPortSettings.c_oflag &= ~OPOST;	// No Output Processing

	// Setting Time outs 
//	SerialPortSettings.c_cc[VMIN] = 3; // Read at least X character(s) 
//	SerialPortSettings.c_cc[VTIME] = 30; // Wait 3sec (0 for indefinetly) 

	if((tcsetattr(fd, TCSANOW, &SerialPortSettings)) != 0) // Set the attributes to the termios structure
	{
		fprintf(stderr,"\n  Erreur! configuration des attributs du port serie");
		close(fd);
		return NULL;
	}	
	// Read data from serial port 
	tcflush(fd, TCIFLUSH);  // Discards old data in the rx buffer
  
  file = fdopen(fd,"w");
  
  setvbuf(file, NULL, _IONBF, 0);
  
  *fdout = fd;
  
  return file;
  
}



int absPolaire(int s, int r, int h)
{
  char buf[BUFSIZE];

  size_t len = 0;
  size_t v = 0;

  fprintf(fBras,"G2201 S%.4d R%.4d H%.4d F10000\n",s,r,h);
  
  while(1)
  {
    v = len;
    len += read(fdBras, &buf[v], sizeof(buf) - len - 1);
    write(STDERR_FILENO,&buf[v],len - v);
    if(!strncmp("ok",&buf[v],2))
    {
      return 0;
    }
    if(buf[v] == 'E')
    {
      return 1;
    }
  }
}


int relPolaire(int s, int r, int h)
{
  char buf[BUFSIZE];

  size_t len = 0;

  fprintf(fBras,"G2205 S%.4d R%.4d H%.4d F3000\n",s,r,h);
  len = read(fdBras, buf, sizeof(buf) - 1);
  write(STDERR_FILENO,buf,len);
  return 0;
}

int absXYZ(int x, int y,int z)
{
  char buf[BUFSIZE];

  size_t len = 0;
  size_t v = 0;

  fprintf(fBras,"G0 X%.4d Y%.4d Z%.4d F10000\n",x,y,z);
  
  while(1)
  {
    v = len;
    len += read(fdBras, &buf[v], sizeof(buf) - len - 1);
    write(STDERR_FILENO,&buf[v],len - v);
    if(!strncmp("ok",&buf[v],2))
    {
      return 0;
    }
    if(buf[v] == 'E')
    {
      return 1;
    }
  }
}

int relXYZ(int x, int y, int z)
{
  char buf[BUFSIZE];
  
  size_t v;

  size_t len = 0;

  fprintf(fBras,"G2204 X%.3d Y%.3d Z%.3d F3000\n",x,y,z);  
  
  while(1)
  {
    v = len;
    len += read(fdBras, &buf[v], sizeof(buf) - len - 1);
    write(STDERR_FILENO,&buf[v],len - v);
    if(!strncmp("ok\r\n",&buf[v],2))
    {
      return 0;
    }
    if(buf[v] == 'E')
    {
      return 1;
    }
  }
}


int quadrille(int  * x1, int *  y1, int  * z1)
{
  unsigned char grid[SCANS_X][SCANS_Y];
  
  int ret = 0;

  int x = *x1;
  int y = *y1;
  int z = *z1;

  int xMax = x + (((SCANS_X-1) * GRID_STEP / 2));
  int yMax = y + (((SCANS_Y-1) * GRID_STEP / 2));
  int xMin = x - (((SCANS_X-1) * GRID_STEP / 2));
  int yMin = y - (((SCANS_Y-1) * GRID_STEP / 2));
  
  x = xMax;
  
  while(x >= xMin)
  {
    y = yMax;
    
    while(y >= yMin)
    {
      move_senseur(x,y,z);
      
      grid[(x-xMin)/GRID_STEP][(y-yMin)/GRID_STEP] = 
        recoisDist(fDist_w, fdDist_r);
        
      y-=GRID_STEP;
    }
    
    y = yMin;
    x -= GRID_STEP;
    
    if(x < xMin)
      break;
    
    while(y <= yMax)
    {
      move_senseur(x,y,z);
      
      grid[(x-xMin)/GRID_STEP][(y-yMin)/GRID_STEP] = 
        recoisDist(fDist_w, fdDist_r);
      
      y+=GRID_STEP;
    }
    
    x-=GRID_STEP;
  }
  
  putc('\n',stdout);
  fflush(stdout);
  
  for(x = SCANS_X-1; x >= 0; x--)
  {
    
    fprintf(stderr,"\n");
    for(y = SCANS_Y-1; y >= 0; y--)
    {
      fprintf(stderr," %.1d",((grid[x][y] < THRESH_PUCK)));
    }
  }
  
  putc('\n',stdout);
  fflush(stdout);
  
  ret = findInGrid(grid, &x, &y);
  
  if(!ret)
  {
    absXYZ(x+xMin,y+yMin,z);
    *x1 = x;
    *y1 = y;
    *z1 = z;
  }
  
  return ret;
}

/////////////////////////////////////////////////////
//
// Cree un processus enfant et fournit un
// 
// FILE * en lecture et en ecriture 
// attaches au stdout et au stdin
// du processus enfant, respectivement.
//
// Retour: pid du processus enfant. 0 si erreur.
//
/////////////////////////////////////////////////////
int cree_enfant(int * fd_rd, int * fd_wr, FILE ** f_rd, FILE ** f_wr, char * path)
{
  pid_t pid;
 
  int pipefd0[2];
  int pipefd1[2];
  
  if(pipe(pipefd0))
  {
    //error
    perror("cree_enfant: pipe()");
    return 1;
  }
  if(pipe(pipefd1))
  {
    //error
    perror("cree_enfant: pipe()");
    return 1;
  }
  
  pid = fork();
  
  if(!pid)     //Bloc execute par enfant
  {
    char * args[2] = {path, (char *)  NULL};
  
    dup2(pipefd0[0],STDIN_FILENO);  //STDIN enfant est pipe  lecture
    dup2(pipefd1[1],STDOUT_FILENO); //STDOUT enfant est pipe ecriture
    
    
    setvbuf(stdout, NULL, _IONBF, 0);
    
    close(pipefd0[1]);              //Ferme bord enfant les bord
    close(pipefd1[0]);              //pipe pour le programme parent
      
    execv(args[0],args);            //arg 0 est path
  }
  else if(pid < 0)  //Erreur lors de fork()
  {
    //erreur
    perror("cree_enfant: fork()");
    return 0;
  }
  
  if(fd_rd)  //Si demande
  {
    *fd_rd = pipefd1[0];  //store fd en lecture dans le pointeur
  }
  if(fd_wr)  //Si demande
  {
    *fd_wr = pipefd0[1];  //store fd en lecture dans le pointeur
  }
  
  if(f_wr)  //Si demande
  {
    *f_wr = fdopen(pipefd0[1], "w"); //Cree un FILE * pour le pipe en write
    if (!*f_wr)
    {
      perror("cree_enfant: fdopen() pour pipe write");
      return 1;
    }
    setvbuf(*f_wr, NULL, _IONBF, 0);
  }  
  if(f_rd)  //Si demande
  {
    *f_rd = fdopen(pipefd1[0], "r"); //Cree un FILE * pour le pipe en read
    if (!*f_rd)
    {
      perror("cree_enfant: fdopen() pour pipe read");
      return 1;
    }
    setvbuf(*f_rd, NULL, 0, 0);
  }
  fcntl(pipefd1[0], F_SETFL, O_NONBLOCK); 
  
  close(pipefd0[0]);      //Ferme cote parent les bord pipe
  close(pipefd1[1]);      //qu'utilise le programme enfant
  
  return pid;
}




int comBras(char * commande, FILE * fBras, char buf2[BUFSIZE])
{
  char buf[BUFSIZE];
  int ret = 0;
  size_t len = 0;
  size_t v = 0;
  
  fprintf(fBras,commande);
  
  while(1)
  {
    v = len;
    len += read(fdBras, &buf[v], sizeof(buf) - len - 1);
    write(STDERR_FILENO,&buf[v],len - v);
    if(!strncmp("ok",&buf[v],2))
    {
      break;
    }
    if(buf[v] == 'E')
    {
      ret = 1;
      break;
    }
  }
  if(buf2)
  {
    memcpy(buf2,buf,len);
    buf2[len] = '\0';
  }
  
  return ret;
}

char recoisDist(FILE * fDist_w, int fdDist_r)
{
  char c[8];
  unsigned char x = 0xff;
  size_t len;
  while(1)
  {
    putc('l',fDist_w);
    
    usleep(1000);
    
    len = read(fdDist_r, c, sizeof(c) - 1);
    if(len > 2)
    {
      break;
    }
  }
  c[len] = 0;
  x = ((c[1] > 0x39)? (c[1] - 0x61 + 10) : (c[1] & 0x0F)) |
      ((((c[0] > 0x39)? (c[0] - 0x61 + 10) : (c[0] & 0x0F)) << 4));
  return x;
}

int move_senseur(int X, int Y, int Z)
{
  double off_x, off_y;
  int coord_x,coord_y;
  
  double angle = atan2(Y, X);
  
  off_x = DIST_CAPTEUR * cos(angle);
  
  off_y = DIST_CAPTEUR * sin(angle);
  
  coord_x = (int)((float)X - off_x);
  coord_y = (int)((float)Y - off_y);
  
 // printf("coord_x %d, coord_y %d\n",coord_x,coord_y);
  absXYZ(coord_x,coord_y,Z);
  return 0;
}

//Retourne pour des coordonnees (x,y) l'angle
//en valeur de rotation utilisable en 
//coordonnees polaires pour le bras 
int XYtoRota(int X, int Y)
{
    return (int)((M_PI_2 + atan2(Y,X)) / M_PI * 180);
}

int findInGrid(unsigned char grid[SCANS_X][SCANS_Y], int * xOut, int * yOut)
{
  
  int x, y;
  
  int xLen = 0;
  int yLen = 0;
  
  
  int start;
  
  
  struct Seg
  {
    int start;
    int stop;
  } segs[SCANS_Y];
  
  
  struct 
  {
    int xstart;
    int xstop;
    int ystart;
    int ystop;
  } carre =
  
  {
    INT_MAX,
    INT_MIN,
    INT_MAX,
    INT_MIN
  };
  
  //Collecte tous les segments pour chaque colonne
  for( y = 0; y < SCANS_Y; y++)
  {
    
    segs[y].start = -1;
    segs[y].stop = -1;
    
    for( x = 0; x < SCANS_X; x++)
    {
      if(grid[x][y] < THRESH_PUCK)
      {
        xLen++;
        
        if((xLen > 1))
        {
          if(segs[y].start < 0)
          {
            segs[y].start = x - 1;
          }
          segs[y].stop = x;
        }
      }
      else
      {
        xLen = 0;
      }
    }
  }
  
  yLen = 0;
  
  //trouve le rectangle le moins restrictif dans la matrice
  for( y = 0; y < SCANS_Y; y++)
  {
    if (segs[y].start >= 0) //Verifie s'il y a quelque chose
    {
      yLen++;
      if(yLen > EPAISSEUR_MIN)
      {
        //Note le debut du carre une seule fois.
        carre.ystart =
        (carre.ystart != INT_MAX)? carre.ystart : (y - EPAISSEUR_MIN);
        
        //Avons-nous un nouveau debut de carre?
        carre.xstart =
          (segs[y].start < carre.xstart)? segs[y].start : carre.xstart;
        
        //Avons-nous une nouvelle fin de carre?
        carre.xstop =
          (segs[y].stop > carre.xstop)? segs[y].stop : carre.xstop;
        
        //Note la 'nouvelle' fin du carre chaque fois.
        carre.ystop = y;
      }
    }
    else //Si on est pas dans un 'carre', reset epaisseur.
    {
      yLen = 0;
    }
  }
  
//  fprintf(stderr,"\nxstart %d, xstop %d\n",carre.xstart, carre.xstop);
  
//  fprintf(stderr,"\nystart %d, ystop %d\n",carre.ystart, carre.ystop);
  if
  ( 
    (carre.xstart == INT_MAX) ||
    (carre.xstop == INT_MIN) ||
    (carre.ystart == INT_MAX) ||
    (carre.ystop == INT_MIN)
  )
  {
//    fprintf(stderr,"Bloc non trouve!\n");
    return 1;
  }
  
  
  yLen = (carre.ystop - carre.ystart) + 1;
  xLen = (carre.xstop - carre.xstart) + 1;
  
//  fprintf(stderr,"xLen %d, yLen %d\n",xLen, yLen);
  
  double xhalf, yhalf;
  
  if(xLen & 0x01)
  {
    xhalf = ((((double)xLen - 1) / 2 ) * (double)GRID_STEP);
  }
  else
  {
    xhalf = ((double)xLen / 2) * (double)GRID_STEP + ((double)GRID_STEP / 2);
  }
  
  if(yLen & 0x01)
  {
    yhalf = ((((double)yLen - 1) / 2) * (double)GRID_STEP);
  }
  else
  {
    yhalf = ((double)yLen / 2) * (double)GRID_STEP + ((double)GRID_STEP / 2);
  }
  
  *xOut = carre.xstart * GRID_STEP + (int) xhalf;
  *yOut = carre.ystart * GRID_STEP + (int) yhalf;
  
  fprintf(stderr,"x %d, y %d\n", *xOut, *yOut);
  
  return 0;
}


int peser(int * x1, int * y1, int * z1)
{
  int x = *x1;
  int z = 120;
  int y = *y1;
  
  char c = 0;
  
  char buf[BUFSIZE];
  x = 88;
  y = -293;;
  absXYZ(x,y,z);
  
  if(quadrille(&x,&y,&z))
  {
    return 1;
  }
  
  soulevebloc();
  
  comBras("G0 Z130\n",fBras,buf);
  
  absPolaire(150,90,130);
  
  x = 107;
  y = 312;
  z = 130;
  
  absXYZ(x,y,z);
  
  
  deposebloc();
  
  z = 130;
  
  absXYZ(x,y,z);
  
  putchar('p');
  
  while(c != 'g')
  {
    c = getchar();
    if(c == 'q')
    {
      return 1;
    }
  }
  
  buf[0] = 0;
  
  soulevebloc();
  comBras("G2204 Z130\n",fBras,buf);
  
  absXYZ(x,y,z);
  *x1 = x;
  *y1 = y;
  *z1 = z;
  
}


void getcmds(int * x1, int * y1, int * z1)
{
  int x = *x1;
  int z = *z1;
  int y = *y1;
  char c = 0;
  while(c != 'q')
  {
    c = getchar();
    
    switch(c)
    {
    case 'a':
      y--;
      absXYZ(x,y,z);
      break;
    case 's':
      x--;
      absXYZ(x,y,z);
      break;
    case 'd':
      y++;
      absXYZ(x,y,z);
      break;
    case 'w':
      x++;
      absXYZ(x,y,z);
      break;
    case 'l':
      fprintf(stderr,"d = %u",recoisDist(fDist_w, fdDist_r));
      break;
    case 'c':
      fprintf(stderr,"x %d, y %d, z %d\n",x,y,z);
      break;
    case 'm':
      z--;
      absXYZ(x,y,z);
      break;
    case 'n':
      z++;
      absXYZ(x,y,z);
      break;
    case 'o':
      comBras("M2231 V0\n",fBras, NULL);
      absXYZ(x,y,z);
      break;
    case 'p':
      comBras("M2231 V1\n",fBras, NULL);
      absXYZ(x,y,z);
      break;
      case 'h':
      quadrille(&x,&y,&z);
      break;
    case 'r':
      x = 150;
      y = 0;
      z = 80;
      absXYZ(x,y,z);
      break;
      
    }
    
  }
  *x1 = x;
  *y1 = y;
  *z1 = z;
  
}


void soulevebloc(void)
{
  char buf[BUFSIZE] = {0};
    while(!strstr(buf,"@6 N0 V1"))
  {
    comBras("G2204 Z-1\n",fBras,buf);
  }
  comBras("M2231 V1\n",fBras, NULL);
}


void deposebloc(void)
{
  char buf[BUFSIZE] = {0};
  while(!strstr(buf,"@6 N0 V1"))
  {
    comBras("G2204 Z-1\n",fBras,buf);
  }
  comBras("G2204 Z1\n",fBras,buf);
  
  comBras("M2231 V0\n",fBras, NULL);
}


void depotmetal(void)
{
  absXYZ(183,62,130);
  deposebloc();
  comBras("G2204 Z130\n",fBras,NULL);
}

void depotrebuts(void)
{
  
  absXYZ(184,-43,130);
  deposebloc();
  comBras("G2204 Z130\n",fBras,NULL);
}
