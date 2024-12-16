#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <string.h>

#define BUFSIZE 64

int cree_enfant(int * fd_rd, int * fd_wr, FILE ** f_rd, FILE ** f_wr, char * path);

int can_socket;

int main(int argC, char argV[])
{
  size_t len;
  
  //FILE * sortant des programmes enfant
  FILE * fBras_w;
  FILE * fBalance_w;
  FILE * fCAN_w;
  
  //fd entrant des programmes enfant
  int fdBras_r;
  int fdBalance_r;
  int fdCAN_r;
  
  char buf[BUFSIZE];
  
  size_t a = 0;
  size_t b = 0;
  
  enum {
    STOP,
    PESE,
    LIS,
    DISPOSE
  } stade = STOP;
  
  char cmd_can = 0;
  
  char char_bras = 0;
  
  char buf_poids[16];
  
  char sens;
  
  char mode = 'o';
  char bras_etat = 'r';
  
  char bloc = 'm';
  
  //Ajuster les path au besoin
  cree_enfant(&fdBras_r,NULL,NULL,&fBras_w,"/home/debian/projet_s5/bras/bras");
  cree_enfant(&fdBalance_r,NULL,NULL,&fBalance_w,"/home/debian/projet_s5/balance/balance");
  cree_enfant(&fdCAN_r,NULL,NULL,&fCAN_w,"/home/debian/projet_s5/can/can_messenger");
  
  
  
//  fprintf(fCAN_w, "mAttente\n");
  
  while(1)
  {
    if(read(fdBras_r, &char_bras,1) > 0)
    {
      char_bras = 0;
    }
    if(read(fdCAN_r, &cmd_can, 1) > 0)
    {
      if(cmd_can == 'g')
      {
        mode = 'o';
        cmd_can = 0;
        fprintf(fCAN_w,"mOpe\n");
      }
      else if(cmd_can == 'a')
      {
        mode = 'a';
        cmd_can = 0;
        fprintf(fCAN_w,"mStop\n");
      }
    }
    if(mode == 'o')
    {
      switch(cmd_can)
      {
      case '1':
        putc('g',fBalance_w);
        cmd_can = 0;
//        fprintf(stderr,"en g\n");
        break;
      case '2':
        putc('z',fBalance_w);
        cmd_can = 0;
//        fprintf(stderr,"en oz");
        break;
      case 'o':
        bloc = 'o';
        cmd_can = 0;
//        fprintf(stderr,"orange");
        break;
      case 'm':
        bloc = 'm';
        cmd_can = 0;
//        fprintf(stderr,"metalique");
        break;
      }                 //switch(cmd_can)
      
      
      switch(stade)
      {
      case STOP:
        
        if(cmd_can == '!')
        {
          putc(bloc,fBras_w);
          stade = PESE;
          cmd_can = 0;
//          fprintf(stderr,"on pese");
        }
        
        break;
      
      case PESE:
      
        if(char_bras == 'p')
        {
          putc('l',fBalance_w);
          char_bras = 0;
          stade = LIS;
//          fprintf(stderr,"on lit");
        }
        
        break;
        
      case LIS:
      
        len = read(fdBalance_r,buf_poids,sizeof(buf_poids)-1);
        if(len > 0)
        {
          stade = DISPOSE;
          buf_poids[len] = 0;
          fprintf(fCAN_w,"p%s",buf_poids);
//          fprintf(stderr,"on dispose");
        }
      
      case DISPOSE:
        
        putc('r',fBras_w);
        stade = STOP;
//        fprintf(stderr,"on stop!");
        break;
      }
    }
  }
  wait(NULL);
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




