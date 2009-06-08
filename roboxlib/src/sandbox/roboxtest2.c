#include <roboxlib/robox.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>

int quit = 0;

void signaled(int signal) {
  quit = 1;
}

int main( int argc, char * argv[] ) 
{
  roboxInit();

  signal(SIGINT, signaled);

  while (!quit) {
    fprintf( stderr, "\rEncoder left %10i right %10i", 
      roboxGetEncoderLeft(), roboxGetEncoderRight() );
    usleep( 10000 );
  }
  printf( "\n" );
  
  roboxShutdown();
  
  return 0;
}

