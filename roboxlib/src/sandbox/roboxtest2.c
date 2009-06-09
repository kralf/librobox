#include <roboxlib/robox.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <math.h>

int quit = 0;

void signaled(int signal) {
  quit = 1;
}

#define enc_to_rad(v) v/(4.0*500.0*50.0)*2.0*M_PI

int main( int argc, char * argv[] ) 
{
  roboxInit();

  signal(SIGINT, signaled);

  while (!quit) {
    fprintf( stderr, "\rEncoder left %8.2f right %8.2f", 
      enc_to_rad(roboxGetEncoderLeft())*180.0/M_PI, 
      enc_to_rad(roboxGetEncoderRight())*180.0/M_PI );
    usleep( 10000 );
  }
  printf( "\n" );
  
  roboxShutdown();
  
  return 0;
}

