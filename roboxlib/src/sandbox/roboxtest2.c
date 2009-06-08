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

   int value = 0;
   while (!quit) {
    fprintf( stderr, "EncoderLeft %i", roboxGetEncoderLeft() );
    fprintf( stderr, "EncoderRight %i", roboxGetEncoderRight() );
    fprintf( stderr, "\r" );
    //roboxSetStrobo( 1 );

  
    roboxSetMotorLeft( value );
    roboxSetMotorRight( value );
    //value++;
    if ( roboxGetEmergency() ) {//|| roboxGetSupervisor() ) {
      printf( "\n" );
      break;
    }
    sleep( 1 );
  }

  roboxShutdown();

  return 0;
}

