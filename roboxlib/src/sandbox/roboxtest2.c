#include <roboxlib/robox.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <fcntl.h>
#include <unistd.h>

int main( int argc, char * argv[] ) 
{
   roboxInit();
   for (;;){
    printf( "EncoderLeft %i", roboxGetEncoderLeft() );
    printf( "EncoderRight %i", roboxGetEncoderRight() );
    printf( "\r" );
    roboxSetBrake( 0 );
    roboxSetStrobo( 1 );

    if ( roboxGetEmergency() ) {
      printf( "\n" );
      break;
    }
    sleep( 1 );
  }
  roboxSetStrobo( 0 );
  roboxShutdown();

}	
}

