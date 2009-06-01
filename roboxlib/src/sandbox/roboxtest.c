#include <roboxlib/robox.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <fcntl.h>
#include <unistd.h>

int main( int argc, char * argv[] ) 
{
  int brakeEngaged = 1;
  roboxInit();
  for (;;) {
    int i = 0;
    for ( i = 0; i < 8; ++i ) {
      printf( "Bumper[%i]: %i ", i, roboxGetBumper( i ) );
    }
    printf( "\r" );
    brakeEngaged = ! brakeEngaged;
    roboxSetBrake( brakeEngaged );
    roboxSetStrobo( brakeEngaged );
    if ( roboxGetBumper( 4 ) ) {
      printf( "\n" );
      break;
    }
    sleep( 1 );
  }
  roboxSetStrobo( 0 );
  roboxShutdown();
  return 0;
}
