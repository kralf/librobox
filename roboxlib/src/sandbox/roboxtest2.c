#include <roboxlib/robox.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <fcntl.h>
#include <unistd.h>

int main( int argc, char * argv[] ) 
{
   roboxInit();
   int value = 0;
   for (;;){
    fprintf( stderr, "EncoderLeft %i", roboxGetEncoderLeft() );
    fprintf( stderr, "EncoderRight %i", roboxGetEncoderRight() );
    fprintf( stderr, "\r" );
    roboxSetBrake( 0 );
    roboxSetMotorEnable( 1 );
    roboxSetStrobo( 1 );
    roboxSetPower( 1 );
   

    roboxSetMotorLeft( value );
    roboxSetMotorRight( value );
    //value++;
    if ( roboxGetEmergency() ) {//|| roboxGetSupervisor() ) {
      printf( "\n" );
      break;
    }
    sleep( 1 );
  }
  roboxSetMotorLeft( 0 );
  roboxSetMotorRight( 0 );
  roboxSetStrobo( 0 );
  roboxSetBrake ( 0 );
  roboxShutdown();
  return 0;
}

