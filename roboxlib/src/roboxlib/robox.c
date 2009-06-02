#include "robox.h"
#include "robox_private.h"
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <memory.h>
#include <sys/time.h>
#include <fcntl.h>
#include <pthread.h>

static struct FileHandles handles;
static pthread_t inputThread;
static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
static int inputThreadRunning;

static struct Bumpers bumperStatus;
static struct Brake brakeStatus; 
static struct EncoderLeft encoderLeftStatus;
static struct EncoderRight encoderRightStatus;
static struct Emergency emergencyStatus;


//------------------------------------------------------------------------------

void
roboxInit()
{
  openHandles();
  if ( pthread_create( &inputThread, NULL, inputHandler, (void *) NULL  ) > 0 ) {
    fprintf( stderr, "Problem creating main thread" );
    closeHandles();
    exit( 1 );
  }
}

//------------------------------------------------------------------------------

void
roboxShutdown()
{
  void * status;
  pthread_mutex_lock( &mutex );
  inputThreadRunning = 0;
  pthread_mutex_unlock( &mutex );
  pthread_join( inputThread, &status );
  closeHandles();
}

//------------------------------------------------------------------------------

int 
openHandle( const char * devicename, int readOnly )
{
  int result;
  if ( readOnly ) {
    result = open( devicename, O_RDONLY );
  } else {
    result = open( devicename, O_WRONLY );
  }
  if ( result < 0 ) {
    fprintf( stderr, "Problem while opening device \"%s\"\n", devicename );
    closeHandles();
    exit( 1 );
  }
  if ( result > handles.maxHandle ) {
    handles.maxHandle = result;
  }
  return result;
}

//------------------------------------------------------------------------------

void 
openHandles()
{
  memset( &handles, sizeof( handles), 0 );
  handles.bumpers[0]   = openHandle( "/dev/robox/sensors/bumper/front", 1 );
  handles.bumpers[1]   = openHandle( "/dev/robox/sensors/bumper/leftfront", 1 );
  handles.bumpers[2]   = openHandle( "/dev/robox/sensors/bumper/left", 1 );
  handles.bumpers[3]   = openHandle( "/dev/robox/sensors/bumper/leftback", 1 );
  handles.bumpers[4]   = openHandle( "/dev/robox/sensors/bumper/back", 1 );
  handles.bumpers[5]   = openHandle( "/dev/robox/sensors/bumper/rightback", 1 );
  handles.bumpers[6]   = openHandle( "/dev/robox/sensors/bumper/right", 1 );
  handles.bumpers[7]   = openHandle( "/dev/robox/sensors/bumper/rightfront", 1 );
  handles.brakeOut     = openHandle( "/dev/robox/drive/brake/disengage", 0 );
  handles.brakeIn      = openHandle( "/dev/robox/drive/brake/disengaged", 1 );
  handles.stroboOut    = openHandle( "/dev/robox/security/flashlight", 0 );
  handles.encoderLeft  = openHandle( "/dev/robox/sensors/encoders/left", 1 );
  handles.encoderRight = openHandle( "/dev/robox/sensors/encoders/right", 1 );
  handles.motorsOut    = openHandle( "/dev/robox/drive/motor/enable", 0 );
  handles.motorLeft    = openHandle( "/dev/robox/drive/motor/left", 0 );
  handles.motorRight   = openHandle( "/dev/robox/drive/motor/right", 0 );
  handles.watchdog     = openHandle( "/dev/robox/security/watchdog", 0 );
  handles.powerOut     = openHandle( "/dev/robox/power/engage", 0 );
  handles.emergencyIn  = openHandle( "/dev/robox/security/stop/emergency", 0 );
}

//------------------------------------------------------------------------------

void 
closeHandles()
{
  size_t i = 0;
  for ( i = 0; i < 8; ++i ) {
    if ( handles.bumpers[i] ) {
      close( handles.bumpers[i] );
    }
  }
  close( handles.brakeOut );
  close( handles.brakeIn );
  close( handles.stroboOut );
  close( handles.encoderLeft );
  close( handles.encoderRight );
  close( handles.motorsOut);
  close( handles.motorLeft );
  close( handles.motorRight );
  close( handles.watchdog );
  close( handles.powerOut );
  close( handles.emergencyIn );
}

//------------------------------------------------------------------------------

void * inputHandler( void * params )
{
  fd_set inputFiles;
  struct timeval timeout;
  int inputReady;
  size_t bufferSize = 1000;
  char buffer[bufferSize];
  inputThreadRunning = 1;      if ( FD_ISSET( handles.brakeIn, &inputFiles ) ) {
        inputReady = read( handles.brakeIn, buffer, bufferSize );
        if ( inputReady ) {
          pthread_mutex_lock( &mutex );
          sscanf( buffer, "%i\n", &brakeStatus.engaged );
          brakeStatus.engaged = ! brakeStatus.engaged;
          brakeStatus.timestamp = timestamp();
          pthread_mutex_unlock( &mutex );
        }
      }
  
  for (;;) {
    pthread_mutex_lock( &mutex );
    int shouldBreak = ! inputThreadRunning;
    pthread_mutex_unlock( &mutex );
    
    if ( shouldBreak ) {
      break;
    }

    timeout.tv_sec  = 0;
    timeout.tv_usec = 50000;
    FD_ZERO( &inputFiles );
    size_t i = 0;
    for ( i = 0; i < 8; ++i ) {
      FD_SET( handles.bumpers[i], &inputFiles );
    }
    FD_SET( handles.brakeIn, &inputFiles );
    FD_SET( handles.encoderLeft, &inputFiles );
    FD_SET( handles.encoderRight, &inputFiles );
    FD_SET( handles.emergencyIn, &inputFiles );
    
    inputReady = select( handles.maxHandle + 1, &inputFiles, (fd_set *) NULL, (fd_set *) NULL, &timeout );
    if ( inputReady < 0 ) {
      fprintf( stderr, "Problem while getting data\n" );
      exit( 1 );
    }
    
    if ( inputReady ) {
      size_t i = 0;
      for ( i = 0; i < 8; ++i ) {
        if ( FD_ISSET( handles.bumpers[i], &inputFiles ) ) {
          inputReady = read( handles.bumpers[i], buffer, bufferSize );
          if ( inputReady ) {
            pthread_mutex_lock( &mutex );
            sscanf( buffer, "%i\n", &bumperStatus.bumpers[i] );
            bumperStatus.timestamp = timestamp();
            pthread_mutex_unlock( &mutex );
          }
        }
      }
      if ( FD_ISSET( handles.emergencyIn, &inputFiles ) ) {
        inputReady = read( handles.emergencyIn, buffer, bufferSize );
        if ( inputReady ) {
          pthread_mutex_lock( &mutex );
          sscanf( buffer, "%i\n", &EmergencyStatus.value );
          EmergencyStatus.value = ! EmergencyStatus.value;
          EmergencyStatus.timestamp = timestamp();
          pthread_mutex_unlock( &mutex );
        }
      }
      if ( FD_ISSET( handles.brakeIn, &inputFiles ) ) {
        inputReady = read( handles.brakeIn, buffer, bufferSize );
        if ( inputReady ) {
          pthread_mutex_lock( &mutex );
          sscanf( buffer, "%i\n", &brakeStatus.engaged );
          brakeStatus.engaged = ! brakeStatus.engaged;
          brakeStatus.timestamp = timestamp();
          pthread_mutex_unlock( &mutex );
        }
      }
      if ( FD_ISSET( handles.encoderLeft, &inputFiles ) ) {
	inputReady = read( handles.encoderLeft, buffer, bufferSize );
	if ( inputReady ) {
	  pthread_mutex_lock ( &mutex );
	  sscanf( buffer, "%i\n", &encoderLeftStatus.value );
          encoderLeftStatus.timestamp = timestamp();
   	  pthread_mutex_unlock( &mutex );
	}
      }
      if ( FD_ISSET( handles.encoderLRight, &inputFiles ) ) {
	inputReady = read( handles.encoderRight, buffer, bufferSize );
	if ( inputReady ) {
	  pthread_mutex_lock ( &mutex );
	  sscanf( buffer, "%i\n", &encoderRightStatus.value );
          encoderRightStatus.timestamp = timestamp();
   	  pthread_mutex_unlock( &mutex );
	}
      }

    } else {
      /*Timed out*/
    }
    roboxSetWatchdog( 0 );
  }
  return NULL;
}

//------------------------------------------------------------------------------

double
timestamp()
{
  struct timeval tv;
  struct timezone tz;
  gettimeofday(&tv, &tz);
  return tv.tv_sec + tv.tv_usec / 1000000.0;
}

//------------------------------------------------------------------------------

int roboxGetBumper( int number )
{
  pthread_mutex_lock( &mutex );
  int result = bumperStatus.bumpers[number];
  pthread_mutex_unlock( &mutex );
  return result;
}

//------------------------------------------------------------------------------

void roboxGetEncoderLeft()
{
  pthread_mutex_lock( &mutex );
  int result = encoderLeftStatus.value;
  pthread_mutex_unlock( &mutex );
  return result;
}
//------------------------------------------------------------------------------

void roboxGetEncoderRight()
{
  pthread_mutex_lock( &mutex );
  int result = encoderRightStatus.value;
  pthread_mutex_unlock( &mutex );
  return result;
}
//------------------------------------------------------------------------------
int roboxGetBrake()
{
  pthread_mutex_lock( &mutex );
  int result = brakeStatus.engaged;
  pthread_mutex_unlock( &mutex );
  return result;
}

//------------------------------------------------------------------------------

void roboxSetBrake( int value )
{
  char s[100];
  sprintf( s, "%i\n", ! value );
  write( handles.brakeOut, s, strlen( s ) );
}

//------------------------------------------------------------------------------

void roboxSetStrobo( int value )
{
  char s[100];
  sprintf( s, "%i\n", value );
  write( handles.stroboOut, s, strlen( s ) );
}

//------------------------------------------------------------------------------

void roboxSetMotorLeft( int value )
{
  char s[100];
  sprintf( s, "%i\n", value );
  write( handles.motorLeft, s, strlen( s ) );
}

//------------------------------------------------------------------------------

void roboxSetMotorsEnable( int value )
{
  char s[100];
  sprintf( s, "%i\n", value );
  write( handles.motorsOut, s, strlen( s ) );
}

//------------------------------------------------------------------------------

void roboxSetMotorRight( int value )
{
  char s[100];
  sprintf( s, "%i\n", value );
  write( handles.motorRight, s, strlen( s ) );
}

//------------------------------------------------------------------------------

void roboxSetWatchdog( int value )
{
  char s[100];
  sprintf( s, "%i\n", value );
  write( handles.watchdog, s, strlen( s ) );
}

//------------------------------------------------------------------------------

void roboxSetPower( int value )
{
  char s[100];
  sprintf( s, "%i\n", value );
  write( handles.powerOut, s, strlen( s ) );
}

//------------------------------------------------------------------------------

void roboxSetEmergency( int value )
{
  char s[100];
  sprintf( s, "%i\n", value );
  write( handles.emergencyOut, s, strlen( s ) );
}
