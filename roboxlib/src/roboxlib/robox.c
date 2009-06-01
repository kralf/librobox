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
  handles.bumpers[0] = openHandle( "/dev/robox/sensors/bumper/front", 1 );
  handles.bumpers[1] = openHandle( "/dev/robox/sensors/bumper/leftfront", 1 );
  handles.bumpers[2] = openHandle( "/dev/robox/sensors/bumper/left", 1 );
  handles.bumpers[3] = openHandle( "/dev/robox/sensors/bumper/leftback", 1 );
  handles.bumpers[4] = openHandle( "/dev/robox/sensors/bumper/back", 1 );
  handles.bumpers[5] = openHandle( "/dev/robox/sensors/bumper/rightback", 1 );
  handles.bumpers[6] = openHandle( "/dev/robox/sensors/bumper/right", 1 );
  handles.bumpers[7] = openHandle( "/dev/robox/sensors/bumper/rightfront", 1 );
  handles.brakeOut   = openHandle( "/dev/robox/drive/brake/disengage", 0 );
  handles.brakeIn    = openHandle( "/dev/robox/drive/brake/disengaged", 1 );
  handles.stroboOut  = openHandle( "/dev/robox/security/flashlight", 0 );
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
}

//------------------------------------------------------------------------------


void * inputHandler( void * params )
{
  fd_set inputFiles;
  struct timeval timeout;
  int inputReady;
  size_t bufferSize = 1000;
  char buffer[bufferSize];
  inputThreadRunning = 1;
  
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
    } else {
      /*Timed out*/
    }
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
