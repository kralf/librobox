#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <memory.h>
#include <sys/time.h>
#include <fcntl.h>
#include <pthread.h>

#include "robox.h"
#include "robox_private.h"


#include <math.h>


static struct FileHandles handles;
static pthread_t securityThread;
static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
static int securityThreadRunning;
static int radius_r = 90;
static int radius_l = 90;
static int wheelbase = 545;
static double cov_matrix[3][3];
static double x;
static double y;
static double theta;


//------------------------------------------------------------------------------

void
roboxInit()
{
  openHandles();
  roboxSetMotorLeft( 2048 );
  roboxSetMotorRight( 2048 );
  roboxSetPower( 1 );
  roboxSetBrake( 0 );
  roboxSetWatchdog( 0 );
  roboxSetMotorEnable( 1 );

  if ( pthread_create( &securityThread, NULL, securityHandler, (void *) NULL ) > 0 ) {
    fprintf( stderr, "Problem creating main thread" );
    closeHandles();
    exit( 1 );
  }
}

//------------------------------------------------------------------------------

void
roboxShutdown()
{
  roboxSetMotorLeft( 2048 );
  roboxSetMotorRight( 2048 );
  roboxSetMotorEnable( 0 );
  roboxSetPower( 0 );
  roboxSetBrake( 1 );

  void * status;
  pthread_mutex_lock( &mutex );
  securityThreadRunning = 0;
  pthread_mutex_unlock( &mutex );
  pthread_join( securityThread, &status );

  roboxSetWatchdog( 0 );
  roboxSetStrobo( 0 );

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
  handles.emergencyIn  = openHandle( "/dev/robox/security/stop/emergency", 1 );
  handles.supervisorIn = openHandle( "/dev/robox/security/stop/supervisor", 1 );
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
  close( handles.supervisorIn );
}

//------------------------------------------------------------------------------

void * securityHandler( void * params )
{
  fd_set inputFiles;
  struct timeval timeout;
  int inputReady;
  securityThreadRunning = 1;      
  int watchdog = 0;
  for (;;) {
    pthread_mutex_lock( &mutex );
    int shouldBreak = ! securityThreadRunning;
    pthread_mutex_unlock( &mutex );
    
    if ( shouldBreak ) {
      break;
    }

    // timeout.tv_sec  = 0;
    // timeout.tv_usec = 5000;
    // FD_ZERO( &inputFiles );
    // inputReady = select( 0, (fd_set *) NULL, (fd_set *) NULL, (fd_set *) NULL, &timeout );
    // if ( inputReady < 0 ) {
    //   fprintf( stderr, "Problem while getting data\n" );
    //   exit( 1 );
    // }
    usleep(5000);
    watchdog = !watchdog;
    roboxSetWatchdog( watchdog );
    //fprintf( stderr, "watchdog: %i\n", watchdog );

    if ( !roboxGetEmergency() ) {
      

      roboxSetStrobo( 0 );
    }
    else {
      roboxSetMotorLeft( 2048 );
      roboxSetMotorRight( 2048 );
      roboxSetStrobo( 1 );
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
  size_t bufferSize = 100;
  char buffer[bufferSize];
  int value = 0;
  int inputReady = read( handles.bumpers[number], buffer, bufferSize );
  if ( inputReady ) {
    sscanf( buffer, "%i\n", &value );
  }
  return value;
}

//------------------------------------------------------------------------------

int roboxGetEncoderLeft()
{
  size_t bufferSize = 100;
  char buffer[bufferSize];
  int value = 0;
  int inputReady = read( handles.encoderLeft, buffer, bufferSize );
  if ( inputReady ) {
    sscanf( buffer, "%i\n", &value );
  }
  return value;
}
//------------------------------------------------------------------------------

int roboxGetEncoderRight()
{
  size_t bufferSize = 100;
  char buffer[bufferSize];
  int value = 0;  
  int inputReady = read( handles.encoderRight, buffer, bufferSize );
  if ( inputReady ) {
    sscanf( buffer, "%i\n", &value );
  }
  return value;
}

//------------------------------------------------------------------------------
int roboxGetBrake()
{
  size_t bufferSize = 100;
  char buffer[bufferSize];
  int value = 0;
  int inputReady = read( handles.brakeIn, buffer, bufferSize );
  if ( inputReady ) {
    sscanf( buffer, "%i\n", &value );
  }
  return value;
}

//------------------------------------------------------------------------------

int roboxGetEmergency()
{
  size_t bufferSize = 100;
  char buffer[bufferSize];
  int value = 0;
  int inputReady = read( handles.emergencyIn, buffer, bufferSize );
  if ( inputReady ) {
    sscanf( buffer, "%i\n", &value );
  }
  return ! value;
}

//------------------------------------------------------------------------------

int roboxGetSupervisor()
{
  size_t bufferSize = 100;
  char buffer[bufferSize];
  int value = 0;
  int inputReady = read( handles.supervisorIn, buffer, bufferSize );
  if ( inputReady ) {
    sscanf( buffer, "%i\n", &value );
  }
  return value;
}

//------------------------------------------------------------------------------

#define LIMIT 0x00ffffff
#define WIN   LIMIT/4
int hal_encoder_delta( value0, value1 )
{
  
  if( value0>LIMIT-WIN && value1<WIN ) {
    return LIMIT-value0 + value1;
  }
  else if ( value0<WIN && value1>LIMIT-WIN ) {
    return  -( LIMIT-value1 + value0 );
  }
  else {
    return value1 - value0;
  }
}

//------------------------------------------------------------------------------

double hal_encoder_to_rad( value )
{
  return value/(4.0*500.0*50.0)*2.0*M_PI;
}

//------------------------------------------------------------------------------

int roboxGetOdometry ( double* x, double* y, double* theta )
{
   typedef struct {
     double radius_r;
     double radius_l;
     double wheelbase;
     double k_r;
     double k_l;
   } parameters;


typedef struct {

int prev_e0; /* old encoder value */
int prev_e1; /* old encoder value */

} private_t;

static private_t private;

static void hal_odometry_task(int arg)
{
  double dalpha_l, dalpha_r;
  int e0, e1;

  init_values();
  private.prev_e0 = roboxGetEncoderLeft();
  private.prev_e1 = roboxGetEncoderRight();

  for(;;) {
    
    e0 = roboxGetEncoderLeft();
    e1 = roboxGetEncoderRight();

    
    dalpha_r = hal_encoder_delta(private.prev_e0,e0);
    dalpha_r = hal_encoder_delta(private.prev_e1,e1);
    dalpha_l = HAL_ENCODER_TO_RAD(dalpha_l);
    dalpha_r = HAL_ENCODER_TO_RAD(dalpha_r);
    
    /* store old values */
    private.prev_e0 = e0;
    private.prev_e1 = e1;

    /* update */
    //hal_odometry_update(param,dalpha_l,dalpha_r);
  }

}

//struct hal_odometry_data_s
static int hal_odometry_update(parameters param,
			       double dalpha_l, double dalpha_r)
{
  
  double drho_r;
  double drho_l;
  double drho;
  double dtheta;
  double beta;

  double q_r;
  double q_l;
  
  double sin_beta;
  double cos_beta;
  double x_cs_p;
  double x_cs_m;
  double x_sc_p;
  double x_sc_m;
  
  double sigma_xx;
  double sigma_xy;
  double sigma_xt;
  double sigma_yy;
  double sigma_yt;
  double sigma_tt;

  double drho_sin_beta;
  double drho_cos_beta;
  
  sigma_xx = cov_matrix[0][0];
  sigma_xy = cov_matrix[0][1];
  sigma_xt = cov_matrix[0][2];
  sigma_yy = cov_matrix[1][1];
  sigma_yt = cov_matrix[1][2];
  sigma_tt = cov_matrix[2][2];


  drho_r = param.radius_r * dalpha_r;
  drho_l = param.radius_l * dalpha_l;
  
  q_r = param.k_r * fabs(drho_r);
  q_l = param.k_l * fabs(drho_l);

  drho   = 0.5 * (drho_r + drho_l);

  dtheta = (drho_r - drho_l)/param.wheelbase;
  
  beta = theta + dtheta*0.5;

  sin_beta = sin(beta);
  cos_beta = cos(beta);
  
  drho_sin_beta = drho * sin_beta;
  drho_cos_beta = drho * cos_beta;

  x_cs_p = 0.5 * (cos_beta + sin_beta*drho/param.wheelbase);
  x_cs_m = 0.5 * (cos_beta - sin_beta*drho/param.wheelbase);
  x_sc_p = 0.5 * (sin_beta + cos_beta*drho/param.wheelbase);
  x_sc_m = 0.5 * (sin_beta - cos_beta*drho/param.wheelbase);

  
    cov_matrix[0][0] = sigma_xx - 2*drho_sin_beta*sigma_xt + pow(drho_sin_beta,2)*sigma_tt
      + q_r*pow(x_cs_m,2) + q_l*pow(x_cs_p,2);

    cov_matrix[0][1] = sigma_xy - drho_sin_beta*sigma_yt + drho_cos_beta*sigma_xt 
      - drho_cos_beta*drho_sin_beta*sigma_tt 
      + q_r*x_cs_m*x_sc_p + q_l*x_cs_p*x_sc_m;  

    cov_matrix[0][2] = sigma_xt - drho_sin_beta*sigma_tt
      + q_r*x_cs_m/param.wheelbase - q_l*x_cs_p/param.wheelbase;

    cov_matrix[1][0] = cov_matrix[0][1];

    cov_matrix[1][1] = sigma_yy + 2*drho_cos_beta*sigma_yt + pow(drho_cos_beta,2)*sigma_tt
      + q_r*pow(x_sc_p,2) + q_l*pow(x_sc_m,2);

    cov_matrix[1][2] = sigma_yt + drho_cos_beta*sigma_tt 
      + q_r*x_sc_p/param.wheelbase - q_l*x_sc_m/param.wheelbase;

    cov_matrix[2][0] = cov_matrix[0][2];

    cov_matrix[2][1] = cov_matrix[1][2];

    cov_matrix[2][2] = sigma_tt 
      + (q_r + q_l)/pow(param.wheelbase,2);
  
    x += drho * cos(beta);
    y += drho * sin(beta);
    theta = modulo_2pi(theta + dtheta); 

  
    return 0;
  }


  double modulo_2pi(double theta)
  {
  
    while(theta >= 2*M_PI)
      theta -= 2*M_PI;
  
    while(theta < 0)
      theta += 2*M_PI;
  
    return theta;

  }

   *x = cov_matrix[0][0];
   *y = cov_matrix[1][1];
   *theta = cov_matrix[2][2];
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

void roboxSetMotorEnable( int value )
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
