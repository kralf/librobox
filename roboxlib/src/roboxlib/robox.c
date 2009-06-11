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


struct FileHandles handles;
pthread_t securityThread;
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
int securityThreadRunning;

double cov_matrix[3][3];
double radius_r = 0.09;
double radius_l = 0.09;
double wheelbase = 0.545;
double k_r = 0.0001;
double k_l = 0.0001;

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

  roboxInitOdometry();
  roboxInitSpeed();

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
  void * status;
  pthread_mutex_lock( &mutex );
  securityThreadRunning = 0;
  pthread_mutex_unlock( &mutex );
  pthread_join( securityThread, &status );

  roboxSetMotorLeft( 2048 );
  roboxSetMotorRight( 2048 );
  roboxSetWatchdog( 0 );
  roboxSetMotorEnable( 0 );
  roboxSetBrake( 1 );
  roboxSetPower( 0 );
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

typedef struct {
  int prev_e0; /* old encoder value */
  int prev_e1; /* old encoder value */
} private_t;

static private_t private;

void roboxInitOdometry()
{
     int i, j;
     for (i = 0; i < 3; ++i)
       for (j = 0; j < 3; ++j)
       cov_matrix[i][j] = 0.0;

     private.prev_e0 = roboxGetEncoderLeft();
     private.prev_e1 = roboxGetEncoderRight();
}

double modulo_2pi(double theta)
{
  
    //while(theta >= 2*M_PI)
    //  theta -= 2*M_PI;
  
    //while(theta < 0)
    //  theta += 2*M_PI;
  
    //return theta;

    int n = floor(theta / (2 * M_PI));
    theta -= n * (2 * M_PI);
    
    if (theta > M_PI)
      theta -= (2 * M_PI);
    return theta;

}

int roboxGetOdometry ( double* x, double* y, double* theta )
{
    int e0, e1;
    double dalpha_l, dalpha_r;

    double drho_r;
    double drho_l;
    double drho;
    double dtheta;
    double beta;

    //double q_r;
    //double q_l;
  
    double sin_beta;
    double cos_beta;
    //double x_cs_p;
    //double x_cs_m;
    //double x_sc_p;
    //double x_sc_m;
  
    //double sigma_xx;
    //double sigma_xy;
    //double sigma_xt;
    //double sigma_yy;
    //double sigma_yt;
    //double sigma_tt;

    //double drho_sin_beta;
    //double drho_cos_beta;

    e0 = roboxGetEncoderLeft();
    e1 = roboxGetEncoderRight();

    dalpha_l = hal_encoder_to_rad(hal_encoder_delta(private.prev_e0,e0));
    dalpha_r = hal_encoder_to_rad(-hal_encoder_delta(private.prev_e1,e1));
    /* store old values */
    private.prev_e0 = e0;
    private.prev_e1 = e1;
  
    //sigma_xx = cov_matrix[0][0];
    //sigma_xy = cov_matrix[0][1];
    //sigma_xt = cov_matrix[0][2];
    //sigma_yy = cov_matrix[1][1];
    //sigma_yt = cov_matrix[1][2];
    //sigma_tt = cov_matrix[2][2];

    drho_r = radius_r * dalpha_r;
    drho_l = radius_l * dalpha_l;
  
    //q_r = k_r * fabs(drho_r);
    //q_l = k_l * fabs(drho_l);

    drho   = 0.5 * (drho_r + drho_l);

    dtheta = (drho_r - drho_l)/wheelbase;
  
    beta = *theta + dtheta;

    sin_beta = sin(beta);
    cos_beta = cos(beta);
  
    //drho_sin_beta = drho * sin_beta;
    //drho_cos_beta = drho * cos_beta;

    //x_cs_p = 0.5 * (cos_beta + sin_beta*drho/wheelbase);
    //x_cs_m = 0.5 * (cos_beta - sin_beta*drho/wheelbase);
    //x_sc_p = 0.5 * (sin_beta + cos_beta*drho/wheelbase);
    //x_sc_m = 0.5 * (sin_beta - cos_beta*drho/wheelbase);
  
    //cov_matrix[0][0] = sigma_xx - 2*drho_sin_beta*sigma_xt + pow(drho_sin_beta,2)*sigma_tt
    //  + q_r*pow(x_cs_m,2) + q_l*pow(x_cs_p,2);

    //cov_matrix[0][1] = sigma_xy - drho_sin_beta*sigma_yt + drho_cos_beta*sigma_xt 
    //  - drho_cos_beta*drho_sin_beta*sigma_tt 
    //  + q_r*x_cs_m*x_sc_p + q_l*x_cs_p*x_sc_m;  

    //cov_matrix[0][2] = sigma_xt - drho_sin_beta*sigma_tt
    //  + q_r*x_cs_m/wheelbase - q_l*x_cs_p/wheelbase;

    //cov_matrix[1][0] = cov_matrix[0][1];

    //cov_matrix[1][1] = sigma_yy + 2*drho_cos_beta*sigma_yt + pow(drho_cos_beta,2)*sigma_tt
    //  + q_r*pow(x_sc_p,2) + q_l*pow(x_sc_m,2);

    //cov_matrix[1][2] = sigma_yt + drho_cos_beta*sigma_tt 
    //  + q_r*x_sc_p/wheelbase - q_l*x_sc_m/wheelbase;

    //cov_matrix[2][0] = cov_matrix[0][2];

    //cov_matrix[2][1] = cov_matrix[1][2];

    //cov_matrix[2][2] = sigma_tt 
    //  + (q_r + q_l)/pow(wheelbase,2);
  
    *x += dalpha_l;//drho * cos(beta);
    *y += dalpha_r;//drho * sin(beta);
    *theta = modulo_2pi(*theta + dtheta); 

    return 0;
}

//------------------------------------------------------------------------------

#define NWHEELS 2
#define SPEED_P      2.9
#define SPEED_I      0.7
#define SPEED_D      -1.2
#define SPEED_ILIMIT 2.0

typedef struct {
  double stime;

  /* pid parameters */
  double p;
  double i;
  double d;
  double ilimit;
  double integ[NWHEELS];

  double current[NWHEELS]; /* current speed [rad/s] */
  double target[NWHEELS];  /* desired speed [rad/s] */
  
  double aovalue[NWHEELS];    /* analog out value */
  int pcounter[NWHEELS];   /* previous encoder value */
  double ptime[NWHEELS];   /* last timestamp */
  
  /* values kept for DEBUG */
  int current_i[NWHEELS];  /* integer value of current speed */
  int target_i[NWHEELS];   /* integer value of target speed */
  int dencoders[NWHEELS];  /* last delta encoders DBG */
  int correction[NWHEELS]; /* last PID correction applied DBG */

} speed_private;

static speed_private spd;


void roboxInitSpeed()
{
  int i;
  struct timeval tv;

  gettimeofday(&tv, 0);
  spd.stime = tv.tv_sec+tv.tv_usec*1e-6;  

  for(i=0; i < 2; i++) {
    spd.target[i]   = 0.0;
    spd.current[i]   = 0.0;
    spd.pcounter[i] = (i == 0) ? roboxGetEncoderLeft() : 
      roboxGetEncoderRight();
    spd.ptime[i] = 0.0;   

    spd.aovalue[i] = 2048;
  }
  //private.from_user.speed_l = 0.0;
  //private.from_user.speed_r = 0.0;

  /* PID parameters */
  spd.p      = SPEED_P;
  spd.i      = SPEED_I;
  spd.d      = SPEED_D;
  spd.ilimit = SPEED_ILIMIT;
  spd.integ[i] = 0; //what is integ ??
}


/* hal_speed_update()
 * function called by real-time task to update the motor value
 * on one wheel */
void roboxSetSpeed( int wheel, double speed_target )
{
  double target = speed_target, ptime, dtime = 0.0;
  int i = wheel;
  double delta_speed, accel = 0.0, speed = 0.0, integ, correction;
  int counter;
  struct timeval tv;

  counter = (i == 1) ? roboxGetEncoderLeft() : roboxGetEncoderRight();
  spd.dencoders[i] = hal_encoder_delta(spd.pcounter[i], counter);
  spd.dencoders[i] = (i == 1) ? spd.dencoders[i] : -spd.dencoders[i];
  spd.pcounter[i] = counter;
  //private.pcounter[2] = counter[2];

  gettimeofday(&tv, 0);
  ptime = tv.tv_sec+tv.tv_usec*1e-6;
  if (spd.ptime[i] > 0.0)
    dtime = ptime-spd.ptime[i];
  spd.ptime[i] = ptime;

  if (dtime > 0.0) {
    speed = hal_encoder_to_rad(spd.dencoders[i]) / dtime;
    accel = (speed - spd.current[i]) / dtime;
  }
  spd.current[i] = speed;

  //printf("%lf -- %6d %6d -- %6.2f %6.2f\n", dtime,
    //spd.dencoders[0], spd.dencoders[1],
    //hal_encoder_to_rad(spd.dencoders[0]) / dtime, 
    //hal_encoder_to_rad(spd.dencoders[1]) / dtime);  
       
      
  /* compute PID and correction */
  //delta_speed = spd.target[i] - spd.current[i];
  delta_speed = target - spd.current[i];
  integ = spd.integ[i] + delta_speed;
  if( (integ > 0.0) && (integ > spd.ilimit) )  integ = spd.ilimit;
  if( (integ < 0.0) && (integ < -spd.ilimit) ) integ = -spd.ilimit;
  spd.integ[i] = integ;
  correction = (spd.p*delta_speed + spd.d*accel + spd.i*integ);

  /* apply correction to current analog out value */
  spd.correction[i] = correction;
  spd.aovalue[i] = (i == 1) ? spd.aovalue[i] - correction : spd.aovalue[i] + correction;


  /* XXX aovalue is passed as an unsigned int to hal_motor_set_channel(),
   * so we MUST check we don't go below 0... otherwise the motor will be
   * set at max speed by hal_motor_set_channel(). Furthermore, this
   * aovalue should be passed as a pointer so that it can be properly
   * clipped by hal_motor_set_channel and we don't get out of
   * sync with the actual used value.
   * Let's do it the dirty way for now. */

  if (spd.aovalue[i] < 1) spd.aovalue[i] = 1;
  if (spd.aovalue[i] > 4095) spd.aovalue[i] = 4095;

  /* reset analog out to 2048 when the robot is stopped, to prevent applying useless
   * force on wheels */
  /* FIXME: we could optimize, by not check here/everytime */
  //if ( (spd.target[i] == 0.0) && (spd.current[i] == 0.0) ) {
    if ( (target == 0.0) && (spd.current[i] == 0.0) ) {
    spd.aovalue[i] = 2048;
  }

  /* set actuators */
  
  if (i == 0) 
    roboxSetMotorLeft( spd.aovalue[0] );
  else 
    roboxSetMotorRight( spd.aovalue[1] );

  fprintf(stdout, "%lf   %8.4f  %8.4f  %8.4f\n", ptime-spd.stime, speed, delta_speed, correction);
  fprintf(stderr, "%d %d\n", (int)spd.aovalue[0], (int)spd.aovalue[1]);  

  return;
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
