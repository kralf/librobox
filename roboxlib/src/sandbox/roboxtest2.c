#include <roboxlib/robox.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <math.h>

double x = 0.0;
double y = 0.0;
double theta = 0.0;

int quit = 0;

void signaled(int signal) {
  quit = 1;
}

#define enc_to_rad(v) v/(4.0*500.0*50.0)*2.0*M_PI

int main( int argc, char * argv[] ) 
{
  struct timeval start_time;

  roboxInit();

  signal(SIGINT, signaled);

  gettimeofday(&start_time, 0);
  while (!quit) {
    //fprintf( stderr, "\rEncoder left %8.2f right %8.2f", 
    //  enc_to_rad(roboxGetEncoderLeft())*180.0/M_PI, 
    //  enc_to_rad(roboxGetEncoderRight())*180.0/M_PI );

    struct timeval current_time;
    gettimeofday(&current_time, 0);
    double t = (current_time.tv_sec-start_time.tv_sec)+
      (current_time.tv_usec-start_time.tv_usec)*1e-6;

    roboxGetOdometry ( &x, &y, &theta );
    fprintf ( stderr, "\rPosition x %8.2f y %8.2f theta %8.2f", 
      x, y, theta);
    //fprintf(stdout, "%lf  %lf  %lf  %lf\n", t, x, y, theta);
    usleep( 1000 );
  }
  printf( "\n" );
  
  roboxShutdown();
  
  return 0;
}

