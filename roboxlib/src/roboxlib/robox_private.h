#ifndef ROBOX_PRIVATE_H
#define ROBOX_PRIVATE_H


struct FileHandles {
  int bumpers[8];
  int brakeOut;
  int brakeIn;
  int stroboOut;
  int encoderLeft;
  int encoderRight;
  int motorsOut;
  int motorLeft;
  int motorRight;
  int watchdog;
  int powerOut;
  int emergencyIn;
  int supervisorIn;
  int maxHandle;
  int odometry;
  int speedOut;
};

int openHandle( const char * devicename, int readOnly );
void openHandles();
void closeHandles();
void * securityHandler( void * params );
double timestamp();



#endif /*ROBOX_PRIVATE_H*/
