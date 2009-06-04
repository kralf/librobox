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
};

struct Bumpers {
  int bumpers[8];
  double timestamp;
};

struct EncoderLeft {
  int value;
  double timestamp;
};

struct EncoderRight {
  int value;
  double timestamp;
};

struct Brake {
  int engaged;
  double timestamp;
};

struct Emergency {
  int value;
  double timestamp;
};

struct Supervisor {
  int value;
  double timestamp;
};


int openHandle( const char * devicename, int readOnly );
void openHandles();
void closeHandles();
void * inputHandler( void * params );
double timestamp();



#endif /*ROBOX_PRIVATE_H*/
