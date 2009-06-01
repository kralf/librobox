#ifndef ROBOX_PRIVATE_H
#define ROBOX_PRIVATE_H


struct FileHandles {
  int bumpers[8];
  int brakeOut;
  int brakeIn;
  int stroboOut;
  int maxHandle;
};

struct Bumpers {
  int bumpers[8];
  double timestamp;
};

struct Brake {
  int engaged;
  double timestamp;
};

int openHandle( const char * devicename, int readOnly );
void openHandles();
void closeHandles();
void * inputHandler( void * params );
double timestamp();



#endif /*ROBOX_PRIVATE_H*/
