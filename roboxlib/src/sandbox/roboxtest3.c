#include <roboxlib/robox.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <fcntl.h>
#include <unistd.h>

int main( int argc, char * argv[] ) 
{
   roboxInit();
   static const int sTime = 10;   

   static const double rConst [2] = {
			0.09, 0.54/2
};
   static const double oPose [3] = {
			0, 0, 0
};
   static const double gPose [5] = {
			4, 2, -M_PI/4, 0.05, M_PI/100
};

   static const double nPose [3] = {
                        // calculated out of new position ??
};
 
   static const double kParams [3] = {
			0.15, 0.42, -0.15
};
	for (;;){
          roboxSetPower( 1 );
	  roboxSetBrake( 0 );
          roboxSetMotorEnable( 1 );
	  roboxSetStrobo( 1 );
	
 	  dSl = roboxGetEncoderLeft();
	  dSr = roboxGetEncoderRight();

	   int ContStep (int sTime, int rConst, int oPose, int gPose, int nPose, int K, int dSl, int dSr, int* nOmega, int* nPolar, int* dt) {
    	  // run ContStep with Omega1,2 and o,g,n-Pose , outputting Omega1,2, oPose
                       
          }
	  roboxSetMotorLeft( int nOmega[0] );
	  roboxSetMotorRight( int nOmega[1] );
	  
	  intstatus = ContStep (sTime, rConst, oPose, gPose, nPose, K, dSl, dSr, &nOmega. &nPolar, &dt)          

	  roboxSetWatchdog ( 1 );

          if ( (nPolar[0]<=gPose[4] && dt<=gPose[5]) || roboxGetEmergency() || roboxGetSupervisor() || roboxGetBumper() ) { 
      		printf( "\n" );
      	  	break;
 	  };
	sleep( 1 );        
	}  

  roboxSetStrobo( 0 );  
  roboxSetBrake ( 1 );
  roboxSetMotorLeft( 0 );
  roboxSetMotorRight( 0 );

// start executing the arm's motion

  roboxShutdown();
  return 0;
}
}

