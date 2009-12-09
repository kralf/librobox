/*
 * Copyright (c) 1997 LAAS/CNRS
 * Copyright (c) 2004
 * Autonomous Systems Lab, Swiss Federal Institute of Technology.
 *
 *
 * All rights reserved.
 *
 * This code is derived from software contributed to LAAS by
 * Matthieu Herrb and Maher Khatib.
 *
 * Redistribution and use  in source  and binary  forms,  with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice, this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice,  this list of  conditions and the following disclaimer in
 *      the  documentation  and/or  other   materials provided  with  the
 *      distribution.
 *
 * THIS  SOFTWARE IS PROVIDED BY  THE  COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND  ANY  EXPRESS OR IMPLIED  WARRANTIES,  INCLUDING,  BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES  OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR  PURPOSE ARE DISCLAIMED. IN  NO EVENT SHALL THE COPYRIGHT
 * HOLDERS OR      CONTRIBUTORS  BE LIABLE FOR   ANY    DIRECT, INDIRECT,
 * INCIDENTAL,  SPECIAL,  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF  SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE   OF THIS SOFTWARE, EVEN   IF ADVISED OF   THE POSSIBILITY OF SUCH
 * DAMAGE.
 */

/*
 * TCP862 implementation by frederic.pont@epfl.ch
 *        inspired by posix.c
 *        January 2006
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <errno.h>
#include <tcp862.h>

//#define LIBSERIAL_TCP862_DEBUG
#include "tcp862_serial.h"

/*
 * --- tcp862_open --------------------------------------------------------
 *
 * Open device and set baud rate.
 */

int
tcp862_open(const char *device)
{
  return open(device, O_RDWR, 0);
}

/*
 * --- tcp862_close -------------------------------------------------------
 *
 * Close device.
 */

int
tcp862_close(int fd)
{
  return close(fd);
}



int
tcp862_setBaud(int fd, int baud)
{

  int result;
  TP862_OPERATION_MODE_STRUCT OperationMode;
  OSC_SOURCE oscSource;

  if (baud < TP_BAUDRATE_9600 || baud > TP_BAUDRATE_1500000) {
    return -1;
  }

  if (baud == TP_BAUDRATE_500000) {
    oscSource = OSCSOURCE_XTAL2;
    result = ioctl(fd,TP862_IOCT_SET_OSC_SRC,oscSource);
    if (result < 0) {
      printf("tcp862_setBaud: error changing OSC to XTAL2\n");
      return -1;
    }
  } else {
    oscSource = OSCSOURCE_XTAL1;
    result = ioctl(fd,TP862_IOCT_SET_OSC_SRC,oscSource);
    if (result < 0) {
      printf("tcp862_setBaud: error changing OSC to XTAL1\n");
      return -1;
    }
  }


//#warning TCP862 is set in RS422 mode!!

  OperationMode.commtype          = COMMTYPE_ASYNC;
  OperationMode.clockmode         = CLOCKMODE_CM7B;
  OperationMode.txclk_out         = DISABLED;
  OperationMode.transceivermode   = TRANSCEIVER_RS530;
  OperationMode.dce_dte           = DCEDTE_DCE;
  OperationMode.oversampling      = ENABLED;
  OperationMode.usetermchar       = DISABLED;
  OperationMode.termchar          = '\0';
  OperationMode.baudrate          = baud;
  OperationMode.clockinversion    = CLOCKINV_NONE;

  //result = ioctl(d->fd, TP862_IOCT_SET_BAUDRATE, baud);
  result = ioctl(fd, TP862_IOCS_SET_OPERATION_MODE, &OperationMode);

  if (result < 0)
    {
      // error setting baudrate
      printf("Error setting baudrate! %d\n", errno);
      return -1;
    }

  result=tcp862_setReadTimeout(fd,10);
  return result;

}


/*
 * --- tcp862_read --------------------------------------------------------
 *
 * Read at most `len' bytes in `buffer'. Returns the number acutally
 * read.
 */

int tcp862_readn(int dev_fd, unsigned char *buf, int nChars){

  int amountRead = 0, bytes_read = 0;
  int max_retries=10, retries=0;
  //  unsigned char* b0=buf;
  while(nChars > 0 && retries<max_retries) {
    amountRead = read(dev_fd, buf, nChars);
    if(amountRead < 0){
      fprintf(stderr, "read error\n");
      return -1;
    }else if(amountRead > 0) {
      bytes_read += amountRead;
      nChars -= amountRead;
      buf += amountRead;
      //      fprintf(stderr,"r");
    } else {
      //fprintf(stderr,"w");
      retries++;
    }
  }
/*   fprintf(stderr,"r[%d]\"", bytes_read); */
/*   int i; */
/*   for (i=0; i<bytes_read; i++){ */
/*     unsigned int c=b0[i]; */
/*     fprintf(stderr,"%02x ",c); */
/*   } */
/*   fprintf(stderr,"\"\n"); */

/*   if (nChars>0) */
/*     fprintf(stderr,"i(%d)",nChars); */
  return bytes_read;
}

/* int */
/* tcp862_read(int fd, char *buffer, int len) */
/* { */
/* #ifdef LIBSERIAL_TCP862_DEBUG */
/*   int result; */

/*   result = read(fd, buffer, len); */

/*   if(result) printf("** %dB received\n",result); */
/*   int i; */
/*   for (i=0;i<result;i++) printf(" %02X ",(unsigned char)buffer[i]); */
/*   printf("\n"); */

/*   return result; */
/* #else */
/*    return read(fd, buffer, len); */
/* #endif */
/* } */


/*
 * --- tcp862_write -------------------------------------------------------
 *
 * Write `len' bytes from `buffer'. Returns the number acutally written.
 */

int
tcp862_writen(int fd, unsigned char *buffer, int len)
{
  int i;
#ifdef LIBSERIAL_TCP862_DEBUG
  int result;
  printf("** %dB sending \n",len);
  for (i=0;i<len;i++)
    printf(" %02X ",(unsigned char)buffer[i]);
  printf("\n");
#endif

  return write(fd, buffer,len);

  /* NOT USED YET, BUT SHOULD BE INVESTIGATED */
  /* slow down transmition to ensure SICK can handle
   * communication at 500000 */
  for (i=0;i<len;i++) {
    if ( (write(fd, buffer+i,1) < 0) ) return i;
    //usleep(1);
  }
  return i;

}


/*
 * --- tcp862_flush -------------------------------------------------------
 *
 * Flush input buffer.
 */

int
tcp862_clearInputBuffer(int fd)
{
  TP862_OPERATION_MODE_STRUCT OperationMode;
  int result = ioctl(fd, TP862_IOCG_GET_OPERATION_MODE, &OperationMode);
  if (result < 0)
    {
      // error setting baudrate
      printf("Error  getting operation mode, %d\n", errno);
      return -1;
    }
  ioctl(fd, TP862_IOC_CLEAR_RX_BUFFER, &OperationMode);
  return 0;
}

int     tcp862_setReadTimeout(int fd, int jiffies){
  int result = ioctl(fd, TP862_IOCT_SET_READ_TIMEOUT, jiffies);
  if (result < 0)
    {
      // error setting baudrate
      printf("Error  setting the timeout, %d\n", errno);
      return -1;
    }
  return 0;
}
