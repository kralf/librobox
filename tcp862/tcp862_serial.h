#ifndef _TCP862_H_
#define _TCP862_H_

int tcp862_open(const char *device);
int tcp862_close(int fd);
int tcp862_setBaud(int fd, int baud);
int tcp862_readn(int fd, unsigned char *buffer, int len);
int tcp862_writen(int fd, unsigned char *buffer, int len);
int tcp862_poll(int fd);
int tcp862_flush(int fd);
int tcp862_clearInputBuffer(int fd);
int tcp862_setReadTimeout(int fs, int jiffies);

#endif
