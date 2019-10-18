#ifndef _SERIAL_H_
#define _SERIAL_H_

#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <netdb.h>
#include <pthread.h>
#include <time.h>
#include <termios.h>
#include <assert.h>
#include <fcntl.h>
#include <sstream>
#include <string.h>
#include <errno.h>

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdint.h>
#include <stdarg.h>


class SERIAL {
public:
  int BAUDRATE;
  int fd;
  char com_port_path[100];
  SERIAL(const char *com_port_path, int BAUDRATE);
  int open_canonical(int packet_size, struct termios *oldtio, struct termios *newtio);
  int open_non_canonical(struct termios *oldtio, struct termios *newtio);
  void close(struct termios *oldtio);
  int write_serial(char *data);
  int read_serial(char *data, int length);
  int avail_serial();
  bool purge();
private:
  
};

#endif