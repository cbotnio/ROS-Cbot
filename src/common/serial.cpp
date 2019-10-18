#include "asv_ros_revamp/common/serial.hpp"


SERIAL::SERIAL(const char *com_port_path_string, int BAUD_RATE)
{
  strcpy(com_port_path, com_port_path_string);
  BAUDRATE = BAUD_RATE;
}

int SERIAL::open_canonical(int packet_size, struct termios *oldtio, struct termios *newtio) 
{
  fd = open(com_port_path, O_RDWR | O_NOCTTY | O_NONBLOCK);

  tcgetattr(fd, oldtio);
  bzero(newtio, sizeof(*newtio));

  newtio->c_cflag = BAUDRATE ^ CRTSCTS | CS8 | CLOCAL | CREAD;

  newtio->c_iflag  = IGNPAR;
  newtio->c_iflag  = IGNBRK;
  newtio->c_oflag  = 0;
  newtio->c_lflag  = 0;
  newtio->c_cc[VTIME]  = 0;
  newtio->c_cc[VMIN]    =  packet_size;
  tcflush(fd, TCIFLUSH);
  tcsetattr(fd, TCSANOW, newtio);
  return fd;
}


int SERIAL::open_non_canonical(struct termios *oldtio, struct termios *newtio)
{

  fd = open(com_port_path, O_RDWR | O_NOCTTY | O_NONBLOCK);

  tcgetattr(fd, oldtio);
  bzero(newtio, sizeof(*newtio));

  newtio->c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;

  newtio->c_iflag     = IGNPAR;
  newtio->c_oflag     = 0;
  newtio->c_lflag     = ICANON;
  newtio->c_cc[VINTR]   = 0;
  newtio->c_cc[VQUIT]   = 0;
  newtio->c_cc[VERASE]  = 0;
  newtio->c_cc[VKILL]   = 0;
  newtio->c_cc[VEOF]    = 4;
  newtio->c_cc[VTIME]   = 0;
  newtio->c_cc[VMIN]    = 1;
  newtio->c_cc[VSWTC]   = 0;
  newtio->c_cc[VSTART]  = 0;
  newtio->c_cc[VSTOP]   = 0;
  newtio->c_cc[VSUSP]   = 0;
  newtio->c_cc[VEOL]    = 0;
  newtio->c_cc[VREPRINT]  = 0;
  newtio->c_cc[VDISCARD]  = 0;
  newtio->c_cc[VWERASE]   = 0;
  newtio->c_cc[VLNEXT]    = 0;
  newtio->c_cc[VEOL2]   = 0;

  tcflush(fd, TCIFLUSH);
  newtio->c_cc[VDISCARD]  = 0;
  tcsetattr(fd, TCSANOW, newtio);
  return fd;
}

void SERIAL::close(struct termios *oldtio)
{
  tcsetattr(fd, TCSANOW, oldtio);
}


bool SERIAL::purge()
{
  if (tcflush(fd, TCIOFLUSH) == -1)
  {
    return false;
  }
  return true;
}

int SERIAL::write_serial(char *data)
{
  int bytes_written = write(fd, data, strlen(data));
  return bytes_written;
}


int SERIAL::avail_serial()
{
  int result ;
  if (ioctl (fd, FIONREAD, &result) == -1)
    return -1 ;
  return result ;
}


int SERIAL::read_serial(char *received_buffer, int length)
{
  char *received_data = (char*)malloc(sizeof(char) * length);
  int bytes_read = read(fd, received_data, length);
  strcpy(received_buffer, received_data);
  return bytes_read;
}
