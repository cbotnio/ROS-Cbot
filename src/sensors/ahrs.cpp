#include "asv_ros_revamp/sensors/ahrs.hpp"

AHRS::AHRS(const char *com_port_path_string, int BAUD_RATE) : SERIAL(com_port_path_string, BAUD_RATE)
{
  strcpy(com_port_path, com_port_path_string);
  BAUDRATE = BAUD_RATE;
}

asv_ros_revamp::AHRS AHRS::check_ahrs(unsigned char *temp, int res) 
{
  int i;
  unsigned char chksum;
  asv_ros_revamp::AHRS temp_ahrs_msg;
  if (*temp == SYNC)  {
    if (*(temp + 29) == checksum_ahrs(temp, 28)) {
      temp_ahrs_msg = tokenize_AHRS_data(temp);
    }
  }
  return temp_ahrs_msg;
}

unsigned char AHRS::checksum_ahrs(unsigned char *buf, int res) 
{
  int i;
  unsigned char chsum = 0;
  for (i = 1; i < 29; i++) chsum += buf[i];
  chsum = chsum % 256;
  return chsum;
}

float AHRS::tofloat(unsigned char *buf)
{
  short s;
  s = ((*buf) * 256) + *(buf + 1);
  return ((float)s);
}


asv_ros_revamp::AHRS AHRS::decode()
{
	asv_ros_revamp::AHRS temp;
  unsigned char buf[128], chsum;
  int res, i;
  res = 0;
  for (i = 0; i < 30; i++) buf[i] = 0;
  res = read(fd, buf, 30);

  if (res == 30)  
    temp = check_ahrs(buf, res);
  else
    temp.AHRS_Status = AHRS_FAIL;
  
  buf[res] = 0;
  temp = tokenize_AHRS_data(buf);
  return temp;
}


void AHRS::write_ahrs() {
  tcflush(fd, TCIFLUSH);
  write(fd, "G", 1);
}

void AHRS::init_ahrs() {
  unsigned char *buf;
  int i;
  tcflush(fd, TCIFLUSH);
  tcflush(fd, TCOFLUSH);
  write(fd, "P", 1); 
  sleep(1);
  write(fd, "R", 1); 
  sleep(1);
  read(fd, buf, 5);
  write(fd, "a", 1); 
  sleep(1);
  read(fd, buf, 5);
  tcflush(fd, TCIFLUSH);
  tcflush(fd, TCOFLUSH);
  write(fd, "G", 1); 
  sleep(1);
}

asv_ros_revamp::AHRS AHRS::tokenize_AHRS_data(unsigned char *buf) 
{
  asv_ros_revamp::AHRS temp;
  temp.Roll = tofloat(&buf[1]) * (0.0054931640625);
  temp.Pitch = tofloat(&buf[3]) * (0.0054931640625);
  temp.YawAngle = tofloat(&buf[5]) * (0.0054931640625);

  temp.RollRate = tofloat(&buf[7]) * 0.004577637;
  temp.PitchRate = tofloat(&buf[9]) * 0.004577637;
  temp.YawRate = tofloat(&buf[11]) * 0.004577637;

  temp.Xaccel = tofloat(&buf[13]) * 0.0000915527;
  temp.Yaccel = tofloat(&buf[15]) * 0.0000915527;
  temp.Zaccel = tofloat(&buf[17]) * 0.0000915527;

  temp.Temp = ((buf[25]) * 256 + buf[26]) * 0.054248 - 61.105;
  temp.AHRS_Status = AHRS_GOOD;

  return temp;
}
