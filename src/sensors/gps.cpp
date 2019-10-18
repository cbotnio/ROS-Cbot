#include "asv_ros_revamp/sensors/gps.hpp"

GPS::GPS(const char *com_port_path_string, int BAUD_RATE) : SERIAL(com_port_path_string, BAUD_RATE)
{
  strcpy(com_port_path, com_port_path_string);
  BAUDRATE = BAUD_RATE;
}

unsigned char GPS::get_hex(char* value)
{
  char hb, lb;
  unsigned char sum1;
  if (*value >= 48 && *value < 65)
    hb = (*value - 48) << 4;
  else
    hb = (*value - 55) << 4;

  if (*(value + 1) >= 48 && *(value + 1) < 65)
    lb = *(value + 1) - 48;
  else
    lb = (*(value + 1) - 55);

  sum1 = hb | lb;
  return (sum1);
}

int GPS::checksum(char* buf, int res, char*(chsum)) 
{
  int i = 0;
  unsigned char val = 0x00, chksum;
  chksum = get_hex(chsum);

  for (i = 1; i < res - strlen(chsum); i++) val ^= buf[i];

  if ( chksum == val)
    return 1;
  else
    return 0;
}

asv_ros_revamp::GPS GPS::decode()
{
  int res, i, j, len, temp_var;
  double Lat, Lon; 
  char Zone[20];
  char buf[200], data[20][10], *ptr[20], *chsum; 
  float templat, templon;
  char *ptrtmp;
  asv_ros_revamp::GPS temp;

  for (i = 0; i < 20; i++) for (j = 0; j < 10; j++) data[i][j] = 0;
  res = read(fd, buf, 150);
  buf[res] = 0;
  chsum = strstr(buf, "*");
  if ((chsum) && (res > 10))
  {
    if (checksum(buf, res - 1, chsum + 1))
    {
      if (strncmp(buf, "$GPRMC", 6) == 0)
      {
        i = 1;
        ptrtmp = strtok(buf, ",");
        while (ptrtmp != NULL)
        {
          strcpy(data[i], ptrtmp);
          i = i + 1;
          ptrtmp = strtok(NULL, ",");
        }

        templat = atof(data[LaT]); 
        templon = atof(data[LoN]);
        temp.lat_deg = templat / 100;
        temp.lon_deg = templon / 100;
        temp.lat_min = templat - temp.lat_deg * 100.0;
        temp.lon_min = templon - temp.lon_deg * 100.0;
        temp.lat_dir = data[DiRN_LaT][0];
        temp.lon_dir = data[DiRN_LoN][0];
        temp.vel = ((atof(data[SpD])) * 0.514444); //knots to meters conversion (0.514444);
        temp.course = atof(data[TrACK]);

        temp.hour = (atoi(data[UtCTIME])) / 10000;
        temp.min = ((atoi(data[UtCTIME])) - (temp.hour * 10000)) / 100;
        temp_var = (((atoi(data[UtCTIME])) / 100) * 100);
        temp.sec = (atoi(data[UtCTIME])) - temp_var;
        temp.day = (atoi(data[UtCDATE])) / 10000;
        temp.month = ((atoi(data[UtCDATE])) - (temp.day * 10000)) / 100;
        temp_var = (((atoi(data[UtCDATE])) / 100) * 100);
        temp.year = (atoi(data[UtCDATE])) - temp_var;

        //Read Valid GPS Lattitude and Longitude.
        if (!strcmp(data[StAT], "A"))
        {
          Lat = temp.lat_deg + temp.lat_min / 60.0;
          Lon = temp.lon_deg + temp.lon_min / 60.0;
          Lat = (temp.lat_dir == 'N') ? Lat : -Lat;
          Lon = (temp.lon_dir == 'E') ? Lon : -Lon;
          temp.latitude = Lat;
          temp.longitude = Lon;
          temp.GPS_status = GPS_FIX_YES;
        }
        else if (!strcmp(data[StAT], "V"))
        {
          temp.GPS_status = GPS_FIX_NO;
        }
        return temp;
      }
    }
    else
    {
      ROS_ERROR("GPS Checksum error");
      temp.GPS_status = GPS_FAIL;
      return temp;
    }
  }
  else  
  {
    ROS_ERROR("GPS reset");
    temp.GPS_status = GPS_FAIL;
    return temp;
  }
}