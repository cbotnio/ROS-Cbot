
#include "cbot_sensors/imu.hpp"


//*****************************************************************************
IMU_AHRS::IMU_AHRS(const char *com_port_path_string, int BAUD_RATE) : SERIAL(com_port_path_string, BAUD_RATE)
{
    strcpy(com_port_path, com_port_path_string);
    BAUDRATE = BAUD_RATE;
}

//#############################################################################S
int IMU_AHRS::checksum ( char *str ) {	// $GPRMC,,,,,,*4D\n
	unsigned char cs=0, sum=0, i=0;
	char           sCS[8];
	
	while ( 1 ) {
		if ( *(str+i) == '*' ) { sum=0; break; }
		if ( sum )				 cs ^= *(str+i);
		if ( *(str+i) == '$' )	 sum=1;
		i++;
	}
	sprintf ( sCS, "%02X", cs );
//	printf ( "%s\n", sCS );
	
	if ( memcmp ( sCS, str+i+1, 2 ) == 0 )		return 1;
	return 0;
}

//*****************************************************************************

// field separator ','
// $GPRMC,064723.000,A,1527.3288,N,07348.1953,E,0.00,270.46,250319,,,A*67
// $VNYMR,-086.098,+000.284,+017.943,+00.0177,+00.2612,-00.0128,+00.056,-03.021,-09.325,+00.000895,-00.001988,+00.001197*60
// 
char *IMU_AHRS::get_field ( char *str, int fld ){
	static char t0[512];
	int  i=0, j=0, k=0;
	
	
	while ( i < 512 ) {
		if ( *(str+i) == ',' || *(str+i) == '\n' ) {	// detect fields...
			if ( fld == j )		return t0;
			j++;
			t0[0]=0;
			k=0;
		}
		else {
			t0[k++] = *(str+i);
			t0[k]   = 0;
		}
		i++;
	}
	t0[0] = 0;
	return t0;
}

//*****************************************************************************
// Calculates the 16-bit CRC for the given ASCII or binary message.
unsigned short IMU_AHRS::calculateCRC(unsigned char data[], unsigned int length) {
	unsigned int i;
	unsigned short crc = 0;

	for ( i=0; i<length; i++ ) {
		crc  = (unsigned char)(crc >> 8) | (crc << 8);
		crc ^= data[i];
		crc ^= (unsigned char)(crc & 0xff) >> 4;
		crc ^= crc << 12;
		crc ^= (crc & 0x00ff) << 5;
	}
	
	return crc;
}

//*****************************************************************************
// voodoo code, dont touch.......
//
int IMU_AHRS::calCRC ( unsigned char buf[] ) {
	unsigned char    j=1, x=1;
	unsigned int     i, k=0, n=1, m=0;
	unsigned short   crc = 0;
	char             cs[16];
	
	int            g1[16]={8, 8, 8,12,16,12,24,12,12,24,20,28,2,4,8	},
	               g2[16]={8, 8, 8, 2, 8, 8, 8, 4, 4, 1, 0, 0,0,0,0},
	               g3[16]={2,12,12,12, 4, 4,16,12,12,12,12, 0,0,0,0},
	               g5[16]={2,12,16,36,12,12,12,12,12, 0, 0, 0,0,0,0};

	if ( buf[0] == 0xFA ) {
		for ( i=0; i<8; i++ ) {		// calculate grp bytes
			if ( buf[1] & j ) {		// 
				if ( i+1 == 1 ) {
					for (k=0,x=1;k<8;k++) {
						if (buf[n+1] & x )	{	m += g1[k];   }
						if (buf[n+2] & x)	{	m += g1[k+8]; }
						x = x << 1;
					}
					x=1;
				}

				if ( i+1 == 3 ) {
					for (k=0;k<8;k++) {
						if (buf[n+1] & x)		{	m += g3[k];   }
						if (buf[n+2] & x)		{	m += g3[k+8]; }
						x = x << 1;
					}
					x=1;
				}
				if ( i+1 == 5 ) {
					for (k=0;k<8;k++) {
						if (buf[n+1] & x)		{	m += g5[k];	  }
						if (buf[n+2] & x)		{	m += g5[k+8]; }
						x = x << 1;
					}
					x=1;
				}
				n += 2;		// from grp_byte
			}
			j = j << 1;
		}
	//	printf ("\n\nn: %d, m:%d\n", n,m); fflush(NULL);
		n += m;
		for ( i=0; i<=n; i++ ) {
			crc  = (unsigned char)(crc >> 8) | (crc << 8);
			crc ^= buf[i];
			crc ^= (unsigned char)(crc & 0xff) >> 4;
			crc ^= crc << 12;
			crc ^= (crc & 0x00ff) << 5;
		}
		sprintf( cs, "%04X, %02X%02X", crc, buf[n+1]&0xFF, buf[n+2]&0xFF );
		// ROS_INFO("CS: %s", cs);
		fflush(NULL);
		if ( memcmp ( cs, cs+6, 4 ) == 0 )	return 1;
	}
	return 0;
}

//*****************************************************************************************************************
cbot_ros_msgs::AHRS IMU_AHRS::tokenizeAHRSData(Nvsdata *dptr) 
{
    cbot_ros_msgs::AHRS temp;
    temp.Roll = (float)(dptr->roll);
    temp.Pitch = -(float)(dptr->pitch) ;
    temp.YawAngle = -(float)(dptr->yaw) + 90.0;

    temp.RollRate = (float)(dptr->ucgX) ;
    temp.PitchRate = (float)(dptr->ucgY) ;
    temp.YawRate = -(float)(dptr->ucgZ) ;

    temp.Xaccel = (float)(dptr->laccelY) ;
    temp.Yaccel = (float)(dptr->laccelX) ;
    temp.Zaccel = -(float)(dptr->laccelZ) ;

    temp.Temp = (dptr->temp);
    temp.AHRS_Status = AHRS_GOOD;

    return temp;
}

//*****************************************************************************************************************

// FA 01 08 00 AE 23 1D C3 2D B1 17 C0 01 BA BC 40 48 48
// FA 01 08 00 08 22 1D C3 1D C3 17 C0 82 C6 BC 40 16 ED 
// FA 01 08 00 9D 21 1D C3 DB 3E 18 C0 BC 8F BC 40 7B C3 
// 
cbot_ros_msgs::AHRS IMU_AHRS::read_ahrs_bin ( void ) {
//	char                 buf[256]={0xFA,0x01,0x08,0x00,0xAE,0x23,0x1D,0xC3,0x2D,0xB1,0x17,0xC0,0x01,0xBA,0xBC,0x40,0x48,0x48}
	cbot_ros_msgs::AHRS temp;
	unsigned char        buf[256], cs[32];
	static char          dbuf[512];
	int                  res=0, i, sync=0;
	static unsigned int  cnt=0, wp=0;
	unsigned short       dm=0;
	float *ptr;

	Nvsdata *dptr;
	
	res = read(fd, buf, 250);
	if ( res ) {
		fflush ( NULL );
	}
	for ( i=0; i<res; i++ ) {
		dbuf[wp++] = buf[i];
		if ( wp >= 512 )		wp=0;
 	}
 	
 	if(calCRC(buf )==1){
		dptr = (Nvsdata *)(buf+8);
		fflush(stdout);
	 	wp=0;
	 	temp = tokenizeAHRSData(dptr);
	}
	return temp;
}
//*****************************************************************************