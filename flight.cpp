#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <inttypes.h>
#include "pi_cc3d_drone/flightstatus.h"

#define error_message printf
#define byte uint8_t
byte _pBuf[256];
byte _crc(int len);  //calc crc over _pBuf with length le
byte _crc(byte* header, byte* data, int len);  //calc crc over header and data array; expects 10 byte header and len byte data array pointers

byte _CRC_TABLE[] = {
        0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15, 0x38, 0x3f, 0x36, 0x31, 0x24, 0x23, 0x2a, 0x2d,
        0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65, 0x48, 0x4f, 0x46, 0x41, 0x54, 0x53, 0x5a, 0x5d,
        0xe0, 0xe7, 0xee, 0xe9, 0xfc, 0xfb, 0xf2, 0xf5, 0xd8, 0xdf, 0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd,
        0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85, 0xa8, 0xaf, 0xa6, 0xa1, 0xb4, 0xb3, 0xba, 0xbd,
        0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2, 0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea,
        0xb7, 0xb0, 0xb9, 0xbe, 0xab, 0xac, 0xa5, 0xa2, 0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a,
        0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32, 0x1f, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0d, 0x0a,
        0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42, 0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a,
        0x89, 0x8e, 0x87, 0x80, 0x95, 0x92, 0x9b, 0x9c, 0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4,
        0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec, 0xc1, 0xc6, 0xcf, 0xc8, 0xdd, 0xda, 0xd3, 0xd4,
        0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c, 0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44,
        0x19, 0x1e, 0x17, 0x10, 0x05, 0x02, 0x0b, 0x0c, 0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34,
        0x4e, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5c, 0x5b, 0x76, 0x71, 0x78, 0x7f, 0x6a, 0x6d, 0x64, 0x63,
        0x3e, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b, 0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13,
        0xae, 0xa9, 0xa0, 0xa7, 0xb2, 0xb5, 0xbc, 0xbb, 0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83,
        0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb, 0xe6, 0xe1, 0xe8, 0xef, 0xfa, 0xfd, 0xf4, 0xf3
};

byte _crc(int len) {
  byte ccrc = 0;
  for (unsigned int k = 0; k < len; k++) {
      ccrc = _CRC_TABLE[((byte) (ccrc ^ _pBuf[k])) & 0xFF];
  }
  return (ccrc & 0xFF);
}


byte _crc(byte* header, byte* data, int len) {
  byte ccrc = 0;
  for (unsigned int k = 0; k < 10; k++) {
      ccrc = _CRC_TABLE[((byte) (ccrc ^ header[k])) & 0xFF];
  }
  for (unsigned int k = 0; k < len; k++) {
      ccrc = _CRC_TABLE[((byte) (ccrc ^ data[k])) & 0xFF];
  }
  return (ccrc & 0xFF);
}


int set_interface_attribs (int fd, int speed, int parity)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                error_message ("error %d from tcgetattr", errno);
                return -1;
        }

        cfsetospeed (&tty, speed);
        cfsetispeed (&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
        tty.c_iflag &= ~IGNBRK;         // disable break processing
        tty.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        tty.c_oflag = 0;                // no remapping, no delays
        tty.c_cc[VMIN]  = 0;            // read doesn't block
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                        // enable reading
        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        tty.c_cflag |= parity;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        {
                error_message ("error %d from tcsetattr", errno);
                return -1;
        }
        return 0;
}

void set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                error_message ("error %d from tggetattr", errno);
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
                error_message ("error %d setting term attributes", errno);
}







int main(){
char *portname = "/dev/serial0";
int fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
set_interface_attribs (fd, B57600, 0);  // set speed to 57,600 bps, 8n1 (no parity)
set_blocking (fd, 0);                // set no blocking
byte *ret = FlightStatusDataUnion.arr;
if (fd < 0)
{
        error_message ("error %d opening %s: %s", errno, portname, strerror (errno));
        return 1;
}
unsigned long objId = FLIGHTSTATUS_OBJID;
  _pBuf[0] = 0x3c; //sync
  _pBuf[1] = 0x21; //msgtype (0x21 = request)
  _pBuf[2] = 0x0a; //len
  _pBuf[3] = 0x00; //len
  _pBuf[4] = (objId >> (8*0)) & 0xff; 
//	printf("%x", _pBuf[4]);
  _pBuf[5] = (objId >> (8*1)) & 0xff;
//	printf("%x", _pBuf[5]);
  _pBuf[6] = (objId >> (8*2)) & 0xff;
//	printf("%x", _pBuf[6]);
  _pBuf[7] = (objId >> (8*3)) & 0xff;
//	printf("%x", _pBuf[7]);
  _pBuf[8] = 0x00; //iid
  _pBuf[9] = 0x00; //iid
_pBuf[10] = _crc(10); //crc

write (fd, _pBuf, 11);           // send 7 character greeting
usleep ((11 + 25) * 100);             // sleep enough to transmit the 7 plus
                                     // receive 25:  approx 100 uS per char transmit

char buf [100], rec[100];
int n = read (fd, buf, sizeof buf); 

printf("heres the data\n");
for(int t=0;t<sizeof(buf);t++){
	printf("%x",buf[t]);
}
if(buf[0]==0x3c){
	printf("\nsync value");
	printf("\nmsgtype :%x", buf[1]);
	printf("\nlength :%x", buf[2]);
	printf("\nlength2 :%x", buf[3]);
	printf("\nobject id :%x", buf[4]);
	printf("\nobject id  :%x", buf[5]);
	printf("\nobject id  :%x", buf[6]);
	printf("\nobject id  :%x", buf[7]);

	 unsigned int len = (unsigned int) buf[2] | (unsigned int) buf[3] << 8;
             int j;          
	     int datalen = len - 10;;
             for (j = 10; j < len; j++) {
               ret[j-10] = buf[j];
	//		   printf("putting values in the union struct");
             }
    			printf("print union vals");
			for(int i=0;i<sizeof(ret);i++){
			printf("%X",ret[i]);
}

		printf("\n%x", FlightStatusDataUnion.data.Armed);
		printf("\n%d",FlightStatusDataUnion.data.Armed); 
		printf("\n%d",FlightStatusDataUnion.data.FlightMode);
 
}
	
/*
*/
}
