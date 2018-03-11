#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdbool.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <inttypes.h>


#include "pi_cc3d_drone/accelsensor.h"
#include "uavobj_msg.h"
#include "pi_cc3d_drone/flightstatus.h"

#define byte uint8_t

byte _pBuf[265];  //internal packetbuffer for receiving a generic packet    
byte _crc(int len);  //calc crc over _pBuf with length le
byte _crc(byte* header, byte* data, int len);  //calc crc over header and data array; expects 10 byte header and len byte data array pointers
int fd;


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

int wiringSetup(){
	

  if ((fd = serialOpen ("/dev/serial0", 57600)) < 0)
  {
    fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
    return 1 ;
  }

  if (wiringPiSetup () == -1)
  {
    fprintf (stdout, "Unable to start wiringPi: %s\n", strerror (errno)) ;
    return 1 ;
}
}

void request(unsigned long objId) {
	wiringSetup();
  _pBuf[0] = 0x3c; //sync
  _pBuf[1] = 0x21; //msgtype (0x21 = request)
  _pBuf[2] = 0x0a; //len
  _pBuf[3] = 0x00; //len
  _pBuf[4] = (objId >> (8*3)) & 0xff; 
	printf("%x", _pBuf[4]);
  _pBuf[5] = (objId >> (8*2)) & 0xff;
	printf("%x", _pBuf[5]);
  _pBuf[6] = (objId >> (8*1)) & 0xff;
	printf("%x", _pBuf[6]);
  _pBuf[7] = (objId >> (8*0)) & 0xff;
	printf("%x", _pBuf[7]);
  _pBuf[8] = 0x00; //iid
  _pBuf[9] = 0x00; //iid
  _pBuf[10] = _crc(10); //crc

  printf("i am about to send the data \n");
  serialPuts (fd, reinterpret_cast<const char*>(_pBuf));
}

void send(unsigned long objId, byte* data, int length) {
  byte header[10];
  header[0] = 0x3c; //sync
  header[1] = 0x20; //msgtype (0x20 = send)
  
  int pLen  = length + 10;  
  header[2] = (pLen >> (8*0)) & 0xff;
  header[3] = (pLen >> (8*1)) & 0xff;
  
  header[4] = (objId >> (8*0)) & 0xff; 
  header[5] = (objId >> (8*1)) & 0xff;
  header[6] = (objId >> (8*2)) & 0xff;
  header[7] = (objId >> (8*3)) & 0xff;
  header[8] = 0x00; //iid
  header[9] = 0x00; //iid

  byte crc = _crc(header, data, length);

  // serialPuts(header, 10);
  // serialPutchar(data, length);
  // serialPutchar(crc);   
}

//bool receive(unsigned long objId, byte *ret, unsigned int timeout) {
 
bool receive(){
	wiringSetup();
	byte rec[10];	
	unsigned long objId = FLIGHTSTATUS_OBJID;
	byte *ret = FlightStatusDataUnion.arr;
 	byte id[3];
	printf("\n");
	id[0] = (objId >> (8*0)) & 0xff; 
        printf("%x", id[0]);
  	id[1] = (objId >> (8*1)) & 0xff;
        printf("%x", id[1]);
  	id[2] = (objId >> (8*2)) & 0xff;
        printf("%x", id[2]);
  	id[3] = (objId >> (8*3)) & 0xff;
        printf("%x", id[3]);
	printf("\n %x", objId);
	bool loop = 1;
	unsigned long time = millis();
 
  while(loop) {
    if (serialDataAvail(fd)) {
		
	unsigned int timeout=2000;
	loop = millis() <= time + timeout;  //loop as long as start + timeout lower than current time
    
      _pBuf[0] = serialGetchar(fd);;
      if (rec[0] == 0){
        //NOOP
      } else if(rec[0] != 0x3c) {
        //Serial.print(sync, HEX);
        //printf(" Bad Byte ");
      } else {
        printf(" in the main loop \n");        
        rec[1] = serialGetchar(fd); //msgtype
        printf("sync value = %X \n",rec[0]);
        rec[2] = serialGetchar(fd);
		 printf("msg type = %X \n",rec[1]);
		  printf("length = %X",rec[2]);
        rec[3] = serialGetchar(fd);
         printf("%X \n",rec[3]);
        unsigned int len = (unsigned int) rec[2] | (unsigned int) rec[3] << 8;
        
        if(len < 10 || len > 265) {
          
        } else {
            printf("\n");
            unsigned long oid0 = serialGetchar(fd);
            rec[4] = oid0;
            printf("%X \n", rec[4]);
            unsigned long oid1 = serialGetchar(fd);
            rec[5] = oid1;
            printf("%X \n",rec[5]);
            unsigned long oid2 = serialGetchar(fd);
            rec[6] = oid2;
            printf("%X \n", rec[6]);
            unsigned long oid3 = serialGetchar(fd);
            rec[7] = oid3;
            printf("%X \n",rec[7]);
            unsigned long oid = oid0 + oid1 * 256 + oid2 * 65536 + oid3 * 16777216;
		    printf("\n");
		    printf("unsigned long: ");
		    printf("%lu", oid);
		    printf("\n");
		    printf("hex value from bit stream: ");
		    printf("%X", oid);
		    printf("\n");
		    printf("");
		    printf("\n just before objID this is header file hex value: ");
			printf("%X", objId);
	printf("%X,", FLIGHTSTATUS_OBJID);
          if(oid != objId) {
		printf("values aint equal");
          } else {
		printf("values are equal g");
            loop = 0;
               
            int iid0 = serialGetchar(fd);
            _pBuf[8] = iid0;
            int iid1 = serialGetchar(fd);
            _pBuf[9] = iid1;
			
    
            unsigned int iid = iid0 + iid1 * 256;             
    
            int datalen = len - 10;;
  
             int j;          
             for (j = 10; j < len; j++) {
               _pBuf[j] = serialGetchar(fd);
               ret[j-10] = _pBuf[j];
			  // printf("putting values in the union struct");
             }
    
			for(int i=0;i<sizeof(ret);i++){
			printf("%X",ret[i]);
			}
	
            byte crc = serialGetchar(fd);

            byte ccrc = _crc(len);
			 
            return crc == ccrc;    
          }        
        }    
      }    
    } else {
      loop = 0;
    }
  }
  return false;
}

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


int main(void){
//while(1){
	request(FLIGHTSTATUS_OBJID);
	receive();
//	printf("%x ", _pBuf[7]);
//	printf("%x ", _pBuf[6]);
//	printf("%x ", _pBuf[5]);
//	printf("%x ", _pBuf[4]);


	
//}
}
