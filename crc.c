#include<crc.h>


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
