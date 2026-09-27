#include "headfile.h"


uint16_t CRC16_MCRF4XX(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;

  if ((data == NULL) && (length != 0U)) return 0U;

  while (length--) {
    crc ^= *data++;

    for (uint8_t i = 0; i < 8; i++) {
      if (crc & 0x0001) {
        crc = (crc >> 1) ^ 0x8408;
      }else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

bool CRC16_MCRF4XX_SelfTest(void) {
  static const uint8_t test_data[] = "123456789";
  return CRC16_MCRF4XX(test_data, sizeof(test_data) - 1U) == 0x6F91U;
}


