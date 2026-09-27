
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

uint16_t CRC16_MCRF4XX(const uint8_t *data, size_t length);
bool CRC16_MCRF4XX_SelfTest(void);


