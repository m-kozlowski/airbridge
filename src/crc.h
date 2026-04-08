#pragma once
#include <stdint.h>
#include <stddef.h>

uint16_t crc16_ccitt(const uint8_t *data, size_t len, uint16_t crc = 0xFFFF);
uint8_t crc8_ccitt(const uint8_t *data, size_t len, uint8_t crc = 0x00);
