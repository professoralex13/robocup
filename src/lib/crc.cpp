#include "lib/crc.hpp"

uint8_t calculate_crc(etl::span<uint8_t> p) {
    uint8_t crc = 0;
    size_t i;

    for (i = 0; i < p.size(); i++) {
        crc = CRC_TABLE[(crc ^ p[i]) & 0xff];
    }

    return crc;
}