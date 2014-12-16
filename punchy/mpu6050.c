#include "mpu6050.h"
#include "i2c.h"
#include "hc05.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include <legacymsp430.h>

// uncomment to use all the print statements.
//#define MPU6050_DEBUG

/* ================================================================================================ *
 | Default MotionApps v2.0 42-byte FIFO packet structure:                                           |
 |                                                                                                  |
 | [QUAT W][      ][QUAT X][      ][QUAT Y][      ][QUAT Z][      ][GYRO X][      ][GYRO Y][      ] |
 |   0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23  |
 |                                                                                                  |
 | [GYRO Z][      ][ACC X ][      ][ACC Y ][      ][ACC Z ][      ][      ]                         |
 |  24  25  26  27  28  29  30  31  32  33  34  35  36  37  38  39  40  41                          |
 * ================================================================================================ */

// this block of memory gets written to the MPU on start-up, and it seems
// to be volatile memory, so it has to be done each time (it only takes ~1
// second though)
const uint8_t dmpMemory[MPU6050_DMP_CODE_SIZE] = {
    // bank 0, 256 bytes
    0xFB, 0x00, 0x00, 0x3E, 0x00, 0x0B, 0x00, 0x36, 0x00, 0x01, 0x00, 0x02, 0x00, 0x03, 0x00, 0x00,
    0x00, 0x65, 0x00, 0x54, 0xFF, 0xEF, 0x00, 0x00, 0xFA, 0x80, 0x00, 0x0B, 0x12, 0x82, 0x00, 0x01,
    0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x28, 0x00, 0x00, 0xFF, 0xFF, 0x45, 0x81, 0xFF, 0xFF, 0xFA, 0x72, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x03, 0xE8, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x7F, 0xFF, 0xFF, 0xFE, 0x80, 0x01,
    0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x3E, 0x03, 0x30, 0x40, 0x00, 0x00, 0x00, 0x02, 0xCA, 0xE3, 0x09, 0x3E, 0x80, 0x00, 0x00,
    0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00,
    0x41, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x0B, 0x2A, 0x00, 0x00, 0x16, 0x55, 0x00, 0x00, 0x21, 0x82,
    0xFD, 0x87, 0x26, 0x50, 0xFD, 0x80, 0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00, 0x05, 0x80, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00,
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x6F, 0x00, 0x02, 0x65, 0x32, 0x00, 0x00, 0x5E, 0xC0,
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xFB, 0x8C, 0x6F, 0x5D, 0xFD, 0x5D, 0x08, 0xD9, 0x00, 0x7C, 0x73, 0x3B, 0x00, 0x6C, 0x12, 0xCC,
    0x32, 0x00, 0x13, 0x9D, 0x32, 0x00, 0xD0, 0xD6, 0x32, 0x00, 0x08, 0x00, 0x40, 0x00, 0x01, 0xF4,
    0xFF, 0xE6, 0x80, 0x79, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0xD0, 0xD6, 0x00, 0x00, 0x27, 0x10,

    // bank 1, 256 bytes
    0xFB, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00,
    0x00, 0x00, 0xFA, 0x36, 0xFF, 0xBC, 0x30, 0x8E, 0x00, 0x05, 0xFB, 0xF0, 0xFF, 0xD9, 0x5B, 0xC8,
    0xFF, 0xD0, 0x9A, 0xBE, 0x00, 0x00, 0x10, 0xA9, 0xFF, 0xF4, 0x1E, 0xB2, 0x00, 0xCE, 0xBB, 0xF7,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x04, 0x00, 0x02, 0x00, 0x02, 0x02, 0x00, 0x00, 0x0C,
    0xFF, 0xC2, 0x80, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0xCF, 0x80, 0x00, 0x40, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x14,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x03, 0x3F, 0x68, 0xB6, 0x79, 0x35, 0x28, 0xBC, 0xC6, 0x7E, 0xD1, 0x6C,
    0x80, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB2, 0x6A, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xF0, 0x00, 0x00, 0x00, 0x30,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x25, 0x4D, 0x00, 0x2F, 0x70, 0x6D, 0x00, 0x00, 0x05, 0xAE, 0x00, 0x0C, 0x02, 0xD0,

    // bank 2, 256 bytes
    0x00, 0x00, 0x00, 0x00, 0x00, 0x65, 0x00, 0x54, 0xFF, 0xEF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x01, 0x00, 0x00, 0x44, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x01, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x65, 0x00, 0x00, 0x00, 0x54, 0x00, 0x00, 0xFF, 0xEF, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00,
    0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    // bank 3, 256 bytes
    0xD8, 0xDC, 0xBA, 0xA2, 0xF1, 0xDE, 0xB2, 0xB8, 0xB4, 0xA8, 0x81, 0x91, 0xF7, 0x4A, 0x90, 0x7F,
    0x91, 0x6A, 0xF3, 0xF9, 0xDB, 0xA8, 0xF9, 0xB0, 0xBA, 0xA0, 0x80, 0xF2, 0xCE, 0x81, 0xF3, 0xC2,
    0xF1, 0xC1, 0xF2, 0xC3, 0xF3, 0xCC, 0xA2, 0xB2, 0x80, 0xF1, 0xC6, 0xD8, 0x80, 0xBA, 0xA7, 0xDF,
    0xDF, 0xDF, 0xF2, 0xA7, 0xC3, 0xCB, 0xC5, 0xB6, 0xF0, 0x87, 0xA2, 0x94, 0x24, 0x48, 0x70, 0x3C,
    0x95, 0x40, 0x68, 0x34, 0x58, 0x9B, 0x78, 0xA2, 0xF1, 0x83, 0x92, 0x2D, 0x55, 0x7D, 0xD8, 0xB1,
    0xB4, 0xB8, 0xA1, 0xD0, 0x91, 0x80, 0xF2, 0x70, 0xF3, 0x70, 0xF2, 0x7C, 0x80, 0xA8, 0xF1, 0x01,
    0xB0, 0x98, 0x87, 0xD9, 0x43, 0xD8, 0x86, 0xC9, 0x88, 0xBA, 0xA1, 0xF2, 0x0E, 0xB8, 0x97, 0x80,
    0xF1, 0xA9, 0xDF, 0xDF, 0xDF, 0xAA, 0xDF, 0xDF, 0xDF, 0xF2, 0xAA, 0xC5, 0xCD, 0xC7, 0xA9, 0x0C,
    0xC9, 0x2C, 0x97, 0x97, 0x97, 0x97, 0xF1, 0xA9, 0x89, 0x26, 0x46, 0x66, 0xB0, 0xB4, 0xBA, 0x80,
    0xAC, 0xDE, 0xF2, 0xCA, 0xF1, 0xB2, 0x8C, 0x02, 0xA9, 0xB6, 0x98, 0x00, 0x89, 0x0E, 0x16, 0x1E,
    0xB8, 0xA9, 0xB4, 0x99, 0x2C, 0x54, 0x7C, 0xB0, 0x8A, 0xA8, 0x96, 0x36, 0x56, 0x76, 0xF1, 0xB9,
    0xAF, 0xB4, 0xB0, 0x83, 0xC0, 0xB8, 0xA8, 0x97, 0x11, 0xB1, 0x8F, 0x98, 0xB9, 0xAF, 0xF0, 0x24,
    0x08, 0x44, 0x10, 0x64, 0x18, 0xF1, 0xA3, 0x29, 0x55, 0x7D, 0xAF, 0x83, 0xB5, 0x93, 0xAF, 0xF0,
    0x00, 0x28, 0x50, 0xF1, 0xA3, 0x86, 0x9F, 0x61, 0xA6, 0xDA, 0xDE, 0xDF, 0xD9, 0xFA, 0xA3, 0x86,
    0x96, 0xDB, 0x31, 0xA6, 0xD9, 0xF8, 0xDF, 0xBA, 0xA6, 0x8F, 0xC2, 0xC5, 0xC7, 0xB2, 0x8C, 0xC1,
    0xB8, 0xA2, 0xDF, 0xDF, 0xDF, 0xA3, 0xDF, 0xDF, 0xDF, 0xD8, 0xD8, 0xF1, 0xB8, 0xA8, 0xB2, 0x86,

    // bank 4, 256 bytes
    0xB4, 0x98, 0x0D, 0x35, 0x5D, 0xB8, 0xAA, 0x98, 0xB0, 0x87, 0x2D, 0x35, 0x3D, 0xB2, 0xB6, 0xBA,
    0xAF, 0x8C, 0x96, 0x19, 0x8F, 0x9F, 0xA7, 0x0E, 0x16, 0x1E, 0xB4, 0x9A, 0xB8, 0xAA, 0x87, 0x2C,
    0x54, 0x7C, 0xB9, 0xA3, 0xDE, 0xDF, 0xDF, 0xA3, 0xB1, 0x80, 0xF2, 0xC4, 0xCD, 0xC9, 0xF1, 0xB8,
    0xA9, 0xB4, 0x99, 0x83, 0x0D, 0x35, 0x5D, 0x89, 0xB9, 0xA3, 0x2D, 0x55, 0x7D, 0xB5, 0x93, 0xA3,
    0x0E, 0x16, 0x1E, 0xA9, 0x2C, 0x54, 0x7C, 0xB8, 0xB4, 0xB0, 0xF1, 0x97, 0x83, 0xA8, 0x11, 0x84,
    0xA5, 0x09, 0x98, 0xA3, 0x83, 0xF0, 0xDA, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0xD8, 0xF1, 0xA5,
    0x29, 0x55, 0x7D, 0xA5, 0x85, 0x95, 0x02, 0x1A, 0x2E, 0x3A, 0x56, 0x5A, 0x40, 0x48, 0xF9, 0xF3,
    0xA3, 0xD9, 0xF8, 0xF0, 0x98, 0x83, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0x97, 0x82, 0xA8, 0xF1,
    0x11, 0xF0, 0x98, 0xA2, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0xDA, 0xF3, 0xDE, 0xD8, 0x83, 0xA5,
    0x94, 0x01, 0xD9, 0xA3, 0x02, 0xF1, 0xA2, 0xC3, 0xC5, 0xC7, 0xD8, 0xF1, 0x84, 0x92, 0xA2, 0x4D,
    0xDA, 0x2A, 0xD8, 0x48, 0x69, 0xD9, 0x2A, 0xD8, 0x68, 0x55, 0xDA, 0x32, 0xD8, 0x50, 0x71, 0xD9,
    0x32, 0xD8, 0x70, 0x5D, 0xDA, 0x3A, 0xD8, 0x58, 0x79, 0xD9, 0x3A, 0xD8, 0x78, 0x93, 0xA3, 0x4D,
    0xDA, 0x2A, 0xD8, 0x48, 0x69, 0xD9, 0x2A, 0xD8, 0x68, 0x55, 0xDA, 0x32, 0xD8, 0x50, 0x71, 0xD9,
    0x32, 0xD8, 0x70, 0x5D, 0xDA, 0x3A, 0xD8, 0x58, 0x79, 0xD9, 0x3A, 0xD8, 0x78, 0xA8, 0x8A, 0x9A,
    0xF0, 0x28, 0x50, 0x78, 0x9E, 0xF3, 0x88, 0x18, 0xF1, 0x9F, 0x1D, 0x98, 0xA8, 0xD9, 0x08, 0xD8,
    0xC8, 0x9F, 0x12, 0x9E, 0xF3, 0x15, 0xA8, 0xDA, 0x12, 0x10, 0xD8, 0xF1, 0xAF, 0xC8, 0x97, 0x87,

    // bank 5, 256 bytes
    0x34, 0xB5, 0xB9, 0x94, 0xA4, 0x21, 0xF3, 0xD9, 0x22, 0xD8, 0xF2, 0x2D, 0xF3, 0xD9, 0x2A, 0xD8,
    0xF2, 0x35, 0xF3, 0xD9, 0x32, 0xD8, 0x81, 0xA4, 0x60, 0x60, 0x61, 0xD9, 0x61, 0xD8, 0x6C, 0x68,
    0x69, 0xD9, 0x69, 0xD8, 0x74, 0x70, 0x71, 0xD9, 0x71, 0xD8, 0xB1, 0xA3, 0x84, 0x19, 0x3D, 0x5D,
    0xA3, 0x83, 0x1A, 0x3E, 0x5E, 0x93, 0x10, 0x30, 0x81, 0x10, 0x11, 0xB8, 0xB0, 0xAF, 0x8F, 0x94,
    0xF2, 0xDA, 0x3E, 0xD8, 0xB4, 0x9A, 0xA8, 0x87, 0x29, 0xDA, 0xF8, 0xD8, 0x87, 0x9A, 0x35, 0xDA,
    0xF8, 0xD8, 0x87, 0x9A, 0x3D, 0xDA, 0xF8, 0xD8, 0xB1, 0xB9, 0xA4, 0x98, 0x85, 0x02, 0x2E, 0x56,
    0xA5, 0x81, 0x00, 0x0C, 0x14, 0xA3, 0x97, 0xB0, 0x8A, 0xF1, 0x2D, 0xD9, 0x28, 0xD8, 0x4D, 0xD9,
    0x48, 0xD8, 0x6D, 0xD9, 0x68, 0xD8, 0xB1, 0x84, 0x0D, 0xDA, 0x0E, 0xD8, 0xA3, 0x29, 0x83, 0xDA,
    0x2C, 0x0E, 0xD8, 0xA3, 0x84, 0x49, 0x83, 0xDA, 0x2C, 0x4C, 0x0E, 0xD8, 0xB8, 0xB0, 0xA8, 0x8A,
    0x9A, 0xF5, 0x20, 0xAA, 0xDA, 0xDF, 0xD8, 0xA8, 0x40, 0xAA, 0xD0, 0xDA, 0xDE, 0xD8, 0xA8, 0x60,
    0xAA, 0xDA, 0xD0, 0xDF, 0xD8, 0xF1, 0x97, 0x86, 0xA8, 0x31, 0x9B, 0x06, 0x99, 0x07, 0xAB, 0x97,
    0x28, 0x88, 0x9B, 0xF0, 0x0C, 0x20, 0x14, 0x40, 0xB8, 0xB0, 0xB4, 0xA8, 0x8C, 0x9C, 0xF0, 0x04,
    0x28, 0x51, 0x79, 0x1D, 0x30, 0x14, 0x38, 0xB2, 0x82, 0xAB, 0xD0, 0x98, 0x2C, 0x50, 0x50, 0x78,
    0x78, 0x9B, 0xF1, 0x1A, 0xB0, 0xF0, 0x8A, 0x9C, 0xA8, 0x29, 0x51, 0x79, 0x8B, 0x29, 0x51, 0x79,
    0x8A, 0x24, 0x70, 0x59, 0x8B, 0x20, 0x58, 0x71, 0x8A, 0x44, 0x69, 0x38, 0x8B, 0x39, 0x40, 0x68,
    0x8A, 0x64, 0x48, 0x31, 0x8B, 0x30, 0x49, 0x60, 0xA5, 0x88, 0x20, 0x09, 0x71, 0x58, 0x44, 0x68,

    // bank 6, 256 bytes
    0x11, 0x39, 0x64, 0x49, 0x30, 0x19, 0xF1, 0xAC, 0x00, 0x2C, 0x54, 0x7C, 0xF0, 0x8C, 0xA8, 0x04,
    0x28, 0x50, 0x78, 0xF1, 0x88, 0x97, 0x26, 0xA8, 0x59, 0x98, 0xAC, 0x8C, 0x02, 0x26, 0x46, 0x66,
    0xF0, 0x89, 0x9C, 0xA8, 0x29, 0x51, 0x79, 0x24, 0x70, 0x59, 0x44, 0x69, 0x38, 0x64, 0x48, 0x31,
    0xA9, 0x88, 0x09, 0x20, 0x59, 0x70, 0xAB, 0x11, 0x38, 0x40, 0x69, 0xA8, 0x19, 0x31, 0x48, 0x60,
    0x8C, 0xA8, 0x3C, 0x41, 0x5C, 0x20, 0x7C, 0x00, 0xF1, 0x87, 0x98, 0x19, 0x86, 0xA8, 0x6E, 0x76,
    0x7E, 0xA9, 0x99, 0x88, 0x2D, 0x55, 0x7D, 0x9E, 0xB9, 0xA3, 0x8A, 0x22, 0x8A, 0x6E, 0x8A, 0x56,
    0x8A, 0x5E, 0x9F, 0xB1, 0x83, 0x06, 0x26, 0x46, 0x66, 0x0E, 0x2E, 0x4E, 0x6E, 0x9D, 0xB8, 0xAD,
    0x00, 0x2C, 0x54, 0x7C, 0xF2, 0xB1, 0x8C, 0xB4, 0x99, 0xB9, 0xA3, 0x2D, 0x55, 0x7D, 0x81, 0x91,
    0xAC, 0x38, 0xAD, 0x3A, 0xB5, 0x83, 0x91, 0xAC, 0x2D, 0xD9, 0x28, 0xD8, 0x4D, 0xD9, 0x48, 0xD8,
    0x6D, 0xD9, 0x68, 0xD8, 0x8C, 0x9D, 0xAE, 0x29, 0xD9, 0x04, 0xAE, 0xD8, 0x51, 0xD9, 0x04, 0xAE,
    0xD8, 0x79, 0xD9, 0x04, 0xD8, 0x81, 0xF3, 0x9D, 0xAD, 0x00, 0x8D, 0xAE, 0x19, 0x81, 0xAD, 0xD9,
    0x01, 0xD8, 0xF2, 0xAE, 0xDA, 0x26, 0xD8, 0x8E, 0x91, 0x29, 0x83, 0xA7, 0xD9, 0xAD, 0xAD, 0xAD,
    0xAD, 0xF3, 0x2A, 0xD8, 0xD8, 0xF1, 0xB0, 0xAC, 0x89, 0x91, 0x3E, 0x5E, 0x76, 0xF3, 0xAC, 0x2E,
    0x2E, 0xF1, 0xB1, 0x8C, 0x5A, 0x9C, 0xAC, 0x2C, 0x28, 0x28, 0x28, 0x9C, 0xAC, 0x30, 0x18, 0xA8,
    0x98, 0x81, 0x28, 0x34, 0x3C, 0x97, 0x24, 0xA7, 0x28, 0x34, 0x3C, 0x9C, 0x24, 0xF2, 0xB0, 0x89,
    0xAC, 0x91, 0x2C, 0x4C, 0x6C, 0x8A, 0x9B, 0x2D, 0xD9, 0xD8, 0xD8, 0x51, 0xD9, 0xD8, 0xD8, 0x79,

    // bank 7, 138 bytes (remainder)
    0xD9, 0xD8, 0xD8, 0xF1, 0x9E, 0x88, 0xA3, 0x31, 0xDA, 0xD8, 0xD8, 0x91, 0x2D, 0xD9, 0x28, 0xD8,
    0x4D, 0xD9, 0x48, 0xD8, 0x6D, 0xD9, 0x68, 0xD8, 0xB1, 0x83, 0x93, 0x35, 0x3D, 0x80, 0x25, 0xDA,
    0xD8, 0xD8, 0x85, 0x69, 0xDA, 0xD8, 0xD8, 0xB4, 0x93, 0x81, 0xA3, 0x28, 0x34, 0x3C, 0xF3, 0xAB,
    0x8B, 0xF8, 0xA3, 0x91, 0xB6, 0x09, 0xB4, 0xD9, 0xAB, 0xDE, 0xFA, 0xB0, 0x87, 0x9C, 0xB9, 0xA3,
    0xDD, 0xF1, 0xA3, 0xA3, 0xA3, 0xA3, 0x95, 0xF1, 0xA3, 0xA3, 0xA3, 0x9D, 0xF1, 0xA3, 0xA3, 0xA3,
    0xA3, 0xF2, 0xA3, 0xB4, 0x90, 0x80, 0xF2, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3,
    0xA3, 0xB2, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xB0, 0x87, 0xB5, 0x99, 0xF1, 0xA3, 0xA3, 0xA3,
    0x98, 0xF1, 0xA3, 0xA3, 0xA3, 0xA3, 0x97, 0xA3, 0xA3, 0xA3, 0xA3, 0xF3, 0x9B, 0xA3, 0xA3, 0xDC,
    0xB9, 0xA7, 0xF1, 0x26, 0x26, 0x26, 0xD8, 0xD8, 0xFF
};

// thanks to Noah Zerkin for piecing this stuff together!
const uint8_t dmpConfig[MPU6050_DMP_CONFIG_SIZE] = {
//  BANK    OFFSET  LENGTH  [DATA]
    0x03,   0x7B,   0x03,   0x4C, 0xCD, 0x6C,         // FCFG_1 inv_set_gyro_calibration
    0x03,   0xAB,   0x03,   0x36, 0x56, 0x76,         // FCFG_3 inv_set_gyro_calibration
    0x00,   0x68,   0x04,   0x02, 0xCB, 0x47, 0xA2,   // D_0_104 inv_set_gyro_calibration
    0x02,   0x18,   0x04,   0x00, 0x05, 0x8B, 0xC1,   // D_0_24 inv_set_gyro_calibration
    0x01,   0x0C,   0x04,   0x00, 0x00, 0x00, 0x00,   // D_1_152 inv_set_accel_calibration
    0x03,   0x7F,   0x06,   0x0C, 0xC9, 0x2C, 0x97, 0x97, 0x97, // FCFG_2 inv_set_accel_calibration
    0x03,   0x89,   0x03,   0x26, 0x46, 0x66,         // FCFG_7 inv_set_accel_calibration
    0x00,   0x6C,   0x02,   0x20, 0x00,               // D_0_108 inv_set_accel_calibration
    0x02,   0x40,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_00 inv_set_compass_calibration
    0x02,   0x44,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_01
    0x02,   0x48,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_02
    0x02,   0x4C,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_10
    0x02,   0x50,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_11
    0x02,   0x54,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_12
    0x02,   0x58,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_20
    0x02,   0x5C,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_21
    0x02,   0xBC,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_22
    0x01,   0xEC,   0x04,   0x00, 0x00, 0x40, 0x00,   // D_1_236 inv_apply_endian_accel
    0x03,   0x7F,   0x06,   0x0C, 0xC9, 0x2C, 0x97, 0x97, 0x97, // FCFG_2 inv_set_mpu_sensors
    0x04,   0x02,   0x03,   0x0D, 0x35, 0x5D,         // CFG_MOTION_BIAS inv_turn_on_bias_from_no_motion
    0x04,   0x09,   0x04,   0x87, 0x2D, 0x35, 0x3D,   // FCFG_5 inv_set_bias_update
    0x00,   0xA3,   0x01,   0x00,                     // D_0_163 inv_set_dead_zone
                 // SPECIAL 0x01 = enable interrupts
    0x00,   0x00,   0x00,   0x01, // SET INT_ENABLE at i=22, SPECIAL INSTRUCTION
    0x07,   0x86,   0x01,   0xFE,                     // CFG_6 inv_set_fifo_interupt
    0x07,   0x41,   0x05,   0xF1, 0x20, 0x28, 0x30, 0x38, // CFG_8 inv_send_quaternion
    0x07,   0x7E,   0x01,   0x30,                     // CFG_16 inv_set_footer
    0x07,   0x46,   0x01,   0x9A,                     // CFG_GYRO_SOURCE inv_send_gyro
    0x07,   0x47,   0x04,   0xF1, 0x28, 0x30, 0x38,   // CFG_9 inv_send_gyro -> inv_construct3_fifo
    0x07,   0x6C,   0x04,   0xF1, 0x28, 0x30, 0x38,   // CFG_12 inv_send_accel -> inv_construct3_fifo
    0x02,   0x16,   0x02,   0x00, 0x04                  // D_0_22 inv_set_fifo_rate

    // This very last 0x01 WAS a 0x09, which drops the FIFO rate down to 20 Hz. 0x07 is 25 Hz,
    // 0x01 is 100Hz. Going faster than 100Hz (0x00=200Hz) tends to result in very noisy data.
    // DMP output frequency is calculated easily using this equation: (200Hz / (1 + value))

    // It is important to make sure the host processor can keep up with reading and processing
    // the FIFO output at the desired rate. Handling FIFO overflow cleanly is also a good idea.
};

const uint8_t dmpUpdates[MPU6050_DMP_UPDATES_SIZE] = {
    0x01,   0xB2,   0x02,   0xFF, 0xFF,
    0x01,   0x90,   0x04,   0x09, 0x23, 0xA1, 0x35,
    0x01,   0x6A,   0x02,   0x06, 0x00,
    0x01,   0x60,   0x08,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00,   0x60,   0x04,   0x40, 0x00, 0x00, 0x00,
    0x01,   0x62,   0x02,   0x00, 0x00,
    0x00,   0x60,   0x04,   0x00, 0x40, 0x00, 0x00
};
uint8_t j;
extern char tempbuf[32];

/* Default values for all registers is 0 except for 107 (0x40) and 117 (0x68)
These are the power configuraition and i2c address registers
Default sample rate is 8kHz */
void mpu6050_init() {
    /* Configure INT pin as digital IO input */
    P2DIR &= ~MPU6050_INT;  // Input
    P2SEL &= ~MPU6050_INT;  // Digital IO Psel and psel2 are 0
    P2SEL2 &= ~MPU6050_INT;
    P2IES &= ~MPU6050_INT;  // Edge select 0 = low to high
    P2IFG &= ~MPU6050_INT;  // Clear the interrupt flag before enabling interrupt

	/* Configure I2C */
	i2c_init();
}

uint8_t mpu6050_getOTPBankValid() {
    i2c_read_reg(MPU6050_RA_XG_OFFS_TC);
    return (i2c_rx_buffer[0] & MPU6050_TC_OTP_BNK_VLD_BIT);
}

void mpu6050_setMemoryBank(uint8_t bank, bool prefetchEnabled, bool userBank) {
    bank &= 0x1F;
    if (userBank) bank |= 0x20;
    if (prefetchEnabled) bank |= 0x40;
    i2c_write_reg(MPU6050_RA_BANK_SEL,bank);
}

// MEM_START_ADDR register
void mpu6050_setMemoryStartAddress(uint8_t address) {
    i2c_write_reg(MPU6050_RA_MEM_START_ADDR,address);
}

// MEM_R_W register
uint8_t mpu6050_readMemoryByte() {
    i2c_read_reg(MPU6050_RA_MEM_R_W);
    return i2c_rx_buffer[0];
}

int8_t mpu6050_getXGyroOffset() {
    i2c_read_bits(MPU6050_RA_XG_OFFS_TC,MPU6050_TC_OFFSET_BIT,MPU6050_TC_OFFSET_LENGTH);
    return i2c_rx_buffer[0];
}
int8_t mpu6050_getYGyroOffset() {
    i2c_read_bits(MPU6050_RA_YG_OFFS_TC,MPU6050_TC_OFFSET_BIT,MPU6050_TC_OFFSET_LENGTH);
    return i2c_rx_buffer[0];
}

int8_t mpu6050_getZGyroOffset() {
    i2c_read_bits(MPU6050_RA_ZG_OFFS_TC,MPU6050_TC_OFFSET_BIT,MPU6050_TC_OFFSET_LENGTH);
    return i2c_rx_buffer[0];
}

void mpu6050_setSlaveAddress(uint8_t num, uint8_t address) {
    if (num > 3) return;
    i2c_write_reg(MPU6050_RA_I2C_SLV0_ADDR+num*3,address);
}
/** Set I2C Master Mode enabled status.
 * @param enabled New I2C Master Mode enabled status
 * @see getI2CMasterModeEnabled()
 * @see MPU6050_RA_USER_CTRL
 * @see MPU6050_USERCTRL_I2C_MST_EN_BIT
 */
void mpu6050_setI2CMasterModeEnabled(bool enabled) {
    i2c_write_bit(MPU6050_RA_USER_CTRL,MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}


/** Reset the I2C Master.
 * This bit resets the I2C Master when set to 1 while I2C_MST_EN equals 0.
 * This bit automatically clears to 0 after the reset has been triggered.
 * @see MPU6050_RA_USER_CTRL
 * @see MPU6050_USERCTRL_I2C_MST_RESET_BIT
 */
void mpu6050_resetI2CMaster() {
    i2c_write_bit(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_RESET_BIT,true);
}

void mpu6050_readMemoryBlock(uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address) {
    mpu6050_setMemoryBank(bank,true,false);
    mpu6050_setMemoryStartAddress(address);
    uint8_t chunkSize;
    for (uint16_t i = 0; i < dataSize;) {
        // determine correct chunk size according to bank position and data size
        chunkSize = MPU6050_DMP_MEMORY_CHUNK_SIZE;

        // make sure we don't go past the data size
        if (i + chunkSize > dataSize) chunkSize = dataSize - i;

        // make sure this chunk doesn't go past the bank boundary (256 bytes)
        if (chunkSize > 256 - address) chunkSize = 256 - address;

        // read the chunk of data as specified
        i2c_read_bytes(MPU6050_RA_MEM_R_W,chunkSize,data+i);

        // increase byte index by [chunkSize]
        i += chunkSize;

        // uint8_t automatically wraps to 0 at 256
        address += chunkSize;

        // if we aren't done, update bank (if necessary) and address
        if (i < dataSize) {
            if (address == 0) bank++;
            mpu6050_setMemoryBank(bank,true,false);
            mpu6050_setMemoryStartAddress(address);
        }
    }
}

// first call: ...0,0,
bool mpu6050_writeMemoryBlock(const uint8_t data[], uint16_t dataSize, uint8_t bank, uint8_t address, bool verify, bool useProgMem) {
 #ifdef MPU6050_DEBUG
    hc05_transmit("writeMemoryBlock\r\n",18);
 #endif // MPU6050_DEBUG
    mpu6050_setMemoryBank(bank,false,false);
    mpu6050_setMemoryStartAddress(address);
    uint8_t chunkSize;
    uint8_t *verifyBuffer;
    uint8_t *progBuffer;
    uint16_t i;
    uint8_t j;
    if (verify) verifyBuffer = (uint8_t *)malloc(MPU6050_DMP_MEMORY_CHUNK_SIZE);
    if (useProgMem) progBuffer = (uint8_t *)malloc(MPU6050_DMP_MEMORY_CHUNK_SIZE);
    for (i = 0; i < dataSize;) {
        // determine correct chunk size according to bank position and data size
        chunkSize = MPU6050_DMP_MEMORY_CHUNK_SIZE;

        // make sure we don't go past the data size
        if (i + chunkSize > dataSize) chunkSize = dataSize - i;

        // make sure this chunk doesn't go past the bank boundary (256 bytes)
        if (chunkSize > 256 - address) chunkSize = 256 - address;

        if (useProgMem) {
            // write the chunk of data as specified
            for (j = 0; j < chunkSize; j++) {
                progBuffer[j] = pgm_read_byte(data+i+j);//data[i+j];//pgm_read_byte(data + i + j);
                //sprintf(tempbuf,"%d ",progBuffer[j]);
                //hc05_transmit(tempbuf,strlen(tempbuf));
            }
        } else {
            // write the chunk of data as specified
            progBuffer = (uint8_t *)data + i;
         //   for(j=0;j<chunkSize;j++)            {
             //sprintf(tempbuf,"%d ",progBuffer[j]);
            //hc05_transmit(tempbuf,strlen(tempbuf));

           // }
        }

        //i2c_write_regs(MPU6050_RA_MEM_R_W, chunkSize, progBuffer);
        i2c_write_bytes(MPU6050_RA_MEM_R_W,chunkSize,progBuffer);

        // verify data if needed
        if (verify && verifyBuffer) {
            mpu6050_setMemoryBank(bank,false,false);
            mpu6050_setMemoryStartAddress(address);
            //i2c_read_bytes(MPU6050_RA_MEM_R_W, chunkSize, verifyBuffer);
            i2c_read_bytes(MPU6050_RA_MEM_R_W,chunkSize,verifyBuffer);
          //  for (int k = 0; k<chunkSize;k++) {
            //    sprintf(tempbuf,"%d ",verifyBuffer[k]);
              //  hc05_transmit(tempbuf,strlen(tempbuf));
              //  }
            if (memcmp(progBuffer, verifyBuffer, chunkSize) != 0) {
                free(verifyBuffer);
                if (useProgMem) free(progBuffer);
                return false; // uh oh.
            }
        }

        // increase byte index by [chunkSize]
        i += chunkSize;

        // uint8_t automatically wraps to 0 at 256
        address += chunkSize;

        // if we aren't done, update bank (if necessary) and address
        if (i < dataSize) {
            if (address == 0) bank++;
            mpu6050_setMemoryBank(bank,false,false);
            mpu6050_setMemoryStartAddress(address);
        }
    }
    if (verify) free(verifyBuffer);
    if (useProgMem) free(progBuffer);
    return true;
}

// first call : ...0,0,true);
bool mpu6050_writeProgMemoryBlock(const uint8_t data[], uint16_t dataSize, uint8_t bank, uint8_t address, bool verify) {
    return mpu6050_writeMemoryBlock(data, dataSize, bank, address, verify, true);
}

//dmp config size is 192 datasize
bool mpu6050_writeDMPConfigurationSet(const uint8_t *data, uint16_t dataSize, bool useProgMem) {
    uint8_t *progBuffer, success, special;
    uint16_t i, j;
    if (useProgMem) {
        progBuffer = (uint8_t *)malloc(8); // assume 8-byte blocks, realloc later if necessary
    }

    // config set data is a long string of blocks with the following structure:
    // [bank] [offset] [length] [byte[0], byte[1], ..., byte[length]]
    uint8_t bank, offset, length;
    for (i = 0; i < dataSize;) { //192 times

        if (useProgMem) { // true
            bank = pgm_read_byte(data + i++);
            offset = pgm_read_byte(data + i++);
            length = pgm_read_byte(data + i++);
        } else {
            bank = data[i++];
            offset = data[i++];
            length = data[i++];
        }

        // write data or perform special action
        if (length > 0) {
            // regular block of data to write
#ifdef MPU6050_DEBUG
            sprintf(tempbuf,"bank: %x, offset: %x, length: %x\r\n", bank,offset,length);
             hc05_transmit(tempbuf,strlen(tempbuf));
#endif // MPU6050_DEBUG
            if (useProgMem) {
                if (sizeof(progBuffer) < length) {
                    free(progBuffer);
                    progBuffer = (uint8_t *)malloc(length);
                }
                for (j = 0; j < length; j++) progBuffer[j] = pgm_read_byte(data + i + j);
            } else {
                progBuffer = (uint8_t *)data + i;
            }
            success = mpu6050_writeMemoryBlock(progBuffer, length, bank, offset, true, false);
            i += length;
        } else {
            // special instruction
            // NOTE: this kind of behavior (what and when to do certain things)
            // is totally undocumented. This code is in here based on observed
            // behavior only, and exactly why (or even whether) it has to be here
            // is anybody's guess for now.
            if (useProgMem) {
                special = pgm_read_byte(data + i++);
            } else {
                special = data[i++];
            }
#ifdef MPU6050_DEBUG
            sprintf(tempbuf,"special: %x found\r\n", special);
             hc05_transmit(tempbuf,strlen(tempbuf));
#endif // MPU6050_DEBUG
            if (special == 0x01) {
                // enable DMP-related interrupts

                //setIntZeroMotionEnabled(true);
                //setIntFIFOBufferOverflowEnabled(true);
                //setIntDMPEnabled(true);
               //i2c_write_reg(MPU6050_RA_INT_ENABLE, 0x32);  // single operation
                i2c_write_reg(MPU6050_RA_INT_ENABLE, 0x32);
                success = true;
            } else {
                // unknown special command
                success = false;
            }
        }

        if (!success) {
            if (useProgMem) free(progBuffer);
            return false; // uh oh
        }
    }
    if (useProgMem) free(progBuffer);
    return true;
}

bool mpu6050_writeProgDMPConfigurationSet(const uint8_t *data, uint16_t dataSize) {
    return mpu6050_writeDMPConfigurationSet(data, dataSize, true);
}
/** Set full interrupt enabled status.
 * Full register byte for all interrupts, for quick reading. Each bit should be
 * set 0 for disabled, 1 for enabled.
 * @param enabled New interrupt enabled status
 * @see getIntFreefallEnabled()
 * @see MPU6050_RA_INT_ENABLE
 * @see MPU6050_INTERRUPT_FF_BIT
 **/
void mpu6050_setIntEnabled(uint8_t enabled) {
    i2c_write_reg(MPU6050_RA_INT_ENABLE,enabled);
}

/** Set gyroscope sample rate divider.
 * @param rate New sample rate divider
 * @see getRate()
 * @see MPU6050_RA_SMPLRT_DIV
 */
void mpu6050_setRate(uint8_t rate) {
    i2c_write_reg(MPU6050_RA_SMPLRT_DIV,rate);
}

/** Set clock source setting.
 * An internal 8MHz oscillator, gyroscope based clock, or external sources can
 * be selected as the MPU-60X0 clock source. When the internal 8 MHz oscillator
 * or an external source is chosen as the clock source, the MPU-60X0 can operate
 * in low power modes with the gyroscopes disabled.
 *
 * Upon power up, the MPU-60X0 clock source defaults to the internal oscillator.
 * However, it is highly recommended that the device be configured to use one of
 * the gyroscopes (or an external clock source) as the clock reference for
 * improved stability. The clock source can be selected according to the following table:
 *
 * <pre>
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
 * </pre>
 *
 * @param source New clock source setting
 * @see getClockSource()
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_CLKSEL_BIT
 * @see MPU6050_PWR1_CLKSEL_LENGTH
 */
void mpu6050_setClockSource(uint8_t source) {
    i2c_write_bits(MPU6050_RA_PWR_MGMT_1,MPU6050_PWR1_CLKSEL_BIT,MPU6050_PWR1_CLKSEL_LENGTH,source);
}

/** Set external FSYNC configuration.
 * @see getExternalFrameSync()
 * @see MPU6050_RA_CONFIG
 * @param sync New FSYNC configuration value
 */
void mpu6050_setExternalFrameSync(uint8_t sync) {
    i2c_write_bits(MPU6050_RA_CONFIG, MPU6050_CFG_EXT_SYNC_SET_BIT, MPU6050_CFG_EXT_SYNC_SET_LENGTH, sync);
}

/** Set digital low-pass filter configuration.
 * @param mode New DLFP configuration setting
 * @see getDLPFBandwidth()
 * @see MPU6050_DLPF_BW_256
 * @see MPU6050_RA_CONFIG
 * @see MPU6050_CFG_DLPF_CFG_BIT
 * @see MPU6050_CFG_DLPF_CFG_LENGTH
 */
void mpu6050_setDLPFMode(uint8_t mode) {
    i2c_write_bits(MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, mode);
}

/** Set full-scale gyroscope range.
 * @param range New full-scale gyroscope range value
 * @see getFullScaleRange()
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
void mpu6050_setFullScaleGyroRange(uint8_t range) {
    i2c_write_bits(MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

void mpu6050_setDMPConfig1(uint8_t config) {
    i2c_write_reg(MPU6050_RA_DMP_CFG_1, config);
}

void mpu6050_setDMPConfig2(uint8_t config) {
    i2c_write_reg(MPU6050_RA_DMP_CFG_2, config);
}

void mpu6050_setOTPBankValid(bool enabled) {
    i2c_write_bit(MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OTP_BNK_VLD_BIT, enabled);
}


void mpu6050_setXGyroOffset(int8_t offset) {
    i2c_write_bits(MPU6050_RA_XG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, offset);
}

void mpu6050_setYGyroOffset(int8_t offset) {
    i2c_write_bits(MPU6050_RA_YG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, offset);
}

void mpu6050_setZGyroOffset(int8_t offset) {
    i2c_write_bits(MPU6050_RA_ZG_OFFS_TC, MPU6050_TC_OFFSET_BIT, MPU6050_TC_OFFSET_LENGTH, offset);
}

void mpu6050_setXGyroOffsetUser(int16_t offset) {
    i2c_write_word(MPU6050_RA_XG_OFFS_USRH, offset);
}

void mpu6050_setYGyroOffsetUser(int16_t offset) {
    i2c_write_word(MPU6050_RA_YG_OFFS_USRH, offset);
}

void mpu6050_setZGyroOffsetUser(int16_t offset) {
    i2c_write_word(MPU6050_RA_ZG_OFFS_USRH, offset);
}

/** Reset the FIFO.
 * This bit resets the FIFO buffer when set to 1 while FIFO_EN equals 0. This
 * bit automatically clears to 0 after the reset has been triggered.
 * @see MPU6050_RA_USER_CTRL
 * @see MPU6050_USERCTRL_FIFO_RESET_BIT
 */
void mpu6050_resetFIFO() {
    i2c_write_bit(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_RESET_BIT, true);
}


/** Get current FIFO buffer size.
 * This value indicates the number of bytes stored in the FIFO buffer. This
 * number is in turn the number of bytes that can be read from the FIFO buffer
 * and it is directly proportional to the number of samples available given the
 * set of sensor data bound to be stored in the FIFO (register 35 and 36).
 * @return Current FIFO buffer size
 */
uint16_t mpu6050_getFIFOCount() {
    i2c_read_bytes(MPU6050_RA_FIFO_COUNTH, 2,mpu6050_buffer);
    uint16_t tmpcount = (mpu6050_buffer[0] << 8) | mpu6050_buffer[1];
    return tmpcount;
}

void mpu6050_getFIFOBytes(uint8_t *data, uint8_t length) {
    i2c_read_bytes(MPU6050_RA_FIFO_R_W, length, data);
}

/** Set free-fall event acceleration threshold.
 * @param threshold New motion detection acceleration threshold value (LSB = 2mg)
 * @see getMotionDetectionThreshold()
 * @see MPU6050_RA_MOT_THR
 */
void mpu6050_setMotionDetectionThreshold(uint8_t threshold) {
    i2c_write_reg(MPU6050_RA_MOT_THR, threshold);
}

/** Set zero motion detection event acceleration threshold.
 * @param threshold New zero motion detection acceleration threshold value (LSB = 2mg)
 * @see getZeroMotionDetectionThreshold()
 * @see MPU6050_RA_ZRMOT_THR
 */
void mpu6050_setZeroMotionDetectionThreshold(uint8_t threshold) {
    i2c_write_reg(MPU6050_RA_ZRMOT_THR, threshold);
}

/** Set motion detection event duration threshold.
 * @param duration New motion detection duration threshold value (LSB = 1ms)
 * @see getMotionDetectionDuration()
 * @see MPU6050_RA_MOT_DUR
 */
void mpu6050_setMotionDetectionDuration(uint8_t duration) {
    i2c_write_reg(MPU6050_RA_MOT_DUR, duration);
}

/** Set zero motion detection event duration threshold.
 * @param duration New zero motion detection duration threshold value (LSB = 1ms)
 * @see getZeroMotionDetectionDuration()
 * @see MPU6050_RA_ZRMOT_DUR
 */
void mpu6050_setZeroMotionDetectionDuration(uint8_t duration) {
    i2c_write_reg(MPU6050_RA_ZRMOT_DUR, duration);
}

/** Set FIFO enabled status.
 * @param enabled New FIFO enabled status
 * @see getFIFOEnabled()
 * @see MPU6050_RA_USER_CTRL
 * @see MPU6050_USERCTRL_FIFO_EN_BIT
 */
void mpu6050_setFIFOEnabled(bool enabled) {
    i2c_write_bit(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_EN_BIT, enabled);
}
void mpu6050_setDMPEnabled(bool enabled) {
    i2c_write_bit(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_EN_BIT, enabled);
}
void mpu6050_resetDMP() {
    i2c_write_bit(MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_RESET_BIT, true);
}

/** Get full set of interrupt status bits.
 * These bits clear to 0 after the register has been read. Very useful
 * for getting multiple INT statuses, since each single bit read clears
 * all of them because it has to read the whole byte.
 * @return Current interrupt status
 * @see MPU6050_RA_INT_STATUS
 */
uint8_t mpu6050_getIntStatus() {
    i2c_read_reg(MPU6050_RA_INT_STATUS);
    return i2c_rx_buffer[0];
}
void mpu6050_dmpinit(){
    // reset device
#ifdef MPU6050_DEBUG
    sprintf(tempbuf,"\n\nResetting MPU6050...\r\n");
    hc05_transmit(tempbuf,strlen(tempbuf));
#endif // MPU6050_DEBUG
    mpu6050_reset();

    delay_ms(30);

    // disable sleep mode
#ifdef MPU6050_DEBUG
    sprintf(tempbuf,"Disabling sleep mode...\r\n");
    hc05_transmit(tempbuf,strlen(tempbuf));
#endif // MPU6050_DEBUG
    mpu6050_wakeup();

    // get MPU hardware revision
#ifdef MPU6050_DEBUG
    sprintf(tempbuf,"Selecting user bank 16..\r\n");
    hc05_transmit(tempbuf,strlen(tempbuf));
    #endif // MPU6050_DEBUG
    mpu6050_setMemoryBank(0x10, true, true);
#ifdef MPU6050_DEBUG
    sprintf(tempbuf,"Selecting memory byte 6..\r\n");
    hc05_transmit(tempbuf,strlen(tempbuf));
    #endif // MPU6050_DEBUG
    mpu6050_setMemoryStartAddress(0x06);
#ifdef MPU6050_DEBUG
    sprintf(tempbuf,"Checking hardware revision..\r\n");
    hc05_transmit(tempbuf,strlen(tempbuf));

    uint8_t hwRevision = mpu6050_readMemoryByte();

    sprintf(tempbuf,"Revision @ user[16][6] = %X\r\n",hwRevision);
    hc05_transmit(tempbuf,strlen(tempbuf));
#endif // MPU6050_DEBUG
  //  sprintf(hwRevision, HEX);
#ifdef MPU6050_DEBUG
    sprintf(tempbuf,"Resetting memory bank selection to 0..\r\n");
    hc05_transmit(tempbuf,strlen(tempbuf));
#endif // MPU6050_DEBUG
    mpu6050_setMemoryBank(0, false, false);

    // check OTP bank valid
#ifdef MPU6050_DEBUG
    sprintf(tempbuf,"Reading OTP bank valid flag..\r\n");
    hc05_transmit(tempbuf,strlen(tempbuf));

    uint8_t otpValid = mpu6050_getOTPBankValid();

    if(otpValid) {
        sprintf(tempbuf,"OTP bank is valid\r\n");
    }
    else {
        sprintf(tempbuf,"OTP bank is not valid!\r\n");
    }
    hc05_transmit(tempbuf,strlen(tempbuf));

    // get X/Y/Z gyro offsets
    sprintf(tempbuf,"Reading gyro offset values..\r\n");
    hc05_transmit(tempbuf,strlen(tempbuf));
#endif // MPU6050_DEBUG
    int8_t xgOffset = mpu6050_getXGyroOffset();
    int8_t ygOffset = mpu6050_getYGyroOffset();
    int8_t zgOffset = mpu6050_getZGyroOffset();
#ifdef MPU6050_DEBUG
    sprintf(tempbuf,"X gyro offset = %d\r\n",xgOffset);
    hc05_transmit(tempbuf,strlen(tempbuf));
    sprintf(tempbuf,"Y gyro offset = %d\r\n",ygOffset);
    hc05_transmit(tempbuf,strlen(tempbuf));
    sprintf(tempbuf,"Z gyro offset = %d\r\n",zgOffset);
    hc05_transmit(tempbuf,strlen(tempbuf));
#endif

#ifdef MPU6050_DEBUG
    // setup weird slave stuff (?)
    sprintf(tempbuf,"Setting slave 0 address to 0x7F..\r\n");
    hc05_transmit(tempbuf,strlen(tempbuf));
#endif // MPU6050_DEBUG
    mpu6050_setSlaveAddress(0, 0x7F);
#ifdef MPU6050_DEBUG
    sprintf(tempbuf,"Disabling I2C Master mode..\r\n");
    hc05_transmit(tempbuf,strlen(tempbuf));
#endif // MPU6050_DEBUG
    mpu6050_setI2CMasterModeEnabled(false);
#ifdef MPU6050_DEBUG
    sprintf(tempbuf,"Setting slave 0 address to 0x68 (self)..\r\n");
    hc05_transmit(tempbuf,strlen(tempbuf));
#endif // MPU6050_DEBUG
    mpu6050_setSlaveAddress(0, 0x68);
#ifdef MPU6050_DEBUG
    sprintf(tempbuf,"Resetting I2C Master control..\r\n");
    hc05_transmit(tempbuf,strlen(tempbuf));
#endif // MPU6050_DEBUG
    mpu6050_resetI2CMaster();
    delay_ms(20);

    // load DMP code into memory banks
#ifdef MPU6050_DEBUG
    sprintf(tempbuf,"Writing DMP code to MPU memory banks (%d bytes)\r\n",MPU6050_DMP_CODE_SIZE);
    hc05_transmit(tempbuf,strlen(tempbuf));
#endif // MPU6050_DEBUG
     if ( mpu6050_writeProgMemoryBlock(dmpMemory, MPU6050_DMP_CODE_SIZE,0,0,true)) {
#ifdef MPU6050_DEBUG
        sprintf(tempbuf,"Success! DMP code written and verified\r\n");
        hc05_transmit(tempbuf,strlen(tempbuf));
#endif // MPU6050_DEBUG
        // write DMP configuration
#ifdef MPU6050_DEBUG
        sprintf(tempbuf,"Writing DMP configuration to MPU memory banks (%d bytes in config def\r\n",MPU6050_DMP_CONFIG_SIZE);
        hc05_transmit(tempbuf,strlen(tempbuf));
#endif // MPU6050_DEBUG
        if (mpu6050_writeProgDMPConfigurationSet(dmpConfig, MPU6050_DMP_CONFIG_SIZE)) {
            sprintf(tempbuf,"Success! DMP configuration written and verified\r\n");
            hc05_transmit(tempbuf,strlen(tempbuf));

#ifdef MPU6050_DEBUG
            sprintf(tempbuf,"Setting clock source to Z Gyro..\r\n");
            hc05_transmit(tempbuf,strlen(tempbuf));
#endif // MPU6050_DEBUG
            mpu6050_setClockSource(MPU6050_CLOCK_PLL_ZGYRO);

#ifdef MPU6050_DEBUG
            sprintf(tempbuf,"Setting DMP and FIFO_OFLOW interrupts enabled..\r\n");
            hc05_transmit(tempbuf,strlen(tempbuf));
#endif // MPU6050_DEBUG
            mpu6050_setIntEnabled(0x12); //0x12 is fifo overflow and ... I have no idea

#ifdef MPU6050_DEBUG
            sprintf(tempbuf,"Setting sample rate to 200Hz..\r\n");
            hc05_transmit(tempbuf,strlen(tempbuf));
#endif // MPU6050_DEBUG
            mpu6050_setRate(4); // 1khz / (1 + 4) = 200 Hz

#ifdef MPU6050_DEBUG
            sprintf(tempbuf,"Setting external frame sync to TEMP_OUT_L[0]..\r\n");
            hc05_transmit(tempbuf,strlen(tempbuf));
#endif // MPU6050_DEBUG
            mpu6050_setExternalFrameSync(MPU6050_EXT_SYNC_TEMP_OUT_L);

#ifdef MPU6050_DEBUG
            sprintf(tempbuf,"Setting DLPF bandwidth to 42Hz..\r\n");
            hc05_transmit(tempbuf,strlen(tempbuf));
#endif // MPU6050_DEBUG
            mpu6050_setDLPFMode(MPU6050_DLPF_BW_42);

#ifdef MPU6050_DEBUG
            sprintf(tempbuf,"Setting gyro sensitivity to +/- 2000 deg/sec..\r\n");
            hc05_transmit(tempbuf,strlen(tempbuf));
#endif // MPU6050_DEBUG
            mpu6050_setFullScaleGyroRange(MPU6050_GYRO_FS_2000);

#ifdef MPU6050_DEBUG
            sprintf(tempbuf,"Setting DMP configuration bytes (function unknown)..\r\n");
            hc05_transmit(tempbuf,strlen(tempbuf));
#endif // MPU6050_DEBUG
            mpu6050_setDMPConfig1(0x03);
            mpu6050_setDMPConfig2(0x00);

#ifdef MPU6050_DEBUG
            sprintf(tempbuf,"Clearing OTP Bank flag..\r\n");
            hc05_transmit(tempbuf,strlen(tempbuf));
#endif // MPU6050_DEBUG
            mpu6050_setOTPBankValid(false);

#ifdef MPU6050_DEBUG
            sprintf(tempbuf,"Setting X/Y/Z gyro offsets to previous values..\r\n");
            hc05_transmit(tempbuf,strlen(tempbuf));
#endif // MPU6050_DEBUG
            mpu6050_setXGyroOffset(xgOffset);
            mpu6050_setYGyroOffset(ygOffset);
            mpu6050_setZGyroOffset(zgOffset);

#ifdef MPU6050_DEBUG
            sprintf(tempbuf,"Setting X/Y/Z gyro user offsets to zero..\r\n");
            hc05_transmit(tempbuf,strlen(tempbuf));
#endif // MPU6050_DEBUG
            mpu6050_setXGyroOffsetUser(0);
            mpu6050_setYGyroOffsetUser(0);
            mpu6050_setZGyroOffsetUser(0);

#ifdef MPU6050_DEBUG
            sprintf(tempbuf,"Writing final memory update 1/7 (function unknown)..\r\n");
            hc05_transmit(tempbuf,strlen(tempbuf));
#endif // MPU6050_DEBUG
            uint8_t dmpUpdate[16], j;
            uint16_t pos = 0;
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
            mpu6050_writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1],true,false);

#ifdef MPU6050_DEBUG
            sprintf(tempbuf,"Writing final memory update 2/7 (function unknown)..\r\n");
            hc05_transmit(tempbuf,strlen(tempbuf));
#endif // MPU6050_DEBUG
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
            mpu6050_writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1],true,false);

#ifdef MPU6050_DEBUG
            sprintf(tempbuf,"Resetting FIFO..\r\n");
            hc05_transmit(tempbuf,strlen(tempbuf));
#endif // MPU6050_DEBUG
            mpu6050_resetFIFO();

#ifdef MPU6050_DEBUG
            sprintf(tempbuf,"Reading FIFO count..\r\n");
            hc05_transmit(tempbuf,strlen(tempbuf));
#endif // MPU6050_DEBUG
            uint8_t fifoCount = mpu6050_getFIFOCount();

#ifdef MPU6050_DEBUG
            sprintf(tempbuf,"Current FIFO count=%d\r\n",fifoCount);
            hc05_transmit(tempbuf,strlen(tempbuf));
#endif // MPU6050_DEBUG
            mpu6050_getFIFOBytes(fifoBuffer, fifoCount);

#ifdef MPU6050_DEBUG
            sprintf(tempbuf,"Setting motion detection threshold to 2..\r\n");
            hc05_transmit(tempbuf,strlen(tempbuf));
#endif // MPU6050_DEBUG
            mpu6050_setMotionDetectionThreshold(2);

#ifdef MPU6050_DEBUG
            sprintf(tempbuf,"Setting zero-motion detection threshold to 156..\r\n");
            hc05_transmit(tempbuf,strlen(tempbuf));
#endif // MPU6050_DEBUG
            mpu6050_setZeroMotionDetectionThreshold(156);

#ifdef MPU6050_DEBUG
            sprintf(tempbuf,"Setting motion detection duration to 80..\r\n");
            hc05_transmit(tempbuf,strlen(tempbuf));
#endif // MPU6050_DEBUG
            mpu6050_setMotionDetectionDuration(35);

#ifdef MPU6050_DEBUG
            sprintf(tempbuf,"Setting zero-motion detection duration to 0..\r\n");
            hc05_transmit(tempbuf,strlen(tempbuf));
#endif // MPU6050_DEBUG
            mpu6050_setZeroMotionDetectionDuration(0);

#ifdef MPU6050_DEBUG
            sprintf(tempbuf,"Resetting FIFO..\r\n");
            hc05_transmit(tempbuf,strlen(tempbuf));
#endif // MPU6050_DEBUG
            mpu6050_resetFIFO();

#ifdef MPU6050_DEBUG
            sprintf(tempbuf,"Enabling FIFO..\r\n");
            hc05_transmit(tempbuf,strlen(tempbuf));
#endif // MPU6050_DEBUG
            mpu6050_setFIFOEnabled(true);

#ifdef MPU6050_DEBUG
            sprintf(tempbuf,"Enabling DMP..\r\n");
            hc05_transmit(tempbuf,strlen(tempbuf));
#endif // MPU6050_DEBUG
            mpu6050_setDMPEnabled(true);

#ifdef MPU6050_DEBUG
            sprintf(tempbuf,"Resetting DMP..\r\n");
            hc05_transmit(tempbuf,strlen(tempbuf));
#endif // MPU6050_DEBUG
            mpu6050_resetDMP();

#ifdef MPU6050_DEBUG
            sprintf(tempbuf,"Writing final memory update 3/7 (function unknown)..\r\n");
            hc05_transmit(tempbuf,strlen(tempbuf));
#endif // MPU6050_DEBUG
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
            mpu6050_writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1],true,false);

#ifdef MPU6050_DEBUG
            sprintf(tempbuf,"Writing final memory update 4/7 (function unknown)..\r\n");
            hc05_transmit(tempbuf,strlen(tempbuf));
#endif // MPU6050_DEBUG
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
            mpu6050_writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1],true,false);

#ifdef MPU6050_DEBUG
            sprintf(tempbuf,"Writing final memory update 5/7 (function unknown)..\r\n");
            hc05_transmit(tempbuf,strlen(tempbuf));
#endif // MPU6050_DEBUG
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
            mpu6050_writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1],true,false);

            sprintf(tempbuf,"Waiting for FIFO count > 2..\r\n");
            hc05_transmit(tempbuf,strlen(tempbuf));

            while ((fifoCount = mpu6050_getFIFOCount()) < 3);

#ifdef MPU6050_DEBUG
            sprintf(tempbuf,"Current FIFO count=%d\r\n",fifoCount);
            hc05_transmit(tempbuf,strlen(tempbuf));

            sprintf(tempbuf,"Reading FIFO data..\r\n");
            hc05_transmit(tempbuf,strlen(tempbuf));
#endif // MPU6050_DEBUG
            mpu6050_getFIFOBytes(fifoBuffer, fifoCount);

#ifdef MPU6050_DEBUG
            sprintf(tempbuf,"Reading interrupt status..\r\n");
            hc05_transmit(tempbuf,strlen(tempbuf));

            uint8_t mpuIntStatus = mpu6050_getIntStatus();

            sprintf(tempbuf,"Current interrupt status=%X\r\n",mpuIntStatus);
            hc05_transmit(tempbuf,strlen(tempbuf));
#endif // MPU6050_DEBUG

#ifdef MPU6050_DEBUG
            sprintf(tempbuf,"Reading final memory update 6/7 (function unknown)..\r\n");
            hc05_transmit(tempbuf,strlen(tempbuf));
#endif // MPU6050_DEBUG
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
            mpu6050_readMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1]);

            sprintf(tempbuf,"Waiting for FIFO count > 2..\r\n");
            hc05_transmit(tempbuf,strlen(tempbuf));

            while ((fifoCount = mpu6050_getFIFOCount()) < 3);

            sprintf(tempbuf,"Current FIFO count=%d\r\n",fifoCount);
            hc05_transmit(tempbuf,strlen(tempbuf));

#ifdef MPU6050_DEBUG
            sprintf(tempbuf,"Reading FIFO data..\r\n");
            hc05_transmit(tempbuf,strlen(tempbuf));
#endif // MPU6050_DEBUG
            mpu6050_getFIFOBytes(fifoBuffer, fifoCount);

#ifdef MPU6050_DEBUG
            sprintf(tempbuf,"Reading interrupt status..\r\n");
            hc05_transmit(tempbuf,strlen(tempbuf));

            mpuIntStatus = mpu6050_getIntStatus();


            sprintf(tempbuf,"Current interrupt status=%X\r\n",mpuIntStatus);
            hc05_transmit(tempbuf,strlen(tempbuf));
#endif // MPU6050_DEBUG

#ifdef MPU6050_DEBUG
            sprintf(tempbuf,"Writing final memory update 7/7 (function unknown)..\r\n");
            hc05_transmit(tempbuf,strlen(tempbuf));
#endif // MPU6050_DEBUG
            for (j = 0; j < 4 || j < dmpUpdate[2] + 3; j++, pos++) dmpUpdate[j] = pgm_read_byte(&dmpUpdates[pos]);
            mpu6050_writeMemoryBlock(dmpUpdate + 3, dmpUpdate[2], dmpUpdate[0], dmpUpdate[1],true,false);

#ifdef MPU6050_DEBUG
            sprintf(tempbuf,"DMP is good to go! Finally\r\n");
            hc05_transmit(tempbuf,strlen(tempbuf));
#endif // MPU6050_DEBUG
            sprintf(tempbuf,"Disabling DMP (you turn it on later)..\r\n");
            hc05_transmit(tempbuf,strlen(tempbuf));

            mpu6050_setDMPEnabled(false);

#ifdef MPU6050_DEBUG
            sprintf(tempbuf,"Setting up internal 42-byte (default) DMP packet buffer..\r\n");
            hc05_transmit(tempbuf,strlen(tempbuf));
#endif // MPU6050_DEBUG
            dmpPacketSize = 42;
            /*if ((dmpPacketBuffer = (uint8_t *)malloc(42)) == 0) {
                return 3; // TODO: proper error code for no memory
            }*/

#ifdef MPU6050_DEBUG
            sprintf(tempbuf,"Resetting FIFO and clearing INT status one last time..\r\n");
            hc05_transmit(tempbuf,strlen(tempbuf));
#endif // MPU6050_DEBUG
            mpu6050_resetFIFO();
            mpu6050_getIntStatus();
        } else {
            sprintf(tempbuf,"ERROR! DMP configuration verification failed\r\n");
            hc05_transmit(tempbuf,strlen(tempbuf));
            return;// 2; // configuration block loading failed
        }
    } else {
        sprintf(tempbuf,"ERROR! DMP code verification failed\r\n");
        hc05_transmit(tempbuf,strlen(tempbuf));
        return;// 1; // main binary block loading failed
    }
    return;// 0; // success
}


void mpu6050_getAddress() {
    i2c_tx_buffer[0] = 0x75;
	i2c_tx_buffer_counter = 1;
	i2c_transmit();
	i2c_receive();
	hc05_transmit((char*)i2c_rx_buffer,1);
}

void mpu6050_wakeup() {
	i2c_tx_buffer[1] = 0x6B;
	i2c_tx_buffer[0] = 0x00;
	i2c_tx_buffer_counter =2;
	i2c_transmit();
	sendAck();
}
void mpu6050_sleep() {
	i2c_tx_buffer[1] = 0x6B;
	i2c_tx_buffer[0] = 0x40;
	i2c_tx_buffer_counter =2;
	i2c_transmit();
	sendAck();
}

void mpu6050_reset() {
	i2c_tx_buffer[1] = 0x6B;
	i2c_tx_buffer[0] = 0x80;
	i2c_tx_buffer_counter =2;
	i2c_transmit();
	sendAck();
}
void mpu6050_accel() {
    i2c_tx_buffer[0] = 0x3B;
	i2c_tx_buffer_counter = 1;
	i2c_transmit();
	i2c_multireceive(6);

/* These are div 16384 if +/-2g, 8192 if +/-4g, 4096 if +/-8g and 2048 if +/-16g*/
    ax = (i2c_rx_buffer[0]<<8 | i2c_rx_buffer[1]);
    ay = (i2c_rx_buffer[2]<<8 | i2c_rx_buffer[3]);
    az = (i2c_rx_buffer[4]<<8 | i2c_rx_buffer[5]);

    sprintf(tempbuf,"E%d,%d,%d\r\n",ax,ay,az);
    hc05_transmit(tempbuf,strlen(tempbuf));
}

void mpu6050_gyro() {
	i2c_tx_buffer[0] = 0x43;
	i2c_tx_buffer_counter = 1;
	i2c_transmit();
	i2c_multireceive(6);

    gx = (i2c_rx_buffer[0]<<8 | i2c_rx_buffer[1]);
    gy = (i2c_rx_buffer[2]<<8 | i2c_rx_buffer[3]);
    gz = (i2c_rx_buffer[4]<<8 | i2c_rx_buffer[5]);
    sprintf(tempbuf,"%d %d %d\r\n",gx,gy,gz);
    hc05_transmit(tempbuf,strlen(tempbuf));
}


void mpu6050_temp() {
    i2c_tx_buffer[0] = 0x41;
	i2c_tx_buffer_counter = 1;
	i2c_transmit();
	i2c_multireceive(2);

    temperature = i2c_rx_buffer[0]<<8 | i2c_rx_buffer[1];
    temperature = (temperature/340) + 37;
    sprintf(tempbuf,"%d\r\n",temperature);
    hc05_transmit(tempbuf,strlen(tempbuf));
}

void mpu6050_configAccel(uint8_t accel_config) {
    i2c_tx_buffer[1] = MPU6050_RA_ACCEL_CONFIG;
	i2c_tx_buffer[0] = accel_config;
	i2c_tx_buffer_counter =2;
	i2c_transmit();
}

void mpu6050_calibrate_gyros()
{
	int x = 0;
	int32_t gx_1000sum = 0;
	int32_t gy_1000sum = 0;
	int32_t gz_1000sum = 0;

	for(x = 0; x<1024; x++)
	{
        i2c_tx_buffer[0] = 0x43;
        i2c_tx_buffer_counter = 1;
        i2c_transmit();
        i2c_multireceive(6);
        gx_1000sum += (i2c_rx_buffer[0]<<8 | i2c_rx_buffer[1]);
        gy_1000sum += (i2c_rx_buffer[2]<<8 | i2c_rx_buffer[3]);
        gz_1000sum += (i2c_rx_buffer[4]<<8 | i2c_rx_buffer[5]);

		delay_ms(1);
	}
	gx = gx_1000sum/1024;
	gy = gy_1000sum/1024;
	gz = gz_1000sum/1024;

	sprintf(tempbuf,"Gyro X offset sum: %ld Gyro X offset: %d\r\n", gx_1000sum,gx);
	hc05_transmit(tempbuf,strlen(tempbuf));
	sprintf(tempbuf,"Gyro Y offset sum: %ld Gyro Y offset: %d\r\n", gy_1000sum,gy);
	hc05_transmit(tempbuf,strlen(tempbuf));
	sprintf(tempbuf,"Gyro Z offset sum: %ld Gyro Z offset: %d\r\n", gz_1000sum,gz);
    hc05_transmit(tempbuf,strlen(tempbuf));
}
