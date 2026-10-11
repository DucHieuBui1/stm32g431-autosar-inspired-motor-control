#ifndef PLATFORM_TYPES_H
#define PLATFORM_TYPES_H

#include <stdint.h>

/* Fixed-width platform types for the 32-bit little-endian Cortex-M4 target. */
typedef uint8_t  boolean;
typedef int8_t   sint8;
typedef uint8_t  uint8;
typedef int16_t  sint16;
typedef uint16_t uint16;
typedef int32_t  sint32;
typedef uint32_t uint32;
typedef int64_t  sint64;
typedef uint64_t uint64;
typedef float    float32;
typedef double   float64;

#define CPU_TYPE_8          8U
#define CPU_TYPE_16         16U
#define CPU_TYPE_32         32U
#define CPU_TYPE_64         64U

#define MSB_FIRST           0U
#define LSB_FIRST           1U
#define HIGH_BYTE_FIRST     0U
#define LOW_BYTE_FIRST      1U

#define CPU_TYPE            CPU_TYPE_32
#define CPU_BIT_ORDER       LSB_FIRST
#define CPU_BYTE_ORDER      LOW_BYTE_FIRST

#ifndef TRUE
#define TRUE                ((boolean)1U)
#endif

#ifndef FALSE
#define FALSE               ((boolean)0U)
#endif

#endif /* PLATFORM_TYPES_H */
