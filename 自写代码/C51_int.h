/**
 * @file C51_int.h
 * @brief C51 固定宽度整型数据类型定义
 * @date 2025-2-18
 * @author 杜以成
 */

#ifndef __C51_INT_H__
#define __C51_INT_H__

typedef signed   char int8_t;
typedef signed   int  int16_t;
typedef signed   long int32_t;
typedef unsigned char uint8_t;
typedef unsigned int  uint16_t;
typedef unsigned long uint32_t;

typedef signed   char int_least8_t;
typedef signed   int  int_least16_t;
typedef signed   long int_least32_t;
typedef unsigned char uint_least8_t;
typedef unsigned int  uint_least16_t;
typedef unsigned long uint_least32_t;

typedef signed   char int_fast8_t;
typedef signed   int  int_fast16_t;
typedef signed   long int_fast32_t;
typedef unsigned char uint_fast8_t;
typedef unsigned int  uint_fast16_t;
typedef unsigned long uint_fast32_t;

// C51 不符合 C99 标准，没有宽度为 64 位的整型数据类型，
// 因此没有 int64_t、int_least64_t、int_fast64_t 等类型。

// intx_t uintx_t

#define INT8_MIN   (-128)
#define INT8_MAX   (127)
#define UINT8_MAX  (255U)

#define INT16_MIN  (-32768)
#define INT16_MAX  (32767)
#define UINT16_MAX (65535U)

#define INT32_MIN  (-2147483648)
#define INT32_MAX  (2147483647)
#define UINT32_MAX (4294967295U)

// int_leastx_t uint_leastx_t

#define INT_LEAST8_MIN   INT8_MIN
#define INT_LEAST8_MAX   INT8_MAX
#define UINT_LEAST8_MAX  UINT8_MAX

#define INT_LEAST16_MIN  INT16_MIN
#define INT_LEAST16_MAX  INT16_MAX
#define UINT_LEAST16_MAX UINT16_MAX

#define INT_LEAST32_MIN  INT32_MIN
#define INT_LEAST32_MAX  INT32_MAX
#define UINT_LEAST32_MAX UINT32_MAX

// int_fastx_t uint_fastx_t

#define INT_FAST8_MIN    INT8_MIN
#define INT_FAST8_MAX    INT8_MAX
#define UINT_FAST8_MAX   UINT8_MAX

#define INT_FAST16_MIN   INT16_MIN
#define INT_FAST16_MAX   INT16_MAX
#define UINT_FAST16_MAX  UINT16_MAX

#define INT_FAST32_MIN   INT32_MIN
#define INT_FAST32_MAX   INT32_MAX
#define UINT_FAST32_MAX  UINT32_MAX

#endif  // !__C51_INT_H__
