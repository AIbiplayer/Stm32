/**
 * @file C51_bool.h
 * @brief C51 布尔类型定义
 * @date 2025-2-18
 * @author 杜以成
 */

#ifndef __C51_BOOL_H__
#define __C51_BOOL_H__

// 对于一般的非 C99 标准的编译器，可以使用以下方式实现布尔类型。
// 但是对于单片机开发而言，需要节省内存，因此不使用枚举实现。
// （枚举类型的大小等于 int 类型的大小）
// typedef enum bool {
//     false = 0,
//     true
// } bool;

// 为了节省内存，使用 bit 类型实现布尔类型。
// 在 C99 标准中，布尔类型关键字是 _Bool 而非 bool，
// 因此此处使用 _Bool 定义布尔类型。
typedef bit _Bool;

// 同时，参照 C99 标准的 stdbool.h 头文件，定义 bool 类型。
typedef _Bool bool;

#define false 0
#define true 1

#endif  // !__C51_BOOL_H__