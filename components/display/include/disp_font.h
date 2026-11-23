/*
 * disp_font.h，8x16 点阵字库声明（数据在 disp_font.c）
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DISP_FONT_W     8    /* 每个字符宽 8 像素 */
#define DISP_FONT_H     16   /* 每个字符高 16 像素（上半 8 + 下半 8） */
#define DISP_FONT_FIRST 0x20 /* 第一个字符：空格 */
#define DISP_FONT_LAST  0x7E /* 最后一个字符：波浪号（0x20~0x7E 共 95 个） */

/* 95 个字符 × 16 字节 */
extern const uint8_t disp_font8x16[][16];

#ifdef __cplusplus
}
#endif