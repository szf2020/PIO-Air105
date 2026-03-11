/**
 * @file avr/pgmspace.h
 * @brief AVR pgmspace.h compatibility for Air105 (ARM Cortex-M4)
 *
 * On ARM, flash and RAM share a single linear 32-bit address space, so
 * PROGMEM data is accessed with normal pointer dereferences — no special
 * instructions (LPM) are required.  All macros and functions here are
 * pass-through / no-op implementations that let AVR-targeted Arduino
 * libraries and sketches compile and run unmodified.
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef __PGMSPACE_H_
#define __PGMSPACE_H_

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================== */
/*  Storage qualifiers (no-op on ARM)                                 */
/* ================================================================== */
#define PROGMEM
#define PGM_P        const char *
#define PGM_VOID_P   const void *
#define PSTR(s)      (s)

/* ================================================================== */
/*  pgm_read_xxx — read data from "program memory"                    */
/*  On ARM these are plain pointer dereferences.                      */
/* ================================================================== */
#define pgm_read_byte(addr)        (*(const uint8_t  *)(addr))
#define pgm_read_word(addr)        (*(const uint16_t *)(addr))
#define pgm_read_dword(addr)       (*(const uint32_t *)(addr))
#define pgm_read_float(addr)       (*(const float    *)(addr))
#define pgm_read_ptr(addr)         (*(const void * const *)(addr))

/* _near / _far variants (AVR-specific, same on ARM) */
#define pgm_read_byte_near(addr)   pgm_read_byte(addr)
#define pgm_read_word_near(addr)   pgm_read_word(addr)
#define pgm_read_dword_near(addr)  pgm_read_dword(addr)
#define pgm_read_float_near(addr)  pgm_read_float(addr)
#define pgm_read_ptr_near(addr)    pgm_read_ptr(addr)
#define pgm_read_byte_far(addr)    pgm_read_byte(addr)
#define pgm_read_word_far(addr)    pgm_read_word(addr)
#define pgm_read_dword_far(addr)   pgm_read_dword(addr)
#define pgm_read_float_far(addr)   pgm_read_float(addr)
#define pgm_read_ptr_far(addr)     pgm_read_ptr(addr)

/* ================================================================== */
/*  String functions — _P variants map to standard C equivalents      */
/* ================================================================== */
#define memcpy_P(dest, src, n)     memcpy((dest), (src), (n))
#define memmem_P(h, hl, n, nl)    memmem((h), (hl), (n), (nl))
#define memcmp_P(s1, s2, n)       memcmp((s1), (s2), (n))
#define strcpy_P(dest, src)        strcpy((dest), (src))
#define strncpy_P(dest, src, n)    strncpy((dest), (src), (n))
#define strcat_P(dest, src)        strcat((dest), (src))
#define strncat_P(dest, src, n)    strncat((dest), (src), (n))
#define strcmp_P(s1, s2)           strcmp((s1), (s2))
#define strncmp_P(s1, s2, n)       strncmp((s1), (s2), (n))
#define strcasecmp_P(s1, s2)       strcasecmp((s1), (s2))
#define strncasecmp_P(s1, s2, n)   strncasecmp((s1), (s2), (n))
#define strlen_P(s)                strlen(s)
#define strnlen_P(s, n)            strnlen((s), (n))
#define strstr_P(h, n)             strstr((h), (n))
#define sprintf_P(buf, fmt, ...)   sprintf((buf), (fmt), ##__VA_ARGS__)
#define snprintf_P(b, s, f, ...)   snprintf((b), (s), (f), ##__VA_ARGS__)
#define vsnprintf_P(b, s, f, a)    vsnprintf((b), (s), (f), (a))
#define printf_P(fmt, ...)         printf((fmt), ##__VA_ARGS__)

/* ================================================================== */
/*  pgm_get_far_address — identity on ARM (no 16-bit address limit)   */
/* ================================================================== */
#define pgm_get_far_address(var)   ((uint32_t)(&(var)))

#ifdef __cplusplus
}
#endif

#endif /* __PGMSPACE_H_ */
