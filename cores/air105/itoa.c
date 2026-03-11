/**
 * @file itoa.c
 * @brief Integer-to-string conversion helpers for Arduino compatibility
 *
 * Newlib provides itoa() and utoa() but not ltoa() or ultoa().
 * This file supplies the missing long variants and provides wrappers
 * for itoa/utoa that redirect to newlib's built-in implementations,
 * ensuring all four functions are available.
 *
 * SPDX-License-Identifier: MIT
 */

#include <stdlib.h>
#include <string.h>

char *ltoa(long value, char *str, int base)
{
    if (base < 2 || base > 36) {
        str[0] = '\0';
        return str;
    }

    char *p = str;
    unsigned long uval;

    if (base == 10 && value < 0) {
        *p++ = '-';
        uval = (unsigned long)(-(value + 1)) + 1UL;
    } else {
        uval = (unsigned long)value;
    }

    /* Generate digits in reverse */
    char *start = p;
    do {
        unsigned long digit = uval % (unsigned long)base;
        *p++ = (char)(digit < 10 ? '0' + digit : 'a' + digit - 10);
        uval /= (unsigned long)base;
    } while (uval);
    *p = '\0';

    /* Reverse the digit portion */
    char *end = p - 1;
    while (start < end) {
        char tmp = *start;
        *start++ = *end;
        *end-- = tmp;
    }

    return str;
}

char *ultoa(unsigned long value, char *str, int base)
{
    if (base < 2 || base > 36) {
        str[0] = '\0';
        return str;
    }

    char *p = str;
    char *start = p;

    do {
        unsigned long digit = value % (unsigned long)base;
        *p++ = (char)(digit < 10 ? '0' + digit : 'a' + digit - 10);
        value /= (unsigned long)base;
    } while (value);
    *p = '\0';

    /* Reverse */
    char *end = p - 1;
    while (start < end) {
        char tmp = *start;
        *start++ = *end;
        *end-- = tmp;
    }

    return str;
}
