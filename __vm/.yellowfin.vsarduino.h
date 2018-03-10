#pragma once

#if defined(__INTELLISENSE__)

#undef __cplusplus
#define __cplusplus 201703L
#undef __inline__
#define __inline__
#undef __asm__
#define __asm__(...)
#undef __extension__
#define __extension__
#undef __volatile__
#define __volatile__(...)

#undef volatile
#define volatile(...) 
#undef __builtin_va_start
#define __builtin_va_start
#undef __builtin_va_end
#define __builtin_va_end
#undef __attribute__
#define __attribute__(...)
#undef NOINLINE
#define NOINLINE
#undef prog_void
#define prog_void

#undef PGM_VOID_P
using PGM_VOID_P = void *;


#undef __builtin_constant_p
#define __builtin_constant_p(e) true
#undef __builtin_strlen
#define __builtin_strlen(e) strlen(e)
#undef __builtin_unreachable
#define __builtin_unreachable
#undef __builtin_expect
#define __builtin_expect(e, v) e

#undef NEW_H
#define NEW_H

#undef __builtin_va_list
using __builtin_va_list = void *;

#undef div_t
using div_t = int;
#undef ldiv_t
using ldiv_t = int;

#undef __builtin_va_list
using __builtin_va_list = void *;



//#include <arduino.h>
//#include <pins_arduino.h> 
#undef F
#define F(string_literal) ((const char *)(string_literal))
#undef PSTR
#define PSTR(string_literal) ((const char *)(string_literal))

#define __INTMAX_C(c) c ## LL
#define __INT8_C(c) c
#define __INT64_C(c) c ## LL
#define __UINT16_C(c) c
#define __UINT64_C(c) c ## ULL
#define __INT32_C(c) c ## L
#define __UINTMAX_C(c) c ## ULL
#define __UINT8_C(c) c
#define __UINT32_C(c) c ## UL
#define __INT16_C(c) c

#define __null ((void *)0)

typedef unsigned char uint8_t;
typedef signed char int8_t;
typedef unsigned short uint16_t;
typedef signed short int16_t;

#endif
