#include "tinympc14_data.h"

/* Embedded in the scheduler translation unit after tinympc14_data.h.
 * The generated grid bounds restrict node to [0, nAngle * planeNodes).
 * The encoder checks each prefix width (0..17), each stride and the last
 * byte of every plane. A trailing zero word makes the final preload valid. */
static void tinympc14_terminal_decode_node(unsigned int node, short *out)
{
    unsigned int plane = node / TINYMPC14_COORD_PLANE_NODES;
    unsigned int local_node = node % TINYMPC14_COORD_PLANE_NODES;
    const short *anchor = &tinympc14_coord_anchor[local_node * TINYMPC14_COORD_RANK];
    unsigned int c;
    if (plane == TINYMPC14_COORD_ANCHOR_PLANE) {
        for (c = 0; c < TINYMPC14_COORD_RANK; ++c) {
            out[c] = anchor[c];
        }
    } else {
        unsigned int index = plane - (plane > TINYMPC14_COORD_ANCHOR_PLANE);
        const unsigned short *prefix = &tinympc14_coord_bit_offsets[index * (TINYMPC14_COORD_RANK + 1U)];
        unsigned int stride = (prefix[TINYMPC14_COORD_RANK] + 7U) / 8U;
        unsigned int offset = tinympc14_coord_plane_offsets[index] + local_node * stride;
        const unsigned int *src = tinympc14_coord_payload + offset / 4U;
        unsigned int bit = (offset % 4U) * 8U;
        unsigned int word = *src++;
        for (c = 0; c < TINYMPC14_COORD_RANK; ++c) {
            unsigned int width = prefix[c + 1U] - prefix[c];
            unsigned int next = bit + width;
            unsigned int zigzag = word >> bit;
            int delta;
            if (next >= 32U) {
                word = *src++;
                if (next > 32U) {
                    zigzag |= word << (32U - bit);
                }
                next -= 32U;
            }
            bit = next;
            zigzag &= (1U << width) - 1U;
            delta = (int)(zigzag >> 1);
            if (zigzag & 1U) {
                delta = -1 - delta;
            }
            out[c] = (short)((int)anchor[c] + delta);
        }
    }
}

#ifdef TINYMPC14_TERMINAL_CODEC_TEST
void Tinympc14_Test_Decode_Node(unsigned int node, short *out)
{
    tinympc14_terminal_decode_node(node, out);
}
#endif

/*
 * File: mldivide.c
 *
 * MATLAB Coder version            : 24.1

 */

/* Include Files */
/*
 * File: mldivide.h
 *
 * MATLAB Coder version            : 24.1

 */

#ifndef MLDIVIDE_H
#define MLDIVIDE_H

/* Include Files */
/*
 * File: rtwtypes.h
 *
 * MATLAB Coder version            : 24.1

 */

#ifndef RTWTYPES_H
#define RTWTYPES_H

/*=======================================================================*
 * Fixed width word size data types:                                     *
 *   int64_T                      - signed 64 bit integers               *
 *   uint64_T                     - unsigned 64 bit integers             *
 *=======================================================================*/

#if defined(__APPLE__)
#ifndef INT64_T
#define INT64_T long
#define FMT64 "l"
#if defined(__LP64__) && !defined(INT_TYPE_64_IS_LONG)
#define INT_TYPE_64_IS_LONG
#endif
#endif
#endif

#if defined(__APPLE__)
#ifndef UINT64_T
#define UINT64_T unsigned long
#define FMT64 "l"
#if defined(__LP64__) && !defined(INT_TYPE_64_IS_LONG)
#define INT_TYPE_64_IS_LONG
#endif
#endif
#endif

/* Include Files */
/*
 * Copyright 1984-2018 The MathWorks, Inc.
 */

#if defined(_MSC_VER)

#endif
#if defined(__GNUC__) && (__GNUC__ > 3 || (__GNUC__ == 3 && __GNUC_MINOR__ > 3))

#endif

#ifndef tmwtypes_h
#define tmwtypes_h

#ifndef __TMWTYPES__
#define __TMWTYPES__
/*
 * File    : tmwtypes.h
 * Abstract:
 *      Data types for use with MATLAB/SIMULINK and the Real-Time Workshop.
 *
 *      When compiling stand-alone model code, data types can be overridden
 *      via compiler switches.
 *
 *      Define NO_FLOATS to eliminate reference to real_T, etc.
 */

#ifdef MW_LIBTOOLING
#include "mwstdint.h"
#endif

#include <limits.h>

/* __STDC_VERSION__ version check below means "check for a C99 compiler".

   Visual Studio (checked on versions 2015 and 2017) does
   not define __STDC_VERSION__, however  it has stdbool.h available,
   thus a separate check for _MSC_VER below.
 */
#if defined(__APPLE_CC__) \
    || (defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 199901L)) \
    || (defined(_MSC_VER) && (_MSC_VER >= 1900))
#ifndef tmwtypes_do_not_include_stdbool
#include <stdbool.h>
#endif
#endif

#define LOGICAL_IS_A_TYPE
#define SPARSE_GENERALIZATION

#ifdef NO_FLOATS
# define double double_not_allowed
# define float  float_not_allowed
#endif /*NO_FLOATS*/

#ifndef NO_FLOATS

#ifndef __MWERKS__
# ifdef __STDC__
#  include <float.h>
# else
#  ifndef FLT_MANT_DIG
#   define FLT_MANT_DIG 24
#  endif
#  ifndef DBL_MANT_DIG
#   define DBL_MANT_DIG 53
#  endif
# endif
#endif

#endif /*NO_FLOATS*/

/*
 *      The following data types cannot be overridden when building MEX files.
 */
#ifdef MATLAB_MEX_FILE
# undef CHARACTER_T
# undef INTEGER_T
# undef BOOLEAN_T
# undef REAL_T
# undef TIME_T
#endif

/*
 * The uchar_T, ushort_T and ulong_T types are needed for compilers which do 
 * not allow defines to be specified, at the command line, with spaces in them.
 */

typedef unsigned char       uchar_T;
typedef unsigned short      ushort_T;
typedef unsigned long       ulong_T;

#if (defined(_MSC_VER) && _MSC_VER >= 1500)      \
    || defined(__x86_64__) || defined(__LP64__)  \
    || defined(__LCC64__)
    
typedef unsigned long long  ulonglong_T;
#endif



/*=======================================================================*
 * Fixed width word size data types:                                     *
 *   int8_T, int16_T, int32_T     - signed 8, 16, or 32 bit integers     *
 *   uint8_T, uint16_T, uint32_T  - unsigned 8, 16, or 32 bit integers   *
 *   real32_T, real64_T           - 32 and 64 bit floating point numbers *
 *=======================================================================*/

/* When used with Real Time Workshop generated code, this
 * header file can be used with a variety of compilers.
 *
 * The compiler could be for an 8 bit embedded processor that
 * only had 8 bits per integer and 16 bits per long.
 * In that example, a 32 bit integer size is not even available.
 * This header file should be robust to that.   
 *
 * For the case of an 8 bit processor, the preprocessor
 * may be limited to 16 bit math like its target.  That limitation 
 * would mean that 32 bit comparisons can't be done accurately.  
 * To increase robustness to this, comparisons are done against
 * smaller values first.  An inaccurate 32 bit comparison isn't
 * attempted if the 16 bit comparison has already succeeded.
 *
 * Limitations on preprocessor math can also be stricter than
 * for the target.  There are known cases where a compiler
 * targeting processors with 64 bit longs can't do accurate
 * preprocessor comparisons on more than 32 bits.  
 */

/* Determine the number of bits for int, long, short, and char.
 * If one fails to be determined, set the number of bits to -1
 */

#ifndef TMW_BITS_PER_INT
# if   INT_MAX  == 0x7FL
#  define TMW_BITS_PER_INT 8
# elif INT_MAX  == 0x7FFFL
#  define TMW_BITS_PER_INT 16
# elif INT_MAX  == 0x7FFFFFFFL
#  define TMW_BITS_PER_INT 32
# else
#  define TMW_BITS_PER_INT -1
# endif
#endif

#ifndef TMW_BITS_PER_LONG
# if   LONG_MAX  == 0x7FL
#  define TMW_BITS_PER_LONG 8
# elif LONG_MAX  == 0x7FFFL
#  define TMW_BITS_PER_LONG 16
# elif LONG_MAX  == 0x7FFFFFFFL
#  define TMW_BITS_PER_LONG 32
# else
#  define TMW_BITS_PER_LONG -1
# endif
#endif

#ifndef TMW_BITS_PER_SHRT
# if   SHRT_MAX  == 0x7FL
#  define TMW_BITS_PER_SHRT 8
# elif SHRT_MAX  == 0x7FFFL
#  define TMW_BITS_PER_SHRT 16
# elif SHRT_MAX  == 0x7FFFFFFFL
#  define TMW_BITS_PER_SHRT 32
# else
#  define TMW_BITS_PER_SHRT -1
# endif
#endif

#ifndef TMW_BITS_PER_SCHAR
# if   SCHAR_MAX  == 0x7FL
#  define TMW_BITS_PER_SCHAR 8
# elif SCHAR_MAX  == 0x7FFFL
#  define TMW_BITS_PER_SCHAR 16
# elif SCHAR_MAX  == 0x7FFFFFFFL
#  define TMW_BITS_PER_SCHAR 32
# else
#  define TMW_BITS_PER_SCHAR -1
# endif
#endif

#ifndef TMW_CHAR_SIGNED
# if SCHAR_MAX == CHAR_MAX
#  define TMW_CHAR_SIGNED 1
# else
#  define TMW_CHAR_SIGNED 0
# endif
#endif

/* It is common for one or more of the integer types
 * to be the same size.  For example, on many embedded
 * processors, both shorts and ints are 16 bits.  On
 * processors used for workstations, it is quite common
 * for both int and long to be 32 bits.  
 *   When there is more than one choice for typdef'ing
 * a portable type like int16_T or uint32_T, in
 * concept, it should not matter which choice is made.
 * However, some style guides and some code checking
 * tools do identify and complain about seemingly
 * irrelevant differences.  For example, a code
 * checking tool may complain about an implicit
 * conversion from int to short even though both
 * are 16 bits.  To reduce these types of
 * complaints, it is best to make int the
 * preferred choice when more than one is available.
 */

#ifndef INT8_T
# if   defined(MW_LIBTOOLING)
#  define INT8_T  int8_t
# elif TMW_BITS_PER_INT   == 8
#  define  INT8_T int
# elif TMW_BITS_PER_LONG  == 8
#  define  INT8_T long
# elif TMW_BITS_PER_SCHAR == 8
#  define  INT8_T signed char
# elif TMW_BITS_PER_SHRT  == 8
#  define  INT8_T short
# endif
#endif
#ifdef INT8_T
 typedef INT8_T int8_T;
#endif

#ifndef UINT8_T
# if   defined(MW_LIBTOOLING)
#  define  UINT8_T uint8_t
# elif   TMW_BITS_PER_INT   == 8
#  define  UINT8_T unsigned int
# elif TMW_BITS_PER_LONG  == 8
#  define  UINT8_T unsigned long
# elif TMW_BITS_PER_SCHAR == 8
#  define  UINT8_T unsigned char
# elif TMW_BITS_PER_SHRT  == 8
#  define  UINT8_T unsigned short
# endif
#endif
#ifdef UINT8_T
 typedef UINT8_T uint8_T;
#endif


#ifndef INT16_T
# if   defined(MW_LIBTOOLING)
#  define  INT16_T int16_t
# elif   TMW_BITS_PER_INT   == 16
#  define  INT16_T int
# elif TMW_BITS_PER_LONG  == 16
#  define  INT16_T long
# elif TMW_BITS_PER_SCHAR == 16
#  define  INT16_T signed char
# elif TMW_BITS_PER_SHRT  == 16
#  define  INT16_T short
# endif
#endif
#ifdef INT16_T
 typedef INT16_T int16_T;
#endif


#ifndef UINT16_T
# if   defined(MW_LIBTOOLING)
#  define  UINT16_T uint16_t
# elif TMW_BITS_PER_INT   == 16
#  define  UINT16_T unsigned int
# elif TMW_BITS_PER_LONG  == 16
#  define  UINT16_T unsigned long
# elif TMW_BITS_PER_SCHAR == 16
#  define  UINT16_T unsigned char
# elif TMW_BITS_PER_SHRT  == 16
#  define  UINT16_T unsigned short
# endif
#endif
#ifdef UINT16_T
 typedef UINT16_T uint16_T;
#endif


#ifndef INT32_T
# if   defined(MW_LIBTOOLING)
#  define  INT32_T int32_t
# elif TMW_BITS_PER_INT   == 32
#  define  INT32_T int
# elif TMW_BITS_PER_LONG  == 32
#  define  INT32_T long
# elif TMW_BITS_PER_SCHAR == 32
#  define  INT32_T signed char
# elif TMW_BITS_PER_SHRT  == 32
#  define  INT32_T short
# endif
#endif
#ifdef INT32_T
 typedef INT32_T int32_T;
#endif


#ifndef UINT32_T
# if   defined(MW_LIBTOOLING)
#  define  UINT32_T uint32_t
# elif TMW_BITS_PER_INT   == 32
#  define  UINT32_T unsigned int
# elif TMW_BITS_PER_LONG  == 32
#  define  UINT32_T unsigned long
# elif TMW_BITS_PER_SCHAR == 32
#   define UINT32_T unsigned char
# elif TMW_BITS_PER_SHRT  == 32
#  define  UINT32_T unsigned short
# endif
#endif
#ifdef UINT32_T
 typedef UINT32_T uint32_T;
#endif

/* The following is used to emulate smaller integer types when only
 * larger types are available.  For example, compilers for TI C3x/C4x DSPs 
 * define char and short to be 32 bits, so 8 and 16 bits are not directly
 * available.  This target is commonly used with RTW rapid prototyping.
 * Other DSPs define char to be 16 bits, so 8 bits is not directly 
 * available.
 */
#ifndef INT8_T
# ifdef INT16_T
#  define INT8_T INT16_T
   typedef INT8_T int8_T;
# else
#  ifdef INT32_T
#   define INT8_T INT32_T
    typedef INT8_T int8_T;
#  endif
# endif
#endif

#ifndef UINT8_T
# ifdef UINT16_T
#   define UINT8_T UINT16_T
    typedef UINT8_T uint8_T;
# else
#  ifdef UINT32_T
#   define UINT8_T UINT32_T
    typedef UINT8_T uint8_T;
#  endif
# endif
#endif

#ifndef INT16_T
# ifdef INT32_T
#   define INT16_T INT32_T
    typedef INT16_T int16_T;
# endif
#endif

#ifndef UINT16_T
# ifdef UINT32_T
#   define UINT16_T UINT32_T
    typedef UINT16_T uint16_T;
# endif
#endif


#ifndef NO_FLOATS

#ifndef REAL32_T
# ifndef __MWERKS__
#  if FLT_MANT_DIG >= 23
#   define REAL32_T float
#  endif
# else
#  define REAL32_T float
# endif
#endif
#ifdef REAL32_T
 typedef REAL32_T real32_T;
#endif


#ifndef REAL64_T
# ifndef __MWERKS__
#  if DBL_MANT_DIG >= 52
#   define REAL64_T double
#  endif
# else
#  define REAL64_T double
# endif
#endif
#ifdef REAL64_T
 typedef REAL64_T real64_T;
#endif

#endif /* NO_FLOATS*/

/*=======================================================================*
 * Fixed width word size data types:                                     *
 *   int64_T                      - signed 64 bit integers               *
 *   uint64_T                     - unsigned 64 bit integers             *
 *=======================================================================*/

# if   defined(MW_LIBTOOLING)
#  ifdef INT64_T
#    undef  INT64_T
#  endif
#  define  INT64_T int64_t
#  ifdef UINT64_T
#    undef  UINT64_T
#  endif
#  define  UINT64_T uint64_t
# endif
#if !defined(INT64_T) || !defined(UINT64_T) || !defined(FMT64)
# if defined(__APPLE__) || defined(__clang__)
#  ifndef INT64_T
#   define INT64_T long long
#  endif
#  ifndef UINT64_T
#   define UINT64_T unsigned long long
#  endif
#  ifndef FMT64
#   define FMT64 "ll"
#  endif
#  if defined(__LP64__) && !defined(INT_TYPE_64_IS_LONG)
#    define INT_TYPE_64_IS_LONG
#  endif
# elif (defined(__x86_64__) || defined(__LP64__))&& !defined(__MINGW64__) 
#  ifndef INT64_T
#   define INT64_T long
#  endif
#  ifndef UINT64_T
#   define UINT64_T unsigned long
#  endif
#  ifndef FMT64
#   define FMT64 "l"
#  endif
#  if !defined(INT_TYPE_64_IS_LONG)
#    define INT_TYPE_64_IS_LONG
#  endif
# elif defined(_MSC_VER) || (defined(__BORLANDC__) && __BORLANDC__ >= 0x530) \
                         || (defined(__WATCOMC__)  && __WATCOMC__  >= 1100)
#  ifndef INT64_T
#   define INT64_T __int64
#  endif
#  ifndef UINT64_T
#   define UINT64_T unsigned __int64
#  endif
#  ifndef FMT64
#   define FMT64 "I64"
#  endif
# elif defined(__GNUC__) || defined(TMW_ENABLE_INT64) \
                         || defined(__LCC64__)
#  ifndef INT64_T
#   define INT64_T long long
#  endif
#  ifndef UINT64_T
#   define UINT64_T unsigned long long
#  endif
#  ifndef FMT64
#   define FMT64 "ll"
#  endif
# endif

#endif

#if defined(INT64_T)
# if defined(__GNUC__) && \
    ((__GNUC__ > 2) || ((__GNUC__ == 2) && (__GNUC_MINOR__ >=9)))
  __extension__
# endif
 typedef INT64_T int64_T;
#endif

#if defined(_WIN64) || (defined(__APPLE__) && defined(__LP64__)) \
                    || defined(__x86_64__) \
                    || defined(__LP64__)
#  define INT_TYPE_64_IS_SUPPORTED
#endif

#if defined(UINT64_T)
# if defined(__GNUC__) && \
    ((__GNUC__ > 2) || ((__GNUC__ == 2) && (__GNUC_MINOR__ >=9)))
  __extension__
# endif
 typedef UINT64_T uint64_T;
#endif

/*===========================================================================*
 * Format string modifiers for using size_t variables in printf statements.  *
 *===========================================================================*/

#ifndef FMT_SIZE_T
#  if (defined( __GNUC__ ) || defined(_STDC_C99))&& !defined(__MINGW64__)
#    define FMT_SIZE_T "z"
#  elif defined (__WATCOMC__)
#    define FMT_SIZE_T "l"
#  elif defined (_WIN32 )
#    define FMT_SIZE_T "I"
#  else
#    define FMT_SIZE_T "l"
#  endif
#endif

#ifndef FMT_PTRDIFF_T
#  if defined(__APPLE__)
#    define FMT_PTRDIFF_T "l"
#  elif defined( __GNUC__ ) || defined(_STDC_C99)
#    define FMT_PTRDIFF_T "t"
#  elif defined (__WATCOMC__)
#    define FMT_PTRDIFF_T "l"
#  elif defined (_WIN32 )
#    define FMT_PTRDIFF_T "I"
#  else
#    define FMT_PTRDIFF_T "l"
#  endif
#endif

/*===========================================================================*
 * General or logical data types where the word size is not guaranteed.      *
 *  real_T     - possible settings include real32_T or real64_T              *
 *  time_T     - possible settings include real32_T or real64_T              *
 *  boolean_T                                                                *
 *  char_T                                                                   *
 *  int_T                                                                    *
 *  uint_T                                                                   *
 *  byte_T                                                                   *
 *===========================================================================*/

#ifndef NO_FLOATS

#ifndef REAL_T
# ifdef REAL64_T
#  define REAL_T real64_T
# else
#  ifdef REAL32_T
#   define REAL_T real32_T
#  endif
# endif
#endif
#ifdef REAL_T
 typedef REAL_T real_T;
#endif

#ifndef TIME_T
# ifdef REAL_T
#  define TIME_T real_T
# endif
#endif
#ifdef TIME_T
 typedef TIME_T time_T;
#endif

#endif /* NO_FLOATS */

#ifndef BOOLEAN_T
# if defined(UINT8_T)
#  define BOOLEAN_T UINT8_T
# else
#  define BOOLEAN_T unsigned int
# endif
#endif
typedef BOOLEAN_T boolean_T;


#ifndef CHARACTER_T
# define CHARACTER_T char
#endif
typedef CHARACTER_T char_T;


#ifndef INTEGER_T
# define INTEGER_T int
#endif
typedef INTEGER_T int_T;


#ifndef UINTEGER_T
# define UINTEGER_T unsigned
#endif
typedef UINTEGER_T uint_T;


#ifndef BYTE_T
# define BYTE_T unsigned char
#endif
typedef BYTE_T byte_T;


/*===========================================================================*
 * Define Complex Structures                                                 *
 *===========================================================================*/
#ifndef NO_FLOATS

#ifndef CREAL32_T
#  ifdef REAL32_T
    typedef struct {
      real32_T re, im;
    } creal32_T;
#    define CREAL32_T creal32_T
#  endif
#endif

#ifndef CREAL64_T
#  ifdef REAL64_T
    typedef struct {
      real64_T re, im;
    } creal64_T;
#    define CREAL64_T creal64_T
#  endif
#endif

#ifndef CREAL_T
#  ifdef REAL_T
    typedef struct {
      real_T re, im;
    } creal_T;
#    define CREAL_T creal_T
#  endif
#endif

#endif /* NO_FLOATS */

#ifndef CINT8_T
#  ifdef INT8_T
    typedef struct {
      int8_T re, im;
    } cint8_T;
#    define CINT8_T cint8_T
#  endif
#endif

#ifndef CUINT8_T
#  ifdef UINT8_T
    typedef struct {
      uint8_T re, im;
    } cuint8_T;
#    define CUINT8_T cuint8_T
#  endif
#endif

#ifndef CINT16_T
#  ifdef INT16_T
    typedef struct {
      int16_T re, im;
    } cint16_T;
#    define CINT16_T cint16_T
#  endif
#endif

#ifndef CUINT16_T
#  ifdef UINT16_T
    typedef struct {
      uint16_T re, im;
    } cuint16_T;
#    define CUINT16_T cuint16_T
#  endif
#endif

#ifndef CINT32_T
#  ifdef INT32_T
    typedef struct {
      int32_T re, im;
    } cint32_T;
#    define CINT32_T cint32_T
#  endif
#endif

#ifndef CUINT32_T
#  ifdef UINT32_T
    typedef struct {
      uint32_T re, im;
    } cuint32_T;
#    define CUINT32_T cuint32_T
#  endif
#endif

#ifndef CINT64_T
#  ifdef INT64_T
    typedef struct {
      int64_T re, im;
    } cint64_T;
#    define CINT64_T cint64_T
#  endif
#endif

#ifndef CUINT64_T
#  ifdef UINT64_T
    typedef struct {
      uint64_T re, im;
    } cuint64_T;
#    define CUINT64_T cuint64_T
#  endif
#endif

/*=======================================================================*
 * Min and Max:                                                          *
 *   int8_T, int16_T, int32_T     - signed 8, 16, or 32 bit integers     *
 *   uint8_T, uint16_T, uint32_T  - unsigned 8, 16, or 32 bit integers   *
 *=======================================================================*/

#define  MAX_int8_T      ((int8_T)(127))            /* 127  */
#define  MIN_int8_T      ((int8_T)(-128))           /* -128 */
#define  MAX_uint8_T     ((uint8_T)(255))           /* 255  */
#define  MIN_uint8_T     ((uint8_T)(0))
                           
#define  MAX_int16_T     ((int16_T)(32767))         /* 32767 */
#define  MIN_int16_T     ((int16_T)(-32768))        /* -32768 */
#define  MAX_uint16_T    ((uint16_T)(65535))        /* 65535 */
#define  MIN_uint16_T    ((uint16_T)(0))
                           
#define  MAX_int32_T     ((int32_T)(2147483647))    /* 2147483647  */
#define  MIN_int32_T     ((int32_T)(-2147483647-1)) /* -2147483648 */
#define  MAX_uint32_T    ((uint32_T)(0xFFFFFFFFU))  /* 4294967295  */
#define  MIN_uint32_T    ((uint32_T)(0))

#if defined(_MSC_VER) || (defined(__BORLANDC__) && __BORLANDC__ >= 0x530) \
                      || (defined(__WATCOMC__)  && __WATCOMC__  >= 1100) \
                      || defined(__LCC64__)
#  ifdef INT64_T
#    define  MAX_int64_T     ((int64_T)(9223372036854775807LL))
#    define  MIN_int64_T     ((int64_T)(-9223372036854775807LL-1LL))
#  endif
#  ifdef UINT64_T
#    define  MAX_uint64_T    ((uint64_T)(0xFFFFFFFFFFFFFFFFULL))
#    define  MIN_uint64_T    ((uint64_T)(0))
#  endif
#else
#  ifdef INT64_T
#    ifdef INT_TYPE_64_IS_LONG
#      define  MAX_int64_T     ((int64_T)(9223372036854775807L))
#      define  MIN_int64_T     ((int64_T)(-9223372036854775807L-1L))
#    else
#      define  MAX_int64_T     ((int64_T)(9223372036854775807LL))
#      define  MIN_int64_T     ((int64_T)(-9223372036854775807LL-1LL))
#    endif
#  endif
#  ifdef UINT64_T
#    ifdef INT_TYPE_64_IS_LONG
#      define  MAX_uint64_T    ((uint64_T)(0xFFFFFFFFFFFFFFFFUL))
#      define  MIN_uint64_T    ((uint64_T)(0))
#    else
#      define  MAX_uint64_T    ((uint64_T)(0xFFFFFFFFFFFFFFFFULL))
#      define  MIN_uint64_T    ((uint64_T)(0))
#    endif
#  endif
#endif

#if (defined(_MSC_VER) && !defined(__clang__))

/* Conversion from unsigned __int64 to double is not implemented in Visual Studio
 * and results in a compile error, thus the value must first be cast to
 * signed __int64, and then to double.
 *
 * If the 64 bit int value is greater than 2^63-1, which is the signed int64 max,
 * the macro below provides a workaround for casting a uint64 value to a double
 * in windows.
 */
#  define uint64_to_double(u) ( ((u) > _I64_MAX) ? \
            (double)(__int64)((u) - _I64_MAX - 1) + (double)_I64_MAX + 1: \
            (double)(__int64)(u) )

/* The following inline function should only be used in the macro double_to_uint64,
 * as it only handles the specfic range of double between 2^63 and 2^64-1 */
__forceinline
uint64_T double_to_uint64_helper(double d) {
  union double_to_uint64_union_type {
    double dd;
    uint64_T i64;
  } di;
  di.dd = d;
  return (((di.i64 & 0x000fffffffffffff) | 0x0010000000000000) << 11);
}

/* The largest double value that can be cast to uint64 in windows is the
 * signed int64 max, which is 2^63-1. The macro below provides
 * a workaround for casting large double values to uint64 in windows.
 */
/* The magic number 18446744073709551616.0 is 2^64 */
/* The magic number 9223372036854775808.0 is 2^63 */
#  define double_to_uint64(d) ( ((d) >= 18446744073709551616.0) ? \
            0xffffffffffffffffULL : \
            ((d) >= 0.0) ? \
            ((d) >= 9223372036854775808.0) ? \
            double_to_uint64_helper(d) : \
            (unsigned __int64)(d) : \
            0ULL )
#else
#  define uint64_to_double(u) ((double)(u))
#  if defined(__BORLANDC__) || defined(__WATCOMC__) || defined(__TICCSC__)
/* double_to_uint64 defined only for MSVC and UNIX */
#  else
#  define double_to_uint64(d) ( ((d) >= 18446744073709551616.0) ? \
            (unsigned long long) 0xffffffffffffffffULL : \
            ((d) >= 0) ? (unsigned long long)(d) : (unsigned long long) 0 )
#  endif
#endif

#if !defined(__cplusplus) && !defined(__bool_true_false_are_defined)

#ifndef _bool_T
#define _bool_T

typedef boolean_T bool;

#ifndef false
#define false (0)
#endif
#ifndef true 
#define true (1)
#endif

#endif /* _bool_T */

#endif /* !__cplusplus */

/* 
 * This software assumes that the code is being compiled on a target using a 
 * 2's complement representation for signed integer values.
 */
#if ((SCHAR_MIN + 1) != -SCHAR_MAX)
#error "This code must be compiled using a 2's complement representation for signed integer values"
#endif

/*
 * Maximum length of a MATLAB identifier (function/variable/model)
 * including the null-termination character.
 */
#define TMW_NAME_LENGTH_MAX 64

/*
 * Maximum values for indices and dimensions
 */
#include <stddef.h>

#ifdef MX_COMPAT_32
typedef int mwSize;
typedef int mwIndex;
typedef int mwSignedIndex;
#else
typedef size_t    mwSize;         /* unsigned pointer-width integer */
typedef size_t    mwIndex;        /* unsigned pointer-width integer */
typedef ptrdiff_t mwSignedIndex;  /* a signed pointer-width integer */
#endif

                                  /* for the individual dim */
/* If updating SLSize or SLIndex, update defintions in sl_types_def.h
   as well. */
#ifndef SLSIZE_SLINDEX
  #define SLSIZE_SLINDEX
  #ifdef INT_TYPE_64_IS_SUPPORTED
    typedef int64_T SLIndex;
    typedef int64_T SLSize;
  #else
    typedef int SLIndex;
    typedef int SLSize;
  #endif
#endif

/* for the total size */
#define SLIndexType size_t
#define INVALID_SIZET_VALUE   (std::numeric_limits<SLIndexType>::max())
#define MAX_VALID_SIZET_VALUE   (std::numeric_limits<SLIndexType>::max() -1)


#if (defined(_LP64) || defined(_WIN64)) && !defined(MX_COMPAT_32)
/* Currently 2^48 based on hardware limitations */
# define MWSIZE_MAX    281474976710655UL
# define MWINDEX_MAX   281474976710655UL
# define MWSINDEX_MAX  281474976710655L
# define MWSINDEX_MIN -281474976710655L
#else
# define MWSIZE_MAX    2147483647UL
# define MWINDEX_MAX   2147483647UL
# define MWSINDEX_MAX  2147483647L
# define MWSINDEX_MIN -2147483647L
#endif
#define MWSIZE_MIN    0UL
#define MWINDEX_MIN   0UL

/** UTF-16 character type */

#if (defined(__cplusplus) && (__cplusplus >= 201103L)) || (defined(_HAS_CHAR16_T_LANGUAGE_SUPPORT) && _HAS_CHAR16_T_LANGUAGE_SUPPORT)
typedef char16_t CHAR16_T;
#define U16_STRING_LITERAL_PREFIX u
#elif defined(_MSC_VER)
typedef wchar_t CHAR16_T;
#define U16_STRING_LITERAL_PREFIX L
#else
typedef UINT16_T CHAR16_T;
#endif

#endif  /* __TMWTYPES__ */

#endif /* tmwtypes_h */


#endif
/*
 * File trailer for rtwtypes.h
 *
 * [EOF]
 */

#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void b_mldivide(const double A[25], const double B[20], double Y[20]);

void c_mldivide(const double A[25], double B[5]);

void mldivide(const double A[25], const double B[15], double Y[15]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for mldivide.h
 *
 * [EOF]
 */

/*
 * File: rt_nonfinite.h
 *
 * MATLAB Coder version            : 24.1

 */

#ifndef RT_NONFINITE_H
#define RT_NONFINITE_H

/* Include Files */


#ifdef __cplusplus
extern "C" {
#endif

extern real_T rtInf;
extern real_T rtMinusInf;
extern real_T rtNaN;
extern real32_T rtInfF;
extern real32_T rtMinusInfF;
extern real32_T rtNaNF;

extern boolean_T rtIsInf(real_T value);
extern boolean_T rtIsInfF(real32_T value);
extern boolean_T rtIsNaN(real_T value);
extern boolean_T rtIsNaNF(real32_T value);

#ifdef __cplusplus
}
#endif
#endif
/*
 * File trailer for rt_nonfinite.h
 *
 * [EOF]
 */

/*
 * File: xgetrf.h
 *
 * MATLAB Coder version            : 24.1

 */

#ifndef XGETRF_H
#define XGETRF_H

/* Include Files */

#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
int xgetrf(double A[25], int ipiv[5]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for xgetrf.h
 *
 * [EOF]
 */


/* Function Definitions */
/*
 * Arguments    : const double A[25]
 *                const double B[20]
 *                double Y[20]
 * Return Type  : void
 */
void b_mldivide(const double A[25], const double B[20], double Y[20])
{
  double b_A[25];
  double X[20];
  double temp;
  int ipiv[5];
  int X_tmp;
  int b_i;
  int i;
  int info;
  int j;
  int jBcol;
  int k;
  int kAcol;
  for (i = 0; i < 4; i++) {
    for (info = 0; info < 5; info++) {
      X[info + 5 * i] = B[i + (info << 2)];
    }
  }
  for (i = 0; i < 5; i++) {
    for (info = 0; info < 5; info++) {
      b_A[info + 5 * i] = A[i + 5 * info];
    }
  }
  xgetrf(b_A, ipiv);
  for (b_i = 0; b_i < 4; b_i++) {
    i = ipiv[b_i];
    if (i != b_i + 1) {
      temp = X[b_i];
      X[b_i] = X[i - 1];
      X[i - 1] = temp;
      temp = X[b_i + 5];
      X[b_i + 5] = X[i + 4];
      X[i + 4] = temp;
      temp = X[b_i + 10];
      X[b_i + 10] = X[i + 9];
      X[i + 9] = temp;
      temp = X[b_i + 15];
      X[b_i + 15] = X[i + 14];
      X[i + 14] = temp;
    }
  }
  for (j = 0; j < 4; j++) {
    jBcol = 5 * j;
    for (k = 0; k < 5; k++) {
      kAcol = 5 * k;
      i = k + jBcol;
      if (X[i] != 0.0) {
        info = k + 2;
        for (b_i = info; b_i < 6; b_i++) {
          X_tmp = (b_i + jBcol) - 1;
          X[X_tmp] -= X[i] * b_A[(b_i + kAcol) - 1];
        }
      }
    }
  }
  for (j = 0; j < 4; j++) {
    jBcol = 5 * j;
    for (k = 4; k >= 0; k--) {
      kAcol = 5 * k;
      i = k + jBcol;
      temp = X[i];
      if (temp != 0.0) {
        X[i] = temp / b_A[k + kAcol];
        for (b_i = 0; b_i < k; b_i++) {
          X_tmp = b_i + jBcol;
          X[X_tmp] -= X[i] * b_A[b_i + kAcol];
        }
      }
    }
  }
  for (i = 0; i < 5; i++) {
    info = i << 2;
    Y[info] = X[i];
    Y[info + 1] = X[i + 5];
    Y[info + 2] = X[i + 10];
    Y[info + 3] = X[i + 15];
  }
}

/*
 * Arguments    : const double A[25]
 *                double B[5]
 * Return Type  : void
 */
void c_mldivide(const double A[25], double B[5])
{
  double b_A[25];
  double temp;
  int ipiv[5];
  int b_i;
  int i;
  int k;
  int kAcol;
  for (i = 0; i < 5; i++) {
    for (kAcol = 0; kAcol < 5; kAcol++) {
      b_A[kAcol + 5 * i] = A[i + 5 * kAcol];
    }
  }
  xgetrf(b_A, ipiv);
  if (ipiv[0] != 1) {
    temp = B[0];
    B[0] = B[ipiv[0] - 1];
    B[ipiv[0] - 1] = temp;
  }
  if (ipiv[1] != 2) {
    temp = B[1];
    B[1] = B[ipiv[1] - 1];
    B[ipiv[1] - 1] = temp;
  }
  if (ipiv[2] != 3) {
    temp = B[2];
    B[2] = B[ipiv[2] - 1];
    B[ipiv[2] - 1] = temp;
  }
  if (ipiv[3] != 4) {
    temp = B[3];
    B[3] = B[ipiv[3] - 1];
    B[ipiv[3] - 1] = temp;
  }
  for (k = 0; k < 5; k++) {
    kAcol = 5 * k;
    if (B[k] != 0.0) {
      i = k + 2;
      for (b_i = i; b_i < 6; b_i++) {
        B[b_i - 1] -= B[k] * b_A[(b_i + kAcol) - 1];
      }
    }
  }
  for (k = 4; k >= 0; k--) {
    kAcol = 5 * k;
    temp = B[k];
    if (temp != 0.0) {
      temp /= b_A[k + kAcol];
      B[k] = temp;
      for (b_i = 0; b_i < k; b_i++) {
        B[b_i] -= B[k] * b_A[b_i + kAcol];
      }
    }
  }
}

/*
 * Arguments    : const double A[25]
 *                const double B[15]
 *                double Y[15]
 * Return Type  : void
 */
void mldivide(const double A[25], const double B[15], double Y[15])
{
  double b_A[25];
  double X[15];
  double temp;
  int ipiv[5];
  int X_tmp;
  int b_i;
  int i;
  int info;
  int j;
  int jBcol;
  int k;
  int kAcol;
  for (info = 0; info < 3; info++) {
    for (i = 0; i < 5; i++) {
      X[i + 5 * info] = B[info + 3 * i];
    }
  }
  for (info = 0; info < 5; info++) {
    for (i = 0; i < 5; i++) {
      b_A[i + 5 * info] = A[info + 5 * i];
    }
  }
  xgetrf(b_A, ipiv);
  for (b_i = 0; b_i < 4; b_i++) {
    info = ipiv[b_i];
    if (info != b_i + 1) {
      temp = X[b_i];
      X[b_i] = X[info - 1];
      X[info - 1] = temp;
      temp = X[b_i + 5];
      X[b_i + 5] = X[info + 4];
      X[info + 4] = temp;
      temp = X[b_i + 10];
      X[b_i + 10] = X[info + 9];
      X[info + 9] = temp;
    }
  }
  for (j = 0; j < 3; j++) {
    jBcol = 5 * j;
    for (k = 0; k < 5; k++) {
      kAcol = 5 * k;
      info = k + jBcol;
      if (X[info] != 0.0) {
        i = k + 2;
        for (b_i = i; b_i < 6; b_i++) {
          X_tmp = (b_i + jBcol) - 1;
          X[X_tmp] -= X[info] * b_A[(b_i + kAcol) - 1];
        }
      }
    }
  }
  for (j = 0; j < 3; j++) {
    jBcol = 5 * j;
    for (k = 4; k >= 0; k--) {
      kAcol = 5 * k;
      info = k + jBcol;
      temp = X[info];
      if (temp != 0.0) {
        X[info] = temp / b_A[k + kAcol];
        for (b_i = 0; b_i < k; b_i++) {
          X_tmp = b_i + jBcol;
          X[X_tmp] -= X[info] * b_A[b_i + kAcol];
        }
      }
    }
  }
  for (info = 0; info < 5; info++) {
    Y[3 * info] = X[info];
    Y[3 * info + 1] = X[info + 5];
    Y[3 * info + 2] = X[info + 10];
  }
}

/*
 * File trailer for mldivide.c
 *
 * [EOF]
 */

/*
 * File: rtGetInf.c
 *
 * MATLAB Coder version            : 24.1

 */

/*
 * Abstract:
 *       MATLAB for code generation function to initialize non-finite, Inf and
 * MinusInf
 */
/* Include Files */
/*
 * File: rtGetInf.h
 *
 * MATLAB Coder version            : 24.1

 */

#ifndef RTGETINF_H
#define RTGETINF_H

/* Include Files */


#ifdef __cplusplus
extern "C" {
#endif

extern real_T rtGetInf(void);
extern real32_T rtGetInfF(void);
extern real_T rtGetMinusInf(void);
extern real32_T rtGetMinusInfF(void);

#ifdef __cplusplus
}
#endif
#endif
/*
 * File trailer for rtGetInf.h
 *
 * [EOF]
 */



/*
 * Function: rtGetInf
 * ================================================================== Abstract:
 *  Initialize rtInf needed by the generated code.
 */
real_T rtGetInf(void)
{
  return rtInf;
}

/*
 * Function: rtGetInfF
 * ================================================================= Abstract:
 *  Initialize rtInfF needed by the generated code.
 */
real32_T rtGetInfF(void)
{
  return rtInfF;
}

/*
 * Function: rtGetMinusInf
 * ============================================================= Abstract:
 *  Initialize rtMinusInf needed by the generated code.
 */
real_T rtGetMinusInf(void)
{
  return rtMinusInf;
}

/*
 * Function: rtGetMinusInfF
 * ============================================================ Abstract:
 *  Initialize rtMinusInfF needed by the generated code.
 */
real32_T rtGetMinusInfF(void)
{
  return rtMinusInfF;
}

/*
 * File trailer for rtGetInf.c
 *
 * [EOF]
 */

/*
 * File: rtGetNaN.c
 *
 * MATLAB Coder version            : 24.1

 */

/*
 * Abstract:
 *       MATLAB for code generation function to initialize non-finite, NaN
 */
/* Include Files */
/*
 * File: rtGetNaN.h
 *
 * MATLAB Coder version            : 24.1

 */

#ifndef RTGETNAN_H
#define RTGETNAN_H

/* Include Files */


#ifdef __cplusplus
extern "C" {
#endif

extern real_T rtGetNaN(void);
extern real32_T rtGetNaNF(void);

#ifdef __cplusplus
}
#endif
#endif
/*
 * File trailer for rtGetNaN.h
 *
 * [EOF]
 */



/*
 * Function: rtGetNaN
 * ======================================================================
 *  Abstract:
 * Initialize rtNaN needed by the generated code.
 *  NaN is initialized as non-signaling. Assumes IEEE.
 */
real_T rtGetNaN(void)
{
  return rtNaN;
}

/*
 * Function: rtGetNaNF
 * =====================================================================
 *  Abstract:
 *  Initialize rtNaNF needed by the generated code.
 *  NaN is initialized as non-signaling. Assumes IEEE
 */
real32_T rtGetNaNF(void)
{
  return rtNaNF;
}

/*
 * File trailer for rtGetNaN.c
 *
 * [EOF]
 */

/*
 * File: rt_nonfinite.c
 *
 * MATLAB Coder version            : 24.1

 */

/*
 * Abstract:
 *      MATLAB for code generation function to initialize non-finites,
 *      (Inf, NaN and -Inf).
 */
/* Include Files */

#include <math.h>

#if defined(__ICL) && __ICL == 1700
#pragma warning(disable : 264)
#endif

real_T rtNaN = (real_T)NAN;
real_T rtInf = (real_T)INFINITY;
real_T rtMinusInf = -(real_T)INFINITY;
real32_T rtNaNF = (real32_T)NAN;
real32_T rtInfF = (real32_T)INFINITY;
real32_T rtMinusInfF = -(real32_T)INFINITY;

#if defined(__ICL) && __ICL == 1700
#pragma warning(default : 264)
#endif

/*
 * Function: rtIsInf ==================================================
 *  Abstract:
 *  Test if value is infinite
 */
boolean_T rtIsInf(real_T value)
{
  return (isinf(value) != 0U);
}

/*
 * Function: rtIsInfF =================================================
 *  Abstract:
 *  Test if single-precision value is infinite
 */
boolean_T rtIsInfF(real32_T value)
{
  return (isinf((real_T)value) != 0U);
}

/*
 * Function: rtIsNaN ==================================================
 *  Abstract:
 *  Test if value is not a number
 */
boolean_T rtIsNaN(real_T value)
{
  return (isnan(value) != 0U);
}

/*
 * Function: rtIsNaNF =================================================
 *  Abstract:
 *  Test if single-precision value is not a number
 */
boolean_T rtIsNaNF(real32_T value)
{
  return (isnan((real_T)value) != 0U);
}

/*
 * File trailer for rt_nonfinite.c
 *
 * [EOF]
 */

/*
 * File: tinympc14_bal_block.c
 *
 * MATLAB Coder version            : 24.1

 */

/* Include Files */
/*
 * File: tinympc14_bal_block.h
 *
 * MATLAB Coder version            : 24.1

 */

#ifndef TINYMPC14_BAL_BLOCK_H
#define TINYMPC14_BAL_BLOCK_H

/* Include Files */

#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void tinympc14_bal_block(double lL, double lR, double A[100], double B[40],
                         double f[10]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for tinympc14_bal_block.h
 *
 * [EOF]
 */



/*
 * File: tinympc14_lpv_kernel_data.h
 *
 * MATLAB Coder version            : 24.1

 */

#ifndef TINYMPC14_LPV_KERNEL_DATA_H
#define TINYMPC14_LPV_KERNEL_DATA_H

/* Include Files */

#include <stddef.h>
#include <stdlib.h>

/* Variable Declarations */
extern const double dv[63];
extern const double dv1[80];
extern const double dv2[60];
extern const double dv3[168];

#endif
/*
 * File trailer for tinympc14_lpv_kernel_data.h
 *
 * [EOF]
 */

#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * 方程 (19)-(23) 单点线性化：两腿各按自身腿长取表，10 态 A/B + 仿射 f
 *  grav_mode: 'B'  = B 族线性口径（对拍用）
 *             'A0' = A 族在 theta=0 线性化（常数 +C2*sin(theta0)）
 *             'V*' = A 族在真实平衡点 theta* 线性化（常数 tau_g(0)+余项）
 *
 * Arguments    : double lL
 *                double lR
 *                double A[100]
 *                double B[40]
 *                double f[10]
 * Return Type  : void
 */
void tinympc14_bal_block(double lL, double lR, double A[100], double B[40],
                         double f[10])
{
  static const signed char iv[5] = {0, 0, 1, 0, 0};
  static const signed char iv1[5] = {0, 0, 0, 1, 0};
  static const signed char iv2[5] = {0, 0, 0, 0, 1};
  double M[25];
  double T[25];
  double Nu[20];
  double b_dv1[20];
  double Nx[15];
  double b_dv[15];
  double y[5];
  double I_z;
  double a;
  double b_I_z_tmp;
  double b_a;
  double b_l_tmp;
  double cL_idx_0_tmp;
  double cL_idx_1_tmp;
  double c_I_z_tmp;
  double d_I_z_tmp;
  double e_I_z_tmp;
  double l_tmp;
  double rL_idx_0;
  double rL_idx_1;
  double rL_idx_2_tmp;
  double rR_idx_0;
  double rR_idx_1;
  double rR_idx_2_tmp;
  int I_z_tmp;
  int a_tmp;
  int b_a_tmp;
  int b_k;
  int k;
  signed char b_I[25];
  /*  分段线性插值 + 端点钳位（与 Leg_Comp.c 同口径） */
  l_tmp = fmin(fmax(lL, 0.15), 0.32110177111662558);
  k = 1;
  while ((k < 20) && (l_tmp >= dv[3 * k])) {
    k++;
  }
  /*  分段线性插值 + 端点钳位（与 Leg_Comp.c 同口径） */
  b_l_tmp = fmin(fmax(lR, 0.15), 0.32110177111662558);
  b_k = 1;
  while ((b_k < 20) && (b_l_tmp >= dv[3 * b_k])) {
    b_k++;
  }
  I_z_tmp = 3 * (k - 1);
  b_I_z_tmp = dv[I_z_tmp];
  c_I_z_tmp = dv[I_z_tmp + 1];
  I_z_tmp = 3 * (b_k - 1);
  d_I_z_tmp = dv[I_z_tmp];
  e_I_z_tmp = dv[I_z_tmp + 2];
  I_z = ((c_I_z_tmp + (l_tmp - b_I_z_tmp) / (dv[3 * k] - b_I_z_tmp) *
                          (dv[3 * k + 1] - c_I_z_tmp)) +
         0.022341639484140273) +
        (e_I_z_tmp + (b_l_tmp - d_I_z_tmp) / (dv[3 * b_k] - d_I_z_tmp) *
                         (dv[3 * b_k + 2] - e_I_z_tmp));
  /*  分段线性插值 + 端点钳位（与 Leg_Comp.c 同口径） */
  k = 1;
  while ((k < 19) && (l_tmp >= dv1[k << 2])) {
    k++;
  }
  I_z_tmp = (k - 1) << 2;
  b_I_z_tmp = dv1[I_z_tmp];
  a_tmp = k << 2;
  a = (l_tmp - b_I_z_tmp) / (dv1[a_tmp] - b_I_z_tmp);
  b_I_z_tmp = dv1[I_z_tmp + 1];
  rL_idx_0 = b_I_z_tmp + a * (dv1[a_tmp + 1] - b_I_z_tmp);
  b_I_z_tmp = dv1[I_z_tmp + 2];
  rL_idx_1 = b_I_z_tmp + a * (dv1[a_tmp + 2] - b_I_z_tmp);
  rL_idx_2_tmp = dv1[I_z_tmp + 3];
  /*  分段线性插值 + 端点钳位（与 Leg_Comp.c 同口径） */
  k = 1;
  while ((k < 19) && (b_l_tmp >= dv1[k << 2])) {
    k++;
  }
  I_z_tmp = (k - 1) << 2;
  b_I_z_tmp = dv1[I_z_tmp];
  b_a_tmp = k << 2;
  b_a = (b_l_tmp - b_I_z_tmp) / (dv1[b_a_tmp] - b_I_z_tmp);
  b_I_z_tmp = dv1[I_z_tmp + 1];
  rR_idx_0 = b_I_z_tmp + b_a * (dv1[b_a_tmp + 1] - b_I_z_tmp);
  b_I_z_tmp = dv1[I_z_tmp + 2];
  rR_idx_1 = b_I_z_tmp + b_a * (dv1[b_a_tmp + 2] - b_I_z_tmp);
  rR_idx_2_tmp = dv1[I_z_tmp + 3];
  /*  分段线性插值 + 端点钳位（与 Leg_Comp.c 同口径） */
  k = 1;
  while ((k < 19) && (l_tmp >= dv2[3 * k])) {
    k++;
  }
  I_z_tmp = 3 * (k - 1);
  b_I_z_tmp = dv2[I_z_tmp];
  l_tmp = (l_tmp - b_I_z_tmp) / (dv2[3 * k] - b_I_z_tmp);
  cL_idx_0_tmp = dv2[I_z_tmp + 1];
  cL_idx_1_tmp = dv2[I_z_tmp + 2];
  /*  分段线性插值 + 端点钳位（与 Leg_Comp.c 同口径） */
  b_k = 1;
  while ((b_k < 19) && (b_l_tmp >= dv2[3 * b_k])) {
    b_k++;
  }
  I_z_tmp = 3 * (b_k - 1);
  b_I_z_tmp = dv2[I_z_tmp];
  c_I_z_tmp = (b_l_tmp - b_I_z_tmp) / (dv2[3 * b_k] - b_I_z_tmp);
  d_I_z_tmp = dv2[I_z_tmp + 1];
  e_I_z_tmp = dv2[I_z_tmp + 2];
  /*  左腿（打印口径沿用左腿） */
  /*  A 族整机项系数 = (m_b/2 + m_leg)*g*l */
  /*  A 族腿自重项系数 = m_leg*g*l_d */
  /*  B 族腿行重力刚度 */
  /*  tau_g(0) = M_LEG*g*x_off(H) */
  memset(&M[0], 0, 25U * sizeof(double));
  M[0] = (0.00072123659251607884 * lL / 0.060000002144914348 +
          0.035891517958839728 * lL) +
         0.079276605183840845 * rL_idx_1;
  M[2] = 1.3212767058302579 * rL_idx_0 * rL_idx_1 -
         (rL_idx_2_tmp + a * (dv1[a_tmp + 3] - rL_idx_2_tmp));
  M[6] = (0.00072123659251607884 * lR / 0.060000002144914348 +
          0.035891517958839728 * lR) +
         0.079276605183840845 * rR_idx_1;
  M[8] = 1.3212767058302579 * rR_idx_0 * rR_idx_1 -
         (rR_idx_2_tmp + b_a * (dv1[b_a_tmp + 3] - rR_idx_2_tmp));
  M[12] = -(0.079276605183840845 * rL_idx_0 + 0.64602748742033478 * lL / 2.0);
  M[13] = -(0.079276605183840845 * rR_idx_0 + 0.64602748742033478 * lR / 2.0);
  M[10] = -0.027012149543549531;
  M[15] = -0.0043296421992597822;
  M[11] = -0.027012149543549531;
  M[16] = -0.0043296421992597822;
  M[17] = 1.3212767058302579 * rL_idx_0 * -0.034041082972946314;
  M[18] = 1.3212767058302579 * rR_idx_0 * -0.034041082972946314;
  M[19] = -0.033195967438518492;
  M[20] =
      I_z * 0.060000002144914348 / 0.41440003470893561 + 0.0024906704857288985;
  M[21] = -M[20];
  M[22] = I_z * lL / 0.41440003470893561;
  M[23] = -I_z * lR / 0.41440003470893561;
  memset(&Nx[0], 0, 15U * sizeof(double));
  /*  N_s = -(方程里除 ddq 外的其余项)，故整体取负 */
  Nx[0] =
      -((1.3212767058302579 * rL_idx_0 + 10.767124405429586 * lL / 2.0) * 9.81);
  Nx[4] =
      -((1.3212767058302579 * rR_idx_0 + 10.767124405429586 * lR / 2.0) * 9.81);
  /*  The output adds each leg's gravity torque to TB. Its reaction acts on */
  /*  the body too; include it with the body's signed horizontal first moment.
   */
  /*  kB already contains the vertical leg first moment. Retain the missing */
  /*  horizontal first-moment bias once, without adding its stiffness twice. */
  Nx[11] = 3.4999417623887892;
  /*  同上：Nu = -∂(其余项)/∂T */
  Nu[0] = lL / 0.060000002144914348 + 1.0;
  Nu[1] = 0.0;
  Nu[2] = -1.0;
  Nu[3] = 0.0;
  Nu[4] = 0.0;
  Nu[5] = lR / 0.060000002144914348 + 1.0;
  Nu[6] = 0.0;
  Nu[7] = -1.0;
  Nu[8] = -1.0;
  Nu[12] = -0.56735136260043728;
  Nu[16] = 3.4533334991227207;
  Nu[9] = -1.0;
  Nu[13] = -0.56735136260043728;
  Nu[17] = -3.4533334991227207;
  Nu[10] = 0.0;
  Nu[14] = 1.0;
  Nu[18] = 0.0;
  Nu[11] = 0.0;
  Nu[15] = 1.0;
  Nu[19] = 0.0;
  /*  s/ds use the mean hip position/velocity consumed from body.vel. */
  /*  At fixed measured lengths: s_hip=s_wheel+(lL*psiL+lR*psiR)/2. */
  T[0] = 0.030000001072457174;
  T[1] = 0.030000001072457174;
  T[2] = lL / 2.0;
  T[3] = lR / 2.0;
  T[4] = 0.0;
  T[5] = -0.14478763783660606;
  T[6] = 0.14478763783660606;
  T[7] = -lL / 0.41440003470893561;
  T[8] = lR / 0.41440003470893561;
  T[9] = 0.0;
  for (I_z_tmp = 0; I_z_tmp < 5; I_z_tmp++) {
    T[I_z_tmp + 10] = iv[I_z_tmp];
    T[I_z_tmp + 15] = iv1[I_z_tmp];
    T[I_z_tmp + 20] = iv2[I_z_tmp];
  }
  memset(&A[0], 0, 100U * sizeof(double));
  for (I_z_tmp = 0; I_z_tmp < 25; I_z_tmp++) {
    b_I[I_z_tmp] = 0;
  }
  for (I_z_tmp = 0; I_z_tmp < 5; I_z_tmp++) {
    b_I[I_z_tmp + 5 * I_z_tmp] = 1;
  }
  for (I_z_tmp = 0; I_z_tmp < 5; I_z_tmp++) {
    for (a_tmp = 0; a_tmp < 5; a_tmp++) {
      A[((a_tmp << 1) + 10 * (I_z_tmp << 1)) + 1] = b_I[a_tmp + 5 * I_z_tmp];
    }
  }
  mldivide(M, Nx, b_dv);
  for (I_z_tmp = 0; I_z_tmp < 3; I_z_tmp++) {
    for (a_tmp = 0; a_tmp < 5; a_tmp++) {
      b_I_z_tmp = 0.0;
      for (b_a_tmp = 0; b_a_tmp < 5; b_a_tmp++) {
        b_I_z_tmp += b_dv[I_z_tmp + 3 * b_a_tmp] * T[b_a_tmp + 5 * a_tmp];
      }
      A[((I_z_tmp << 1) + 10 * ((a_tmp << 1) + 1)) + 4] = b_I_z_tmp;
    }
  }
  memset(&B[0], 0, 40U * sizeof(double));
  b_mldivide(M, Nu, b_dv1);
  for (I_z_tmp = 0; I_z_tmp < 4; I_z_tmp++) {
    for (a_tmp = 0; a_tmp < 5; a_tmp++) {
      b_I_z_tmp = 0.0;
      for (b_a_tmp = 0; b_a_tmp < 5; b_a_tmp++) {
        b_I_z_tmp += b_dv1[I_z_tmp + (b_a_tmp << 2)] * T[b_a_tmp + 5 * a_tmp];
      }
      B[I_z_tmp + (((a_tmp << 1) + 1) << 2)] = b_I_z_tmp;
    }
  }
  memset(&f[0], 0, 10U * sizeof(double));
  y[0] = 0.0;
  y[1] = 0.0;
  y[2] = 0.0;
  y[3] = (12.96172448419483 *
              (cL_idx_1_tmp + l_tmp * (dv2[3 * k + 2] - cL_idx_1_tmp)) *
              sin(cL_idx_0_tmp + l_tmp * (dv2[3 * k + 1] - cL_idx_0_tmp)) +
          12.96172448419483 *
              (e_I_z_tmp + c_I_z_tmp * (dv2[3 * b_k + 2] - e_I_z_tmp)) *
              sin(d_I_z_tmp + c_I_z_tmp * (dv2[3 * b_k + 1] - d_I_z_tmp))) -
         1.826603543884421;
  y[4] = 0.0;
  c_mldivide(M, y);
  for (I_z_tmp = 0; I_z_tmp < 5; I_z_tmp++) {
    b_I_z_tmp = 0.0;
    for (a_tmp = 0; a_tmp < 5; a_tmp++) {
      b_I_z_tmp += y[a_tmp] * T[a_tmp + 5 * I_z_tmp];
    }
    f[(I_z_tmp << 1) + 1] = b_I_z_tmp;
  }
}

/*
 * File trailer for tinympc14_bal_block.c
 *
 * [EOF]
 */

/*
 * File: tinympc14_fused_dyn.c
 *
 * MATLAB Coder version            : 24.1

 */

/* Include Files */
/*
 * File: tinympc14_fused_dyn.h
 *
 * MATLAB Coder version            : 24.1

 */

#ifndef TINYMPC14_FUSED_DYN_H
#define TINYMPC14_FUSED_DYN_H

/* Include Files */

#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void tinympc14_fused_dyn(const double op_A_bal[100], const double op_B_bal[40],
                         const double op_f_bal[10], double op_K_eff,
                         const double x[14], const double u[6], double xd[14]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for tinympc14_fused_dyn.h
 *
 * [EOF]
 */



#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * 14 态降阶动力学。前十态沿用冻结腿长的平衡块，高度和横滚使用同一闭链虚功。
 *  op 由 tinympc14_fused_op 提供（lL/lR 与本点平衡块）；psi 取 x(5)/x(7)
 *
 * Arguments    : const double op_A_bal[100]
 *                const double op_B_bal[40]
 *                const double op_f_bal[10]
 *                double op_K_eff
 *                const double x[14]
 *                const double u[6]
 *                double xd[14]
 * Return Type  : void
 */
void tinympc14_fused_dyn(const double op_A_bal[100], const double op_B_bal[40],
                         const double op_f_bal[10], double op_K_eff,
                         const double x[14], const double u[6], double xd[14])
{
  double row[7];
  double M_hr[4];
  double wheel_force[2];
  double acceleration_idx_1;
  double b_a21_tmp;
  double load;
  double sec_phi;
  int a21_tmp;
  int c_a21_tmp;
  int k;
  int r2;
  memset(&xd[0], 0, 14U * sizeof(double));
  for (a21_tmp = 0; a21_tmp < 7; a21_tmp++) {
    k = a21_tmp << 1;
    xd[k] = x[k + 1];
  }
  for (a21_tmp = 0; a21_tmp < 5; a21_tmp++) {
    b_a21_tmp = 0.0;
    for (k = 0; k < 10; k++) {
      b_a21_tmp += x[k] * op_A_bal[k + 10 * ((a21_tmp << 1) + 1)];
    }
    k = (a21_tmp << 1) + 1;
    r2 = k << 2;
    xd[k] = (b_a21_tmp + (((u[0] * op_B_bal[r2] + u[1] * op_B_bal[r2 + 1]) +
                           u[2] * op_B_bal[r2 + 2]) +
                          u[3] * op_B_bal[r2 + 3])) +
            op_f_bal[k];
  }
  /*  双轮无侧滑的 CAD 升降/横滚切向惯量与重力，坐标顺序 [H,roll]。 */
  /*  分段线性插值 + 端点钳位（与 Leg_Comp.c 同口径） */
  sec_phi = fmin(fmax(x[12] + 0.27, 0.15), 0.32110177111662558);
  k = 1;
  while ((k < 20) && (sec_phi >= dv3[k << 3])) {
    k++;
  }
  r2 = (k - 1) << 3;
  acceleration_idx_1 = dv3[r2];
  k <<= 3;
  sec_phi = (sec_phi - acceleration_idx_1) / (dv3[k] - acceleration_idx_1);
  for (a21_tmp = 0; a21_tmp < 7; a21_tmp++) {
    acceleration_idx_1 = dv3[(a21_tmp + r2) + 1];
    row[a21_tmp] = acceleration_idx_1 +
                   sec_phi * (dv3[(a21_tmp + k) + 1] - acceleration_idx_1);
  }
  double b_u;
  double b_wheel_force;
  double height_arm_idx_0;
  double height_arm_idx_1;
  double x_tmp;
  M_hr[0] = row[0];
  M_hr[1] = row[1];
  M_hr[2] = row[1];
  M_hr[3] = row[2];
  load = row[3] + op_K_eff * x[12];
  /*  双轮平地闭链 L_i=(H±b sin(phi))/(cos(phi)cos(psi_i))。 */
  /*  固定两腿矢状角的虚位移 dL_i=height_arm_i*dH+roll_arm_i*dphi。 */
  /*  轴力与 +L 共轭，乘上述两臂分别得到虚升降力和虚横滚力矩。 */
  sec_phi = 1.0 / cos(x[10]);
  height_arm_idx_0 = sec_phi / cos(x[4]);
  height_arm_idx_1 = sec_phi / cos(x[6]);
  x_tmp = tan(x[10]);
  acceleration_idx_1 = (x[12] + 0.27) * x_tmp;
  /*  名义竖直承重 load、两腿等分时的几何倾覆矩。其合成效果已由 */
  /*  tinympc14_roll_force 给定，内部仅保留偏离该预载引起的力矩变化。 */
  /*  固定髋部前向位置和腿角，轮心水平位移为 -sin(psi_i)*dL_i。 */
  /*  模型正轮矩与电机轮矩反号，故等价轴力为 +Tw_i*sin(psi_i)/R_w。 */
  wheel_force[0] = u[0] * sin(x[4]) / 0.060000002144914348;
  wheel_force[1] = u[1] * sin(x[6]) / 0.060000002144914348;
  b_a21_tmp =
      height_arm_idx_0 * (acceleration_idx_1 + 0.2072000173544678 * sec_phi);
  b_u = u[4] * b_a21_tmp;
  b_wheel_force = wheel_force[0] * b_a21_tmp;
  b_a21_tmp =
      height_arm_idx_1 * (acceleration_idx_1 + -0.2072000173544678 * sec_phi);
  sec_phi =
      wheel_force[0] * height_arm_idx_0 + wheel_force[1] * height_arm_idx_1;
  b_u += u[5] * b_a21_tmp;
  b_wheel_force += wheel_force[1] * b_a21_tmp;
  wheel_force[0] =
      (((u[4] * height_arm_idx_0 + u[5] * height_arm_idx_1) - load) -
       8.0 * x[13]) +
      sec_phi;
  wheel_force[1] =
      ((b_u - load * (x[12] + 0.27) * x_tmp) - row[4]) + b_wheel_force;
  if (fabs(row[1]) > fabs(row[0])) {
    k = 1;
    r2 = 0;
  } else {
    k = 0;
    r2 = 1;
  }
  a21_tmp = r2 << 1;
  c_a21_tmp = k << 1;
  b_a21_tmp = M_hr[c_a21_tmp];
  sec_phi = M_hr[a21_tmp] / b_a21_tmp;
  load = M_hr[c_a21_tmp + 1];
  acceleration_idx_1 = (wheel_force[r2] - wheel_force[k] * sec_phi) /
                       (M_hr[a21_tmp + 1] - sec_phi * load);
  xd[13] = (wheel_force[k] - acceleration_idx_1 * load) / b_a21_tmp;
  xd[11] = 0.0 * x[10] + acceleration_idx_1;
}

/*
 * File trailer for tinympc14_fused_dyn.c
 *
 * [EOF]
 */

/*
 * File: tinympc14_fused_op.c
 *
 * MATLAB Coder version            : 24.1

 */

/* Include Files */
/*
 * File: tinympc14_fused_op.h
 *
 * MATLAB Coder version            : 24.1

 */

#ifndef TINYMPC14_FUSED_OP_H
#define TINYMPC14_FUSED_OP_H

/* Include Files */

/*
 * File: tinympc14_lpv_kernel_internal_types.h
 *
 * MATLAB Coder version            : 24.1

 */

#ifndef TINYMPC14_LPV_KERNEL_INTERNAL_TYPES_H
#define TINYMPC14_LPV_KERNEL_INTERNAL_TYPES_H

/* Include Files */

/*
 * File: tinympc14_lpv_kernel_types.h
 *
 * MATLAB Coder version            : 24.1

 */

#ifndef TINYMPC14_LPV_KERNEL_TYPES_H
#define TINYMPC14_LPV_KERNEL_TYPES_H

/* Include Files */


/* Type Definitions */
#ifndef typedef_struct4_T
#define typedef_struct4_T
typedef struct {
  double length_grid[19];
  double angle_axis[33];
  double angle_scale;
  float basis[4536];
  float offset[189];
} struct4_T;
#endif /* typedef_struct4_T */

#endif
/*
 * File trailer for tinympc14_lpv_kernel_types.h
 *
 * [EOF]
 */


/* Type Definitions */
#ifndef typedef_struct_T
#define typedef_struct_T
typedef struct {
  double l_wl;
  double l_bl;
  double I_ll;
  double theta0;
  double l_d;
  double l_wr;
  double l_br;
  double I_lr;
  double theta0_r;
  double l_d_r;
  double kA0;
  double kA0_r;
  double kA;
  double kA_r;
  double kB;
  double kB_r;
  double tg0;
  double tg0_r;
  double xoff;
  double xoff_r;
  double Gconst;
  double Gconst_r;
  double theta_star;
  double theta_star_l;
  double theta_star_r;
  double body_pitch_slope;
  double body_pitch_constant;
  double leg_body_slope[2];
} struct_T;
#endif /* typedef_struct_T */

#ifndef typedef_b_struct_T
#define typedef_b_struct_T
typedef struct {
  double A_bal[100];
  double B_bal[40];
  double f_bal[10];
  struct_T aux;
  double lL;
  double lR;
  double H;
  double M_v;
  double M_load;
  double K_eff;
  double psi;
  double M_hr[4];
  double G_hr[2];
  double I_roll;
} b_struct_T;
#endif /* typedef_b_struct_T */

#endif
/*
 * File trailer for tinympc14_lpv_kernel_internal_types.h
 *
 * [EOF]
 */

#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void tinympc14_fused_op(double sched_H, double sched_lL, double sched_lR,
                        double sched_psi, b_struct_T *op);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for tinympc14_fused_op.h
 *
 * [EOF]
 */





#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * 本调度点的平衡块与腿长块参数包：一次算好供非线性核与数值雅可比复用
 *
 * Arguments    : double sched_H
 *                double sched_lL
 *                double sched_lR
 *                double sched_psi
 *                b_struct_T *op
 * Return Type  : void
 */
void tinympc14_fused_op(double sched_H, double sched_lL, double sched_lR,
                        double sched_psi, b_struct_T *op)
{
  static const signed char iv[5] = {0, 0, 1, 0, 0};
  static const signed char iv1[5] = {0, 0, 0, 1, 0};
  static const signed char iv2[5] = {0, 0, 0, 0, 1};
  double M[25];
  double T[25];
  double Nu[20];
  double b_dv1[20];
  double Nx[15];
  double b_dv[15];
  double row[7];
  double y[5];
  double C2g;
  double C2g_r;
  double I_z;
  double b_I_z_tmp;
  double body_constant;
  double cL_idx_0;
  double cL_idx_1;
  double cR_idx_0;
  double cR_idx_1;
  double kB;
  double kB_r;
  double rL_idx_0;
  double rL_idx_1;
  double rL_idx_2;
  double rR_idx_0;
  double rR_idx_1;
  double rR_idx_2;
  double tg0;
  double tg0_r;
  double tg0_r_tmp;
  double tg0_tmp;
  int I_z_tmp;
  int b_k;
  int k;
  signed char b_I[25];
  /*  方程 (19)-(23) 单点线性化：两腿各按自身腿长取表，10 态 A/B + 仿射 f */
  /*  grav_mode: 'B'  = B 族线性口径（对拍用） */
  /*             'A0' = A 族在 theta=0 线性化（常数 +C2*sin(theta0)） */
  /*             'V*' = A 族在真实平衡点 theta* 线性化（常数 tau_g(0)+余项） */
  /*  分段线性插值 + 端点钳位（与 Leg_Comp.c 同口径） */
  C2g_r = fmin(fmax(sched_lL, 0.15), 0.32110177111662558);
  k = 1;
  while ((k < 20) && (C2g_r >= dv[3 * k])) {
    k++;
  }
  /*  分段线性插值 + 端点钳位（与 Leg_Comp.c 同口径） */
  kB = fmin(fmax(sched_lR, 0.15), 0.32110177111662558);
  b_k = 1;
  while ((b_k < 20) && (kB >= dv[3 * b_k])) {
    b_k++;
  }
  I_z_tmp = 3 * (k - 1);
  body_constant = dv[I_z_tmp];
  b_I_z_tmp = dv[I_z_tmp + 1];
  I_z_tmp = 3 * (b_k - 1);
  cR_idx_1 = dv[I_z_tmp];
  C2g = dv[I_z_tmp + 2];
  I_z = ((b_I_z_tmp + (C2g_r - body_constant) / (dv[3 * k] - body_constant) *
                          (dv[3 * k + 1] - b_I_z_tmp)) +
         0.022341639484140273) +
        (C2g +
         (kB - cR_idx_1) / (dv[3 * b_k] - cR_idx_1) * (dv[3 * b_k + 2] - C2g));
  /*  分段线性插值 + 端点钳位（与 Leg_Comp.c 同口径） */
  k = 1;
  while ((k < 19) && (C2g_r >= dv1[k << 2])) {
    k++;
  }
  I_z_tmp = (k - 1) << 2;
  b_I_z_tmp = dv1[I_z_tmp];
  b_k = k << 2;
  b_I_z_tmp = (C2g_r - b_I_z_tmp) / (dv1[b_k] - b_I_z_tmp);
  body_constant = dv1[I_z_tmp + 1];
  rL_idx_0 = body_constant + b_I_z_tmp * (dv1[b_k + 1] - body_constant);
  body_constant = dv1[I_z_tmp + 2];
  rL_idx_1 = body_constant + b_I_z_tmp * (dv1[b_k + 2] - body_constant);
  body_constant = dv1[I_z_tmp + 3];
  rL_idx_2 = body_constant + b_I_z_tmp * (dv1[b_k + 3] - body_constant);
  /*  分段线性插值 + 端点钳位（与 Leg_Comp.c 同口径） */
  k = 1;
  while ((k < 19) && (kB >= dv1[k << 2])) {
    k++;
  }
  I_z_tmp = (k - 1) << 2;
  b_I_z_tmp = dv1[I_z_tmp];
  b_k = k << 2;
  b_I_z_tmp = (kB - b_I_z_tmp) / (dv1[b_k] - b_I_z_tmp);
  body_constant = dv1[I_z_tmp + 1];
  rR_idx_0 = body_constant + b_I_z_tmp * (dv1[b_k + 1] - body_constant);
  body_constant = dv1[I_z_tmp + 2];
  rR_idx_1 = body_constant + b_I_z_tmp * (dv1[b_k + 2] - body_constant);
  body_constant = dv1[I_z_tmp + 3];
  rR_idx_2 = body_constant + b_I_z_tmp * (dv1[b_k + 3] - body_constant);
  /*  分段线性插值 + 端点钳位（与 Leg_Comp.c 同口径） */
  k = 1;
  while ((k < 19) && (C2g_r >= dv2[3 * k])) {
    k++;
  }
  I_z_tmp = 3 * (k - 1);
  b_I_z_tmp = dv2[I_z_tmp];
  b_I_z_tmp = (C2g_r - b_I_z_tmp) / (dv2[3 * k] - b_I_z_tmp);
  body_constant = dv2[I_z_tmp + 1];
  cL_idx_0 = body_constant + b_I_z_tmp * (dv2[3 * k + 1] - body_constant);
  body_constant = dv2[I_z_tmp + 2];
  cL_idx_1 = body_constant + b_I_z_tmp * (dv2[3 * k + 2] - body_constant);
  /*  分段线性插值 + 端点钳位（与 Leg_Comp.c 同口径） */
  k = 1;
  while ((k < 19) && (kB >= dv2[3 * k])) {
    k++;
  }
  I_z_tmp = 3 * (k - 1);
  b_I_z_tmp = dv2[I_z_tmp];
  b_I_z_tmp = (kB - b_I_z_tmp) / (dv2[3 * k] - b_I_z_tmp);
  body_constant = dv2[I_z_tmp + 1];
  cR_idx_0 = body_constant + b_I_z_tmp * (dv2[3 * k + 1] - body_constant);
  body_constant = dv2[I_z_tmp + 2];
  cR_idx_1 = body_constant + b_I_z_tmp * (dv2[3 * k + 2] - body_constant);
  /*  左腿（打印口径沿用左腿） */
  /*  A 族整机项系数 = (m_b/2 + m_leg)*g*l */
  C2g = 12.96172448419483 * cL_idx_1;
  /*  A 族腿自重项系数 = m_leg*g*l_d */
  C2g_r = 12.96172448419483 * cR_idx_1;
  kB = (1.3212767058302579 * rL_idx_0 + 10.767124405429586 * sched_lL / 2.0) *
       9.81;
  /*  B 族腿行重力刚度 */
  kB_r = (1.3212767058302579 * rR_idx_0 + 10.767124405429586 * sched_lR / 2.0) *
         9.81;
  tg0_tmp = sin(cL_idx_0);
  tg0 = C2g * tg0_tmp;
  /*  tau_g(0) = M_LEG*g*x_off(H) */
  tg0_r_tmp = sin(cR_idx_0);
  tg0_r = C2g_r * tg0_r_tmp;
  memset(&M[0], 0, 25U * sizeof(double));
  M[0] = (0.00072123659251607884 * sched_lL / 0.060000002144914348 +
          0.035891517958839728 * sched_lL) +
         0.079276605183840845 * rL_idx_1;
  M[2] = 1.3212767058302579 * rL_idx_0 * rL_idx_1 - rL_idx_2;
  M[6] = (0.00072123659251607884 * sched_lR / 0.060000002144914348 +
          0.035891517958839728 * sched_lR) +
         0.079276605183840845 * rR_idx_1;
  M[8] = 1.3212767058302579 * rR_idx_0 * rR_idx_1 - rR_idx_2;
  M[12] =
      -(0.079276605183840845 * rL_idx_0 + 0.64602748742033478 * sched_lL / 2.0);
  M[13] =
      -(0.079276605183840845 * rR_idx_0 + 0.64602748742033478 * sched_lR / 2.0);
  M[10] = -0.027012149543549531;
  M[15] = -0.0043296421992597822;
  M[11] = -0.027012149543549531;
  M[16] = -0.0043296421992597822;
  M[17] = 1.3212767058302579 * rL_idx_0 * -0.034041082972946314;
  M[18] = 1.3212767058302579 * rR_idx_0 * -0.034041082972946314;
  M[19] = -0.033195967438518492;
  M[20] =
      I_z * 0.060000002144914348 / 0.41440003470893561 + 0.0024906704857288985;
  M[21] = -M[20];
  M[22] = I_z * sched_lL / 0.41440003470893561;
  M[23] = -I_z * sched_lR / 0.41440003470893561;
  memset(&Nx[0], 0, 15U * sizeof(double));
  /*  N_s = -(方程里除 ddq 外的其余项)，故整体取负 */
  Nx[0] = -kB;
  Nx[4] = -kB_r;
  /*  The output adds each leg's gravity torque to TB. Its reaction acts on */
  /*  the body too; include it with the body's signed horizontal first moment.
   */
  /*  kB already contains the vertical leg first moment. Retain the missing */
  /*  horizontal first-moment bias once, without adding its stiffness twice. */
  Nx[11] = 3.4999417623887892;
  body_constant = (tg0 + tg0_r) - 1.826603543884421;
  /*  同上：Nu = -∂(其余项)/∂T */
  Nu[0] = sched_lL / 0.060000002144914348 + 1.0;
  Nu[1] = 0.0;
  Nu[2] = -1.0;
  Nu[3] = 0.0;
  Nu[4] = 0.0;
  Nu[5] = sched_lR / 0.060000002144914348 + 1.0;
  Nu[6] = 0.0;
  Nu[7] = -1.0;
  Nu[8] = -1.0;
  Nu[12] = -0.56735136260043728;
  Nu[16] = 3.4533334991227207;
  Nu[9] = -1.0;
  Nu[13] = -0.56735136260043728;
  Nu[17] = -3.4533334991227207;
  Nu[10] = 0.0;
  Nu[14] = 1.0;
  Nu[18] = 0.0;
  Nu[11] = 0.0;
  Nu[15] = 1.0;
  Nu[19] = 0.0;
  /*  s/ds use the mean hip position/velocity consumed from body.vel. */
  /*  At fixed measured lengths: s_hip=s_wheel+(lL*psiL+lR*psiR)/2. */
  T[0] = 0.030000001072457174;
  T[1] = 0.030000001072457174;
  T[2] = sched_lL / 2.0;
  T[3] = sched_lR / 2.0;
  T[4] = 0.0;
  T[5] = -0.14478763783660606;
  T[6] = 0.14478763783660606;
  T[7] = -sched_lL / 0.41440003470893561;
  T[8] = sched_lR / 0.41440003470893561;
  T[9] = 0.0;
  for (k = 0; k < 5; k++) {
    T[k + 10] = iv[k];
    T[k + 15] = iv1[k];
    T[k + 20] = iv2[k];
  }
  memset(&op->A_bal[0], 0, 100U * sizeof(double));
  for (k = 0; k < 25; k++) {
    b_I[k] = 0;
  }
  for (k = 0; k < 5; k++) {
    b_I[k + 5 * k] = 1;
  }
  for (k = 0; k < 5; k++) {
    for (b_k = 0; b_k < 5; b_k++) {
      op->A_bal[((b_k << 1) + 10 * (k << 1)) + 1] = b_I[b_k + 5 * k];
    }
  }
  mldivide(M, Nx, b_dv);
  for (k = 0; k < 3; k++) {
    for (b_k = 0; b_k < 5; b_k++) {
      b_I_z_tmp = 0.0;
      for (I_z_tmp = 0; I_z_tmp < 5; I_z_tmp++) {
        b_I_z_tmp += b_dv[k + 3 * I_z_tmp] * T[I_z_tmp + 5 * b_k];
      }
      op->A_bal[((k << 1) + 10 * ((b_k << 1) + 1)) + 4] = b_I_z_tmp;
    }
  }
  memset(&op->B_bal[0], 0, 40U * sizeof(double));
  b_mldivide(M, Nu, b_dv1);
  for (k = 0; k < 4; k++) {
    for (b_k = 0; b_k < 5; b_k++) {
      b_I_z_tmp = 0.0;
      for (I_z_tmp = 0; I_z_tmp < 5; I_z_tmp++) {
        b_I_z_tmp += b_dv1[k + (I_z_tmp << 2)] * T[I_z_tmp + 5 * b_k];
      }
      op->B_bal[k + (((b_k << 1) + 1) << 2)] = b_I_z_tmp;
    }
  }
  memset(&op->f_bal[0], 0, 10U * sizeof(double));
  y[0] = 0.0;
  y[1] = 0.0;
  y[2] = 0.0;
  y[3] = body_constant;
  y[4] = 0.0;
  c_mldivide(M, y);
  for (k = 0; k < 5; k++) {
    b_I_z_tmp = 0.0;
    for (b_k = 0; b_k < 5; b_k++) {
      b_I_z_tmp += y[b_k] * T[b_k + 5 * k];
    }
    op->f_bal[(k << 1) + 1] = b_I_z_tmp;
  }
  op->aux.l_wl = rL_idx_0;
  op->aux.l_bl = rL_idx_1;
  op->aux.I_ll = rL_idx_2;
  op->aux.theta0 = cL_idx_0;
  op->aux.l_d = cL_idx_1;
  op->aux.l_wr = rR_idx_0;
  op->aux.l_br = rR_idx_1;
  op->aux.I_lr = rR_idx_2;
  op->aux.theta0_r = cR_idx_0;
  op->aux.l_d_r = cR_idx_1;
  op->aux.kA0 = 65.774469692826955 * sched_lL - C2g * cos(cL_idx_0);
  op->aux.kA0_r = 65.774469692826955 * sched_lR - C2g_r * cos(cR_idx_0);
  op->aux.kA = kB;
  op->aux.kA_r = kB_r;
  op->aux.kB = kB;
  op->aux.kB_r = kB_r;
  op->aux.tg0 = tg0;
  op->aux.tg0_r = tg0_r;
  op->aux.xoff = cL_idx_1 * tg0_tmp;
  op->aux.xoff_r = cR_idx_1 * tg0_r_tmp;
  op->aux.Gconst = 0.0;
  op->aux.Gconst_r = 0.0;
  op->aux.theta_star = 0.0;
  op->aux.theta_star_l = 0.0;
  op->aux.theta_star_r = 0.0;
  op->aux.body_pitch_slope = -3.4999417623887892;
  op->aux.body_pitch_constant = body_constant;
  op->aux.leg_body_slope[0] = Nx[9];
  op->aux.leg_body_slope[1] = Nx[10];
  op->lL = sched_lL;
  op->lR = sched_lR;
  op->H = sched_H;
  /*  sched.H is absolute; only x(13) uses H-H_REF. */
  /*  Old and training configurations use the wheel-excluded support mass. */
  /*  (dV/dH)/g of the coupled leg mechanism at nominal height */
  /*  气弹簧等效刚度：'spring' 按 H 中心差分 dFs/dL0；'fixed' 取 cfg.K_EFF */
  op->K_eff = 0.0;
  op->psi = sched_psi;
  /*  双轮无侧滑的 CAD 升降/横滚切向惯量与重力，坐标顺序 [H,roll]。 */
  /*  分段线性插值 + 端点钳位（与 Leg_Comp.c 同口径） */
  body_constant = fmin(fmax(sched_H, 0.15), 0.32110177111662558);
  k = 1;
  while ((k < 20) && (body_constant >= dv3[k << 3])) {
    k++;
  }
  I_z_tmp = (k - 1) << 3;
  b_I_z_tmp = dv3[I_z_tmp];
  b_k = k << 3;
  b_I_z_tmp = (body_constant - b_I_z_tmp) / (dv3[b_k] - b_I_z_tmp);
  for (k = 0; k < 7; k++) {
    body_constant = dv3[(k + I_z_tmp) + 1];
    row[k] = body_constant + b_I_z_tmp * (dv3[(k + b_k) + 1] - body_constant);
  }
  op->M_hr[0] = row[0];
  op->M_hr[1] = row[1];
  op->M_hr[2] = row[1];
  op->M_hr[3] = row[2];
  op->G_hr[0] = row[3];
  op->G_hr[1] = row[4];
  op->I_roll = row[2];
  op->M_v = row[0];
  op->M_load = row[3] / 9.81;
}

/*
 * File trailer for tinympc14_fused_op.c
 *
 * [EOF]
 */

/*
 * File: tinympc14_lpv_kernel.c
 *
 * MATLAB Coder version            : 24.1

 */

/* Include Files */
/*
 * File: tinympc14_lpv_kernel.h
 *
 * MATLAB Coder version            : 24.1

 */

#ifndef TINYMPC14_LPV_KERNEL_H
#define TINYMPC14_LPV_KERNEL_H

/* Include Files */


#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void tinympc14_lpv_kernel(const struct4_T *table, const double rdiag[6],
                                 double left, double right, double psi,
                                 double Ad[196], double Bd[84], double fd[14],
                                 double K[84], double P[196], double Qu[36],
                                 double At[196]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for tinympc14_lpv_kernel.h
 *
 * [EOF]
 */



/*
 * File: tinympc14_scheduled_zoh.h
 *
 * MATLAB Coder version            : 24.1

 */

#ifndef TINYMPC14_SCHEDULED_ZOH_H
#define TINYMPC14_SCHEDULED_ZOH_H

/* Include Files */

#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
double tinympc14_scheduled_zoh(double left, double right, double psi,
                               double Ad[196], double Bd[84], double fd[14]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for tinympc14_scheduled_zoh.h
 *
 * [EOF]
 */

/*
 * File: tinympc14_terminal_cubic_eval.h
 *
 * MATLAB Coder version            : 24.1

 */

#ifndef TINYMPC14_TERMINAL_CUBIC_EVAL_H
#define TINYMPC14_TERMINAL_CUBIC_EVAL_H

/* Include Files */

#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void tinympc14_terminal_cubic_eval(const double table_length_grid[19],
                                   const double table_angle_axis[33],
                                   double table_angle_scale,
                                   const float table_basis[4536],
                                   const float table_offset[189], double left,
                                   double right, double psi, double P[196],
                                   double K[84]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for tinympc14_terminal_cubic_eval.h
 *
 * [EOF]
 */

#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Keep weights and terminal coefficients as read-only data, outside arithmetic.
 *
 * Arguments    : const struct4_T *table
 *                const double rdiag[6]
 *                double left
 *                double right
 *                double psi
 *                double Ad[196]
 *                double Bd[84]
 *                double fd[14]
 *                double K[84]
 *                double P[196]
 *                double Qu[36]
 *                double At[196]
 * Return Type  : void
 */
void tinympc14_lpv_kernel(const struct4_T *table, const double rdiag[6],
                          double left, double right, double psi, double Ad[196],
                          double Bd[84], double fd[14], double K[84],
                          double P[196], double Qu[36], double At[196])
{
  static const double y[36] = {
      0.001, 0.0, 0.0,   0.0, 0.0,   0.0, 0.0, 0.001, 0.0, 0.0,   0.0, 0.0,
      0.0,   0.0, 0.001, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.001, 0.0, 0.0,
      0.0,   0.0, 0.0,   0.0, 0.001, 0.0, 0.0, 0.0,   0.0, 0.0,   0.0, 0.001};
  static const signed char X[36] = {1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
                                    0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0,
                                    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1};
  double b_P[84];
  double A[36];
  double d[36];
  double smax;
  int A_tmp;
  int b_i;
  int i;
  int j;
  int jA;
  int jBcol;
  int jp1j;
  int k;
  signed char ipiv[6];
  /*  Shared MATLAB/C scheduler. Round the model before deriving its
   * auxiliaries, */
  /*  matching the float matrices consumed by the deployed Riccati recursion. */
  smax = tinympc14_scheduled_zoh(left, right, psi, Ad, Bd, fd);
  for (i = 0; i < 196; i++) {
    Ad[i] = (float)Ad[i];
  }
  for (i = 0; i < 84; i++) {
    Bd[i] = (float)Bd[i];
  }
  for (i = 0; i < 14; i++) {
    fd[i] = (float)fd[i];
  }
  tinympc14_terminal_cubic_eval(table->length_grid, table->angle_axis,
                                table->angle_scale, table->basis, table->offset,
                                left, right, psi - smax, P, K);
  for (i = 0; i < 196; i++) {
    P[i] = (float)P[i];
  }
  for (i = 0; i < 84; i++) {
    K[i] = (float)K[i];
  }
  memset(&d[0], 0, 36U * sizeof(double));
  for (j = 0; j < 6; j++) {
    d[j + 6 * j] = rdiag[j];
  }
  for (i = 0; i < 14; i++) {
    for (jp1j = 0; jp1j < 6; jp1j++) {
      smax = 0.0;
      for (jA = 0; jA < 14; jA++) {
        smax += P[i + 14 * jA] * Bd[jp1j + 6 * jA];
      }
      b_P[i + 14 * jp1j] = smax;
    }
  }
  for (i = 0; i < 6; i++) {
    for (jp1j = 0; jp1j < 6; jp1j++) {
      smax = 0.0;
      for (jA = 0; jA < 14; jA++) {
        smax += Bd[jp1j + 6 * jA] * b_P[jA + 14 * i];
      }
      A_tmp = jp1j + 6 * i;
      A[i + 6 * jp1j] = (d[A_tmp] + y[A_tmp]) + smax;
    }
    ipiv[i] = (signed char)(i + 1);
  }
  for (j = 0; j < 5; j++) {
    int b_tmp;
    int mmj_tmp;
    mmj_tmp = 4 - j;
    b_tmp = j * 7;
    jp1j = b_tmp + 2;
    jA = 6 - j;
    jBcol = 0;
    smax = fabs(A[b_tmp]);
    for (k = 2; k <= jA; k++) {
      double s;
      s = fabs(A[(b_tmp + k) - 1]);
      if (s > smax) {
        jBcol = k - 1;
        smax = s;
      }
    }
    if (A[b_tmp + jBcol] != 0.0) {
      if (jBcol != 0) {
        jA = j + jBcol;
        ipiv[j] = (signed char)(jA + 1);
        for (k = 0; k < 6; k++) {
          jBcol = j + k * 6;
          smax = A[jBcol];
          A_tmp = jA + k * 6;
          A[jBcol] = A[A_tmp];
          A[A_tmp] = smax;
        }
      }
      i = (b_tmp - j) + 6;
      for (b_i = jp1j; b_i <= i; b_i++) {
        A[b_i - 1] /= A[b_tmp];
      }
    }
    jA = b_tmp;
    for (jBcol = 0; jBcol <= mmj_tmp; jBcol++) {
      smax = A[(b_tmp + jBcol * 6) + 6];
      if (smax != 0.0) {
        i = jA + 8;
        jp1j = (jA - j) + 12;
        for (A_tmp = i; A_tmp <= jp1j; A_tmp++) {
          A[A_tmp - 1] += A[((b_tmp + A_tmp) - jA) - 7] * -smax;
        }
      }
      jA += 6;
    }
  }
  for (i = 0; i < 36; i++) {
    d[i] = X[i];
  }
  for (b_i = 0; b_i < 5; b_i++) {
    signed char i1;
    i1 = ipiv[b_i];
    if (i1 != b_i + 1) {
      for (j = 0; j < 6; j++) {
        jBcol = b_i + 6 * j;
        jA = (int)d[jBcol];
        A_tmp = (i1 + 6 * j) - 1;
        d[jBcol] = d[A_tmp];
        d[A_tmp] = jA;
      }
    }
  }
  for (j = 0; j < 6; j++) {
    jBcol = 6 * j;
    for (k = 0; k < 6; k++) {
      jA = 6 * k;
      i = k + jBcol;
      if (d[i] != 0.0) {
        jp1j = k + 2;
        for (b_i = jp1j; b_i < 7; b_i++) {
          A_tmp = (b_i + jBcol) - 1;
          d[A_tmp] -= d[i] * A[(b_i + jA) - 1];
        }
      }
    }
  }
  for (j = 0; j < 6; j++) {
    jBcol = 6 * j;
    for (k = 5; k >= 0; k--) {
      jA = 6 * k;
      i = k + jBcol;
      smax = d[i];
      if (smax != 0.0) {
        d[i] = smax / A[k + jA];
        for (b_i = 0; b_i < k; b_i++) {
          A_tmp = b_i + jBcol;
          d[A_tmp] -= d[i] * A[b_i + jA];
        }
      }
    }
  }
  for (i = 0; i < 6; i++) {
    for (jp1j = 0; jp1j < 6; jp1j++) {
      Qu[jp1j + 6 * i] = (float)d[i + 6 * jp1j];
    }
  }
  for (i = 0; i < 14; i++) {
    for (jp1j = 0; jp1j < 14; jp1j++) {
      smax = 0.0;
      for (jA = 0; jA < 6; jA++) {
        smax += K[jp1j + 14 * jA] * Bd[jA + 6 * i];
      }
      At[i + 14 * jp1j] = (float)(Ad[jp1j + 14 * i] - smax);
    }
  }
}

/*
 * File trailer for tinympc14_lpv_kernel.c
 *
 * [EOF]
 */

/*
 * File: tinympc14_lpv_kernel_data.c
 *
 * MATLAB Coder version            : 24.1

 */

/* Include Files */



/* Variable Definitions */
const double dv[63] = {0.15,
                       0.082034064849397431,
                       0.084232005225319539,
                       0.15900535637455923,
                       0.081928797634362976,
                       0.084117854824074439,
                       0.16801071274911847,
                       0.081779503044520752,
                       0.083956290629141686,
                       0.17701606912367771,
                       0.081591541571948742,
                       0.083753082020965347,
                       0.18602142549823697,
                       0.081369005348102763,
                       0.083512632697047923,
                       0.19502678187279621,
                       0.081115061240274317,
                       0.0832383501508783,
                       0.20403213824735544,
                       0.080832190206414647,
                       0.082932903421911133,
                       0.21303749462191468,
                       0.080522357271082631,
                       0.082598406125790808,
                       0.22204285099647392,
                       0.080187134185251335,
                       0.082236548526765513,
                       0.23104820737103315,
                       0.079827789236793179,
                       0.0818486942332933,
                       0.24005356374559239,
                       0.079445353881198472,
                       0.081435951930934267,
                       0.24905892012015166,
                       0.079040672768728656,
                       0.08099922923493523,
                       0.25806427649471086,
                       0.0786144417118314,
                       0.08053927355590805,
                       0.26706963286927016,
                       0.078167236777711549,
                       0.08005670340833107,
                       0.27,
                       0.078017274610080622,
                       0.079894885554682868,
                       0.27607498924382934,
                       0.077699536767359009,
                       0.079552032596802755,
                       0.28508034561838863,
                       0.077211740704734189,
                       0.079025689028189544,
                       0.29408570199294787,
                       0.076704181512740277,
                       0.078478029416198566,
                       0.30309105836750705,
                       0.076177136733959108,
                       0.077909350801568536,
                       0.31209641474206634,
                       0.075630836922463668,
                       0.0773198995613466,
                       0.32110177111662558,
                       0.075065472160159991,
                       0.076709878394237041};

const double dv1[80] = {0.15,
                        0.13285333875533914,
                        0.017146661244660862,
                        0.012359332452696783,
                        0.15900535637455923,
                        0.13777448603489867,
                        0.021230870339660559,
                        0.012494559690068267,
                        0.16801071274911847,
                        0.14285506530716294,
                        0.025155647441955541,
                        0.012633815663394516,
                        0.17701606912367771,
                        0.14807055082698448,
                        0.028945518296693236,
                        0.012777588717444492,
                        0.18602142549823697,
                        0.1534012023711584,
                        0.032620223127078567,
                        0.012926235673268165,
                        0.19502678187279621,
                        0.15883095128247465,
                        0.036195830590321554,
                        0.013080020499410247,
                        0.20403213824735544,
                        0.16434658385206005,
                        0.0396855543952954,
                        0.013239140183438948,
                        0.21303749462191468,
                        0.16993713339853045,
                        0.043100361223384238,
                        0.013403742406244439,
                        0.22204285099647392,
                        0.17559342135312819,
                        0.046449429643345717,
                        0.013573937836804049,
                        0.23104820737103315,
                        0.1813077063907314,
                        0.049740500980301743,
                        0.013749808816440711,
                        0.24005356374559239,
                        0.18707341302884123,
                        0.052980150716751176,
                        0.013931415566875559,
                        0.24905892012015166,
                        0.19288491943529135,
                        0.056174000684860323,
                        0.014118800663905896,
                        0.25806427649471086,
                        0.19873738987742395,
                        0.0593268866172869,
                        0.014311992269231754,
                        0.26706963286927016,
                        0.20462664119718249,
                        0.062442991672087667,
                        0.014511006451003238,
                        0.27607498924382934,
                        0.21054903547956783,
                        0.065525953764261527,
                        0.014715848815603862,
                        0.28508034561838863,
                        0.21650139306695559,
                        0.068578952551433051,
                        0.014926515598724171,
                        0.29408570199294787,
                        0.22248092150372248,
                        0.071604780489225389,
                        0.015142994310087005,
                        0.30309105836750705,
                        0.22848515703855013,
                        0.07460590132895692,
                        0.015365263984742215,
                        0.31209641474206634,
                        0.23451191607615796,
                        0.0775844986659084,
                        0.01559329505837467,
                        0.32110177111662558,
                        0.24055925453130822,
                        0.080542516585317345,
                        0.015827048849249941};

const double dv2[60] = {0.15,
                        1.3668222421591096,
                        0.084648694356346588,
                        0.15900535637455923,
                        1.3193519779847043,
                        0.085332000387800588,
                        0.16801071274911847,
                        1.2741683405964537,
                        0.08606190517905811,
                        0.17701606912367771,
                        1.2309532275233723,
                        0.0868350346319386,
                        0.18602142549823697,
                        1.1894500335092073,
                        0.087648625983976478,
                        0.19502678187279621,
                        1.149448442368852,
                        0.088500349552050092,
                        0.20403213824735544,
                        1.1107733964649134,
                        0.089388190069213863,
                        0.21303749462191468,
                        1.0732769547703174,
                        0.090310366115618546,
                        0.22204285099647392,
                        1.0368321826303217,
                        0.091265274435202021,
                        0.23104820737103315,
                        1.001328491066481,
                        0.092251450829519854,
                        0.24005356374559239,
                        0.966668021967368,
                        0.0932675422810945,
                        0.24905892012015166,
                        0.93276279338050971,
                        0.0943122867989675,
                        0.25806427649471086,
                        0.89953239810458918,
                        0.095384498643867,
                        0.26706963286927016,
                        0.86690210207727947,
                        0.096483057343973236,
                        0.27607498924382934,
                        0.83480122490738384,
                        0.097606899408712783,
                        0.28508034561838863,
                        0.80316170841988144,
                        0.098755011981015647,
                        0.29408570199294787,
                        0.77191679334379337,
                        0.099926427895936948,
                        0.30309105836750705,
                        0.74099973091926374,
                        0.10112022177240385,
                        0.31209641474206634,
                        0.71034245570683774,
                        0.10233550687916107,
                        0.32110177111662558,
                        0.67987413758273552,
                        0.10357143260296643};

const double dv3[168] = {0.15,
                         11.93056227801298,
                         0.017598099227677785,
                         0.4892530050403881,
                         119.53455954378225,
                         0.17749465581229318,
                         21.687494881048845,
                         -3.7573109494815071,
                         0.15900535637455923,
                         11.960890463251848,
                         0.017530825600336364,
                         0.53019155249764416,
                         120.03443729486537,
                         0.17379261054391604,
                         23.187282346465935,
                         -4.101163867176183,
                         0.16801071274911847,
                         11.990819238344693,
                         0.017442600991301266,
                         0.57334388556705729,
                         120.45547136619385,
                         0.17066610814696404,
                         24.605030295842106,
                         -4.3672206970768457,
                         0.17701606912367771,
                         12.019661865325858,
                         0.017342860362548326,
                         0.61863560973753773,
                         120.81341418619108,
                         0.16800162300251209,
                         25.961083213918805,
                         -4.5751675372684986,
                         0.18602142549823697,
                         12.047149993269974,
                         0.01723707012317939,
                         0.66602059905130873,
                         121.12026889113626,
                         0.16571227818916245,
                         27.270067697301659,
                         -4.7391026214427256,
                         0.19502678187279621,
                         12.0732431420737,
                         0.0171283087522838,
                         0.71547007725536849,
                         121.38531133729643,
                         0.1637306754806449,
                         28.542677555658891,
                         -4.8692909189185229,
                         0.20403213824735544,
                         12.098023965192167,
                         0.017018189523026292,
                         0.76696619872200467,
                         121.61580135656334,
                         0.16200387952195561,
                         29.786838952812889,
                         -4.97330695736764,
                         0.21303749462191468,
                         12.121640306305268,
                         0.016907408556565307,
                         0.82049818765603211,
                         121.81748663715356,
                         0.16048984648680698,
                         31.008527133517941,
                         -5.0568349792001754,
                         0.22204285099647392,
                         12.144273401770286,
                         0.016796073734821088,
                         0.87605997612666542,
                         121.99496552055538,
                         0.15915484040107719,
                         32.212284927290781,
                         -5.1241749758901278,
                         0.23104820737103315,
                         12.166121016741888,
                         0.016683902115126367,
                         0.93364874581056234,
                         122.15195210293321,
                         0.15797153674427111,
                         33.401613397564738,
                         -5.1786238373097149,
                         0.24005356374559239,
                         12.187389315498617,
                         0.016570336259006363,
                         0.9932640307403473,
                         122.29147258039424,
                         0.15691761196440845,
                         34.579237627374965,
                         -5.2227338187549321,
                         0.24905892012015166,
                         12.208290010024582,
                         0.016454608853997021,
                         1.0549071804689782,
                         122.41601247816402,
                         0.15597468155322858,
                         35.747299826924078,
                         -5.2584999536975374,
                         0.25806427649471086,
                         12.229040880533347,
                         0.016335772746124383,
                         1.1185810655330846,
                         122.52762830695137,
                         0.15512749147967009,
                         36.907497513363325,
                         -5.287493763717066,
                         0.26706963286927016,
                         12.24986867338184,
                         0.016212706068641045,
                         1.1842899569607734,
                         122.62803312326358,
                         0.15436329602006538,
                         38.061189248678645,
                         -5.3109654629679426,
                         0.27,
                         12.256702386381329,
                         0.016171534199553098,
                         1.2061115540199536,
                         122.65853430509367,
                         0.15413068056763546,
                         38.435389965699152,
                         -5.3175857033238572,
                         0.27607498924382934,
                         12.271013954656473,
                         0.016084097373556983,
                         1.2520395434626388,
                         122.71866271412705,
                         0.15367137423338617,
                         39.209469069706991,
                         -5.3299155808872456,
                         0.28508034561838863,
                         12.292737901636102,
                         0.015948412280340454,
                         1.3218370723326289,
                         122.80072722970264,
                         0.153042650580125,
                         40.353226163302566,
                         -5.3451524024694663,
                         0.29408570199294787,
                         12.315331356486483,
                         0.015803840340827158,
                         1.3936916183073751,
                         122.87525176212441,
                         0.15246939443690732,
                         41.493187937034875,
                         -5.3573332650103112,
                         0.30309105836750705,
                         12.339126828736282,
                         0.015648217962110286,
                         1.4676145026912477,
                         122.94310842861154,
                         0.15194497978092647,
                         42.6299544083304,
                         -5.36699755571135,
                         0.31209641474206634,
                         12.364514598207855,
                         0.015478919717045569,
                         1.543619906815022,
                         123.00504183744044,
                         0.15146369094218803,
                         43.764024872303253,
                         -5.3745923196402146,
                         0.32110177111662558,
                         12.391964748502229,
                         0.015292705409949828,
                         1.6217257543225585,
                         123.06168931153567,
                         0.15102056358856664,
                         44.895818577886921,
                         -5.3804921833489061};

/*
 * File trailer for tinympc14_lpv_kernel_data.c
 *
 * [EOF]
 */

/*
 * File: tinympc14_lpv_kernel_initialize.c
 *
 * MATLAB Coder version            : 24.1

 */

/* Include Files */
/*
 * File: tinympc14_lpv_kernel_initialize.h
 *
 * MATLAB Coder version            : 24.1

 */

#ifndef TINYMPC14_LPV_KERNEL_INITIALIZE_H
#define TINYMPC14_LPV_KERNEL_INITIALIZE_H

/* Include Files */

#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void tinympc14_lpv_kernel_initialize(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for tinympc14_lpv_kernel_initialize.h
 *
 * [EOF]
 */



/* Function Definitions */
/*
 * Arguments    : void
 * Return Type  : void
 */
void tinympc14_lpv_kernel_initialize(void)
{
}

/*
 * File trailer for tinympc14_lpv_kernel_initialize.c
 *
 * [EOF]
 */

/*
 * File: tinympc14_lpv_kernel_terminate.c
 *
 * MATLAB Coder version            : 24.1

 */

/* Include Files */
/*
 * File: tinympc14_lpv_kernel_terminate.h
 *
 * MATLAB Coder version            : 24.1

 */

#ifndef TINYMPC14_LPV_KERNEL_TERMINATE_H
#define TINYMPC14_LPV_KERNEL_TERMINATE_H

/* Include Files */

#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void tinympc14_lpv_kernel_terminate(void);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for tinympc14_lpv_kernel_terminate.h
 *
 * [EOF]
 */



/* Function Definitions */
/*
 * Arguments    : void
 * Return Type  : void
 */
void tinympc14_lpv_kernel_terminate(void)
{
}

/*
 * File trailer for tinympc14_lpv_kernel_terminate.c
 *
 * [EOF]
 */

/*
 * File: tinympc14_scheduled_ab.c
 *
 * MATLAB Coder version            : 24.1

 */

/* Include Files */
/*
 * File: tinympc14_scheduled_ab.h
 *
 * MATLAB Coder version            : 24.1

 */

#ifndef TINYMPC14_SCHEDULED_AB_H
#define TINYMPC14_SCHEDULED_AB_H

/* Include Files */

#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
double tinympc14_scheduled_ab(double left, double right, double psi,
                              double A[196], double B[84], double f[14],
                              double *terminal_center);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for tinympc14_scheduled_ab.h
 *
 * [EOF]
 */







#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Continuous model at one LPV scheduler input, with no toolbox dependency.
 *
 * Arguments    : double left
 *                double right
 *                double psi
 *                double A[196]
 *                double B[84]
 *                double f[14]
 *                double *terminal_center
 * Return Type  : double
 */
double tinympc14_scheduled_ab(double left, double right, double psi,
                              double A[196], double B[84], double f[14],
                              double *terminal_center)
{
  b_struct_T expl_temp_tmp;
  double b_A[28];
  double length_gain[20];
  double x[14];
  double x0[14];
  double xm[14];
  double xp[14];
  double b_B[12];
  double ue[6];
  double b_roll_gain[4];
  double cos_psi[2];
  double gravity_stiffness[2];
  double height_gain[2];
  double roll_gain[2];
  double a21;
  double b;
  double change_idx_0;
  double change_idx_2;
  double d;
  double d1;
  double d2;
  double heave_scale;
  double load_tmp;
  double residual;
  double roll;
  double sched_H;
  double sched_H_tmp;
  double vertical_tmp;
  int a21_tmp;
  int i;
  int r1;
  int r2;
  /*  Keep the continuous model scratch separate from ZOH. */
  /*  Invert the firmware height observation on level ground. The deployment */
  /*  table uses one common sagittal angle for both measured leg lengths. */
  vertical_tmp = cos(psi);
  roll = atan((left - right) * vertical_tmp / 0.41440003470893561);
  sched_H_tmp = cos(roll);
  sched_H = 0.5 * (left + right) * vertical_tmp * sched_H_tmp;
  /*  非线性平衡：两腿同角、alpha=0、theta_b 取 pitch_ref，解腿角与六路输入使 14
   * 态加速度为零 */
  /*  牛顿迭代 + 数值雅可比；返回 x_eq/u_eq 供仿射项锚定 */
  tinympc14_fused_op(sched_H, left, right, psi, &expl_temp_tmp);
  /*  Zero longitudinal and yaw acceleration imply zero wheel torque. */
  /*  The two leg gravity moments and the body pitch moment then fix the */
  /*  common leg lean and hip torques. Vertical and roll balance fix forces. */
  *terminal_center = (expl_temp_tmp.aux.body_pitch_constant -
                      expl_temp_tmp.aux.body_pitch_slope * -0.05) /
                     ((expl_temp_tmp.aux.kB + expl_temp_tmp.aux.kB_r) -
                      (expl_temp_tmp.aux.leg_body_slope[0] +
                       expl_temp_tmp.aux.leg_body_slope[1]));
  a21 = expl_temp_tmp.G_hr[1] - expl_temp_tmp.I_roll * 0.0 * roll;
  /*  两腿分担升降广义载荷。在该分担点的几何倾覆矩已归入外扰。 */
  heave_scale = sched_H_tmp * cos(*terminal_center);
  b = heave_scale * sched_H_tmp / 0.2072000173544678;
  memset(&xp[0], 0, 14U * sizeof(double));
  load_tmp =
      expl_temp_tmp.M_load * 9.81 + expl_temp_tmp.K_eff * (sched_H - 0.27);
  heave_scale *= load_tmp;
  ue[0] = 0.0;
  ue[1] = 0.0;
  xp[4] = *terminal_center;
  ue[2] = -expl_temp_tmp.aux.kB * *terminal_center;
  ue[4] = (heave_scale + a21 * b) / 2.0;
  xp[6] = *terminal_center;
  ue[3] = -expl_temp_tmp.aux.kB_r * *terminal_center;
  ue[5] = (heave_scale + -a21 * b) / 2.0;
  xp[8] = -0.05;
  xp[10] = roll;
  xp[12] = sched_H - 0.27;
  tinympc14_fused_dyn(expl_temp_tmp.A_bal, expl_temp_tmp.B_bal,
                      expl_temp_tmp.f_bal, expl_temp_tmp.K_eff, xp, ue, x);
  residual = 0.0;
  /*  融合 14 态线性化：力投影解析求导，腿长表用割线斜率，仿射项由动力学核反推。
   */
  /*  */
  /*    参考位姿 x_ref = [0 0 0 0 psi_l 0 psi_r 0 pitch_ref 0 alpha 0 H-H_REF
   * 0]， */
  /*    默认参考输入 u_ref = [0 0 0 0 Fbar Fbar]，Fbar = op.M_load*g/2。 */
  /*    f = dyn(x_ref,u_ref) - A*x_ref - B*u_ref，保证展开点处动力学一致。 */
  /*    展开点不一定是平衡点；不得通过改 f 强行制造一个跨调度点的定点。 */
  /*  */
  /*    腿轴力与腿长共轭。双轮平地约束下，H/alpha 两行使用同一腿长雅可比。 */
  /*    轮矩按轮心水平位移作功。psi 取本调度点的腿角。 */
  /*    Production F_pitch=0: direct horizontal force addition double-counts */
  /*    internal support after the balance equations have eliminated internal
   * forces. */
  /*  */
  /*    sw 沿用登记表开关：#1 sw.F_pitch、#2 sw.F_H_proj、#8 sw.Ts_H、#8alpha
   * sw.Ts_alpha */
  /*    sched 可带 psi_l/psi_r（缺省都取 sched.psi） */
  /*  */
  /*    版本：v0.6  日期：2026-10-22 */
  for (r2 = 0; r2 < 14; r2++) {
    heave_scale = fabs(x[r2]);
    if (rtIsNaN(heave_scale) || (heave_scale > residual)) {
      residual = heave_scale;
    }
    x0[r2] = 0.0;
  }
  x0[4] = psi;
  x0[6] = psi;
  x0[8] = -0.05;
  x0[10] = roll;
  x0[12] = sched_H - 0.27;
  /*  input expansion point only; never replace the affine residual */
  memset(&A[0], 0, 196U * sizeof(double));
  memset(&B[0], 0, 84U * sizeof(double));
  for (i = 0; i < 10; i++) {
    memcpy(&A[i * 14], &expl_temp_tmp.A_bal[i * 10], 10U * sizeof(double));
    r2 = i << 2;
    B[6 * i] = expl_temp_tmp.B_bal[r2];
    B[6 * i + 1] = expl_temp_tmp.B_bal[r2 + 1];
    B[6 * i + 2] = expl_temp_tmp.B_bal[r2 + 2];
    B[6 * i + 3] = expl_temp_tmp.B_bal[r2 + 3];
  }
  double side_arm_idx_0;
  double side_arm_idx_1;
  A[151] = 1.0;
  A[181] = 1.0;
  A[194] = -expl_temp_tmp.K_eff / expl_temp_tmp.M_v;
  A[195] = -8.0 / expl_temp_tmp.M_v;
  /*  双轮平地闭链 L_i=(H±b sin(phi))/(cos(phi)cos(psi_i))。 */
  /*  固定两腿矢状角的虚位移 dL_i=height_arm_i*dH+roll_arm_i*dphi。 */
  /*  轴力与 +L 共轭，乘上述两臂分别得到虚升降力和虚横滚力矩。 */
  heave_scale = 1.0 / sched_H_tmp;
  change_idx_0 = tan(roll);
  roll = sin(psi);
  side_arm_idx_0 = 0.2072000173544678 * heave_scale;
  side_arm_idx_1 = -0.2072000173544678 * heave_scale;
  height_gain[0] = heave_scale / vertical_tmp;
  height_gain[1] = heave_scale / vertical_tmp;
  change_idx_2 = sched_H * change_idx_0;
  d = 0.060000002144914348 * expl_temp_tmp.M_v;
  sched_H_tmp = change_idx_0 * change_idx_0;
  heave_scale = sched_H * (sched_H_tmp + heave_scale * heave_scale);
  d1 = height_gain[0] * (change_idx_2 + side_arm_idx_0);
  roll_gain[0] = d1;
  B[82] = height_gain[0] / expl_temp_tmp.M_v;
  B[70] = d1 / 1.2061115540199536;
  d2 = ue[4] + 0.0 * roll / 0.060000002144914348;
  B[78] = height_gain[0] * roll / d;
  B[66] = d1 * roll / 0.0723666958282032;
  b = height_gain[0] * d2;
  gravity_stiffness[0] = b;
  a21 = tan(psi);
  A[186] =
      height_gain[0] * 0.0 * vertical_tmp / d + b * a21 / expl_temp_tmp.M_v;
  A[158] = d1 * 0.0 * vertical_tmp / 0.0723666958282032 +
           d1 * d2 * a21 / 1.2061115540199536;
  side_arm_idx_0 =
      height_gain[0] * (heave_scale + 2.0 * side_arm_idx_0 * change_idx_0) * d2;
  d1 = height_gain[1] * (change_idx_2 + side_arm_idx_1);
  roll_gain[1] = d1;
  B[83] = height_gain[1] / expl_temp_tmp.M_v;
  B[71] = d1 / 1.2061115540199536;
  d2 = ue[5] + 0.0 * roll / 0.060000002144914348;
  B[79] = height_gain[1] * roll / d;
  B[67] = d1 * roll / 0.0723666958282032;
  b = height_gain[1] * d2;
  A[188] =
      height_gain[1] * 0.0 * vertical_tmp / d + b * a21 / expl_temp_tmp.M_v;
  A[160] = d1 * 0.0 * vertical_tmp / 0.0723666958282032 +
           d1 * d2 * a21 / 1.2061115540199536;
  side_arm_idx_1 =
      height_gain[1] * (heave_scale + 2.0 * side_arm_idx_1 * change_idx_0) * d2;
  heave_scale = gravity_stiffness[0] + b;
  A[192] = heave_scale * change_idx_0 / expl_temp_tmp.M_v;
  A[164] = ((side_arm_idx_0 + side_arm_idx_1) -
            load_tmp * sched_H * (sched_H_tmp + 1.0)) /
           1.2061115540199536;
  A[166] = ((heave_scale - load_tmp) - expl_temp_tmp.K_eff * sched_H) *
           change_idx_0 / 1.2061115540199536;
  /*  Vary height and roll at fixed measured sagittal leg angles. Level-ground
   */
  /*  closure gives L_i = (H +/- half_track*sin(roll))/(cos(roll)*cos(psi_i)).
   */
  /*  m, half of the original hardware table spacing */
  cos_psi[0] = fmin(fmax(left, 0.15), 0.32110177111662558);
  cos_psi[1] = fmin(fmax(right, 0.15), 0.32110177111662558);
  d = ue[2];
  d1 = ue[3];
  for (r1 = 0; r1 < 2; r1++) {
    double A0[100];
    double A1[100];
    double B0[40];
    double B1[40];
    double b_x0[10];
    double f0[10];
    double f1[10];
    double sin_psi[2];
    gravity_stiffness[0] = cos_psi[0];
    sin_psi[0] = cos_psi[0];
    gravity_stiffness[1] = cos_psi[1];
    sin_psi[1] = cos_psi[1];
    gravity_stiffness[r1] = fmax(0.15, cos_psi[r1] - 0.005);
    sin_psi[r1] = fmin(0.32110177111662558, cos_psi[r1] + 0.005);
    tinympc14_bal_block(gravity_stiffness[0], gravity_stiffness[1], A0, B0, f0);
    tinympc14_bal_block(sin_psi[0], sin_psi[1], A1, B1, f1);
    for (i = 0; i < 100; i++) {
      A1[i] -= A0[i];
    }
    for (i = 0; i < 10; i++) {
      d2 = 0.0;
      for (a21_tmp = 0; a21_tmp < 10; a21_tmp++) {
        d2 += x0[a21_tmp] * A1[a21_tmp + 10 * i];
      }
      b_x0[i] = d2;
    }
    for (i = 0; i < 40; i++) {
      B1[i] -= B0[i];
    }
    heave_scale = sin_psi[r1] - gravity_stiffness[r1];
    for (i = 0; i < 10; i++) {
      a21_tmp = i << 2;
      length_gain[r1 + (i << 1)] =
          (((b_x0[i] + (((0.0 * B1[a21_tmp] + 0.0 * B1[a21_tmp + 1]) +
                         d * B1[a21_tmp + 2]) +
                        d1 * B1[a21_tmp + 3])) +
            f1[i]) -
           f0[i]) /
          heave_scale;
    }
    r2 = r1 << 1;
    b_roll_gain[r2] = roll_gain[r1];
    b_roll_gain[r2 + 1] = height_gain[r1];
  }
  for (i = 0; i < 2; i++) {
    d = b_roll_gain[i];
    d1 = b_roll_gain[i + 2];
    for (a21_tmp = 0; a21_tmp < 10; a21_tmp++) {
      r2 = a21_tmp << 1;
      r1 = ((i << 1) + 14 * a21_tmp) + 10;
      A[r1] += d * length_gain[r2] + d1 * length_gain[r2 + 1];
    }
  }
  /*  The projected CAD kinetic energy includes H/roll cross inertia. Convert */
  /*  the existing generalized-force derivatives using that same mass matrix. */
  b_roll_gain[1] = 0.0;
  b_roll_gain[2] = 0.0;
  b_roll_gain[0] = expl_temp_tmp.M_v;
  b_roll_gain[3] = 1.2061115540199536;
  if (fabs(expl_temp_tmp.M_hr[2]) > fabs(expl_temp_tmp.M_hr[0])) {
    r1 = 1;
    r2 = 0;
  } else {
    r1 = 0;
    r2 = 1;
  }
  a21_tmp = r2 << 1;
  r2 = r1 << 1;
  heave_scale = expl_temp_tmp.M_hr[r2];
  a21 = expl_temp_tmp.M_hr[a21_tmp] / heave_scale;
  b = expl_temp_tmp.M_hr[r2 + 1];
  roll = expl_temp_tmp.M_hr[a21_tmp + 1] - a21 * b;
  d = b_roll_gain[r2];
  d1 = (b_roll_gain[a21_tmp] - d * a21) / roll;
  change_idx_2 = d1;
  change_idx_0 = (d - d1 * b) / heave_scale;
  d = b_roll_gain[r2 + 1];
  d1 = (b_roll_gain[a21_tmp + 1] - d * a21) / roll;
  heave_scale = (d - d1 * b) / heave_scale;
  for (i = 0; i < 14; i++) {
    d = A[i + 182];
    d2 = A[i + 154];
    b_A[i] = d * change_idx_0 + d2 * heave_scale;
    b_A[i + 14] = d * change_idx_2 + d2 * d1;
  }
  for (i = 0; i < 2; i++) {
    memcpy(&A[i * -28 + 182], &b_A[i * 14], 14U * sizeof(double));
  }
  for (i = 0; i < 6; i++) {
    d = B[i + 78];
    d2 = B[i + 66];
    b_B[i] = d * change_idx_0 + d2 * heave_scale;
    b_B[i + 6] = d * change_idx_2 + d2 * d1;
  }
  for (i = 0; i < 2; i++) {
    for (a21_tmp = 0; a21_tmp < 6; a21_tmp++) {
      B[a21_tmp + 6 * (-2 * i + 13)] = b_B[a21_tmp + 6 * i];
    }
  }
  /*  H changes the CAD mass and gravity tables as well as the geometric arms.
   */
  memcpy(&xp[0], &x0[0], 14U * sizeof(double));
  memcpy(&xm[0], &x0[0], 14U * sizeof(double));
  xp[12] = (sched_H - 0.27) + 1.0E-5;
  xm[12] = (sched_H - 0.27) - 1.0E-5;
  tinympc14_fused_dyn(expl_temp_tmp.A_bal, expl_temp_tmp.B_bal,
                      expl_temp_tmp.f_bal, expl_temp_tmp.K_eff, xp, ue, f);
  tinympc14_fused_dyn(expl_temp_tmp.A_bal, expl_temp_tmp.B_bal,
                      expl_temp_tmp.f_bal, expl_temp_tmp.K_eff, xm, ue, x);
  A[194] = (f[13] - x[13]) / 2.0E-5;
  A[166] = (f[11] - x[11]) / 2.0E-5;
  tinympc14_fused_dyn(expl_temp_tmp.A_bal, expl_temp_tmp.B_bal,
                      expl_temp_tmp.f_bal, expl_temp_tmp.K_eff, x0, ue, f);
  for (i = 0; i < 14; i++) {
    d = 0.0;
    for (a21_tmp = 0; a21_tmp < 14; a21_tmp++) {
      d += x0[a21_tmp] * A[a21_tmp + 14 * i];
    }
    d1 = 0.0;
    for (a21_tmp = 0; a21_tmp < 6; a21_tmp++) {
      d1 += ue[a21_tmp] * B[a21_tmp + 6 * i];
    }
    f[i] = (f[i] - d) - d1;
  }
  /*  平衡块辅助量（tg0/xoff/theta_star 等）原样带出 */
  return residual;
}

/*
 * File trailer for tinympc14_scheduled_ab.c
 *
 * [EOF]
 */

/*
 * File: tinympc14_scheduled_zoh.c
 *
 * MATLAB Coder version            : 24.1

 */

/* Include Files */



/*
 * File: tinympc14_zoh_embedded.h
 *
 * MATLAB Coder version            : 24.1

 */

#ifndef TINYMPC14_ZOH_EMBEDDED_H
#define TINYMPC14_ZOH_EMBEDDED_H

/* Include Files */

#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void tinympc14_zoh_embedded(const double A[196], const double B[84],
                            const double f[14], double Ad[196], double Bd[84],
                            double fd[14]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for tinympc14_zoh_embedded.h
 *
 * [EOF]
 */


/* Function Definitions */
/*
 * Runtime model entry with constant calibrated parameters and three signals.
 *
 * Arguments    : double left
 *                double right
 *                double psi
 *                double Ad[196]
 *                double Bd[84]
 *                double fd[14]
 * Return Type  : double
 */
double tinympc14_scheduled_zoh(double left, double right, double psi,
                               double Ad[196], double Bd[84], double fd[14])
{
  double A[196];
  double B[84];
  double f[14];
  double terminal_center;
  tinympc14_scheduled_ab(left, right, psi, A, B, f, &terminal_center);
  tinympc14_zoh_embedded(A, B, f, Ad, Bd, fd);
  return terminal_center;
}

/*
 * File trailer for tinympc14_scheduled_zoh.c
 *
 * [EOF]
 */

/*
 * File: tinympc14_terminal_cubic_eval.c
 *
 * MATLAB Coder version            : 24.1

 */

/* Include Files */



#include <math.h>
#include <string.h>

/* Function Definitions */
/*
 * Interpolate log-Cholesky coordinates, never the entries of the cost matrix.
 *  psi is relative to the local equilibrium lean. The asinh axis resolves the
 *  narrow gain transition there without changing the physical model or costs.
 *
 * Arguments    : const double table_length_grid[19]
 *                const double table_angle_axis[33]
 *                double table_angle_scale
 *                const float table_basis[4536]
 *                const float table_offset[189]
 *                double left
 *                double right
 *                double psi
 *                double P[196]
 *                double K[84]
 * Return Type  : void
 */
void tinympc14_terminal_cubic_eval(const double table_length_grid[19],
                                   const double table_angle_axis[33],
                                   double table_angle_scale,
                                   const float table_basis[4536],
                                   const float table_offset[189], double left,
                                   double right, double psi, double P[196],
                                   double K[84])
{
  static const double y[196] = {
      0.001, 0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0,
      0.0,   0.0, 0.0, 0.001, 0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0,
      0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.001, 0.0, 0.0, 0.0,   0.0, 0.0,
      0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.001, 0.0, 0.0,
      0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0,
      0.001, 0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0,
      0.0,   0.0, 0.0, 0.001, 0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0,
      0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.001, 0.0, 0.0, 0.0,   0.0, 0.0,
      0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.001, 0.0, 0.0,
      0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0,
      0.001, 0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0,
      0.0,   0.0, 0.0, 0.001, 0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0,
      0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.001, 0.0, 0.0, 0.0,   0.0, 0.0,
      0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.001, 0.0, 0.0,
      0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0,
      0.001, 0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0, 0.0,   0.0, 0.0,
      0.0,   0.0, 0.0, 0.001};
  double L[196];
  double factor[189];
  double coords[24];
  double wl[4];
  double wp[4];
  double wr[4];
  double position;
  double position_tmp;
  int c;
  int element;
  int i;
  int il;
  int ip;
  int j;
  int k;
  short iv[24];
  boolean_T xneg;
  position_tmp = table_length_grid[1] - table_length_grid[0];
  position =
      fmin(fmax((left - table_length_grid[0]) / position_tmp, 0.0), 18.0);
  il = (int)fmin(fmax(floor(position), 1.0), 16.0);
  position -= (double)il - 1.0;
  wl[0] = -(position - 1.0) * (position - 2.0) * (position - 3.0) / 6.0;
  wl[1] = position * (position - 2.0) * (position - 3.0) / 2.0;
  wl[2] = -position * (position - 1.0) * (position - 3.0) / 2.0;
  wl[3] = position * (position - 1.0) * (position - 2.0) / 6.0;
  position =
      fmin(fmax((right - table_length_grid[0]) / position_tmp, 0.0), 18.0);
  element = (int)fmin(fmax(floor(position), 1.0), 16.0);
  position -= (double)element - 1.0;
  wr[0] = -(position - 1.0) * (position - 2.0) * (position - 3.0) / 6.0;
  wr[1] = position * (position - 2.0) * (position - 3.0) / 2.0;
  wr[2] = -position * (position - 1.0) * (position - 3.0) / 2.0;
  wr[3] = position * (position - 1.0) * (position - 2.0) / 6.0;
  position_tmp = psi / table_angle_scale;
  xneg = (position_tmp < 0.0);
  if (xneg) {
    position_tmp = -position_tmp;
  }
  if (position_tmp >= 2.68435456E+8) {
    position_tmp = log(position_tmp) + 0.69314718055994529;
  } else if (position_tmp > 2.0) {
    position_tmp =
        log(2.0 * position_tmp +
            1.0 / (sqrt(position_tmp * position_tmp + 1.0) + position_tmp));
  } else {
    position = position_tmp * position_tmp;
    position_tmp += position / (sqrt(position + 1.0) + 1.0);
    position = fabs(position_tmp);
    if ((position > 4.503599627370496E+15) ||
        (rtIsInf(position_tmp) || rtIsNaN(position_tmp))) {
      position_tmp++;
      position_tmp = log(position_tmp);
    } else if (!(position < 2.2204460492503131E-16)) {
      position_tmp = log(position_tmp + 1.0) *
                     (position_tmp / ((position_tmp + 1.0) - 1.0));
    }
  }
  if (xneg) {
    position_tmp = -position_tmp;
  }
  position = fmin(fmax((position_tmp - table_angle_axis[0]) /
                           (table_angle_axis[1] - table_angle_axis[0]),
                       0.0),
                  32.0);
  ip = (int)fmin(fmax(floor(position), 1.0), 30.0);
  position -= (double)ip - 1.0;
  wp[0] = -(position - 1.0) * (position - 2.0) * (position - 3.0) / 6.0;
  wp[1] = position * (position - 2.0) * (position - 3.0) / 2.0;
  wp[2] = -position * (position - 1.0) * (position - 3.0) / 2.0;
  wp[3] = position * (position - 1.0) * (position - 2.0) / 6.0;
  memset(&coords[0], 0, 24U * sizeof(double));
  for (k = 0; k < 4; k++) {
    for (j = 0; j < 4; j++) {
      for (i = 0; i < 4; i++) {
        position = wl[i] * wr[j] * wp[k];
        /*  Deployment reads just this node from the generated cost unit. */
        /*  Coder specializes this branch for a structure without coords. */
        tinympc14_terminal_decode_node(
            (unsigned int)((((il + i) + 19 * ((element + j) - 1)) +
                            361 * ((ip + k) - 1)) -
                           1),
            &iv[0]);
        for (c = 0; c < 24; c++) {
          coords[c] += position * (double)iv[c];
        }
      }
    }
  }
  for (ip = 0; ip < 189; ip++) {
    position = 0.0;
    for (element = 0; element < 24; element++) {
      position += table_basis[ip + 189 * element] * coords[element];
    }
    factor[ip] = position + table_offset[ip];
  }
  memset(&L[0], 0, 196U * sizeof(double));
  element = 0;
  for (j = 0; j < 14; j++) {
    ip = 13 - j;
    for (i = 0; i <= ip; i++) {
      L[j + 14 * (j + i)] = factor[element + i];
    }
    element = (element - j) + 14;
    il = j + 14 * j;
    L[il] = exp(L[il]);
  }
  for (ip = 0; ip < 14; ip++) {
    for (element = 0; element < 14; element++) {
      position = 0.0;
      for (il = 0; il < 14; il++) {
        position += L[il + 14 * ip] * L[il + 14 * element];
      }
      il = ip + 14 * element;
      P[il] = position + y[il];
    }
  }
  /*  The fallback gain is interpolated directly. Recovering it from a rounded,
   */
  /*  approximate P amplifies small cost errors in weakly weighted directions.
   */
  ip = 0;
  element = 0;
  for (il = 0; il < 84; il++) {
    K[element + 14 * ip] = factor[il + 105];
    ip++;
    if (ip > 5) {
      ip = 0;
      element++;
    }
  }
}

/*
 * File trailer for tinympc14_terminal_cubic_eval.c
 *
 * [EOF]
 */

/*
 * File: tinympc14_zoh_embedded.c
 *
 * MATLAB Coder version            : 24.1

 */

/* Include Files */


#include <string.h>

/* Function Definitions */
/*
 * Fixed-order ZOH candidate for the 2 ms firmware scheduler.
 *  The top rows of the augmented series satisfy a left multiplication by A.
 *
 * Arguments    : const double A[196]
 *                const double B[84]
 *                const double f[14]
 *                double Ad[196]
 *                double Bd[84]
 *                double fd[14]
 * Return Type  : void
 */
void tinympc14_zoh_embedded(const double A[196], const double B[84],
                            const double f[14], double Ad[196], double Bd[84],
                            double fd[14])
{
  double delta[294];
  double term[294];
  double Astep[196];
  int i;
  int i1;
  int i2;
  int k;
  for (i = 0; i < 196; i++) {
    Astep[i] = 0.002 * A[i];
  }
  for (i = 0; i < 14; i++) {
    for (i1 = 0; i1 < 14; i1++) {
      term[i1 + 21 * i] = 0.002 * A[i1 + 14 * i];
    }
    for (i1 = 0; i1 < 6; i1++) {
      term[(i1 + 21 * i) + 14] = 0.002 * B[i1 + 6 * i];
    }
    term[21 * i + 20] = 0.002 * f[i];
  }
  memcpy(&delta[0], &term[0], 294U * sizeof(double));
  for (k = 0; k < 4; k++) {
    double b_term[294];
    double d;
    for (i = 0; i < 21; i++) {
      for (i1 = 0; i1 < 14; i1++) {
        d = 0.0;
        for (i2 = 0; i2 < 14; i2++) {
          d += term[i + 21 * i2] * Astep[i2 + 14 * i1];
        }
        b_term[i + 21 * i1] = d / ((double)k + 2.0);
      }
    }
    for (i = 0; i < 294; i++) {
      d = b_term[i];
      term[i] = d;
      delta[i] += d;
    }
  }
  memset(&Ad[0], 0, 196U * sizeof(double));
  for (k = 0; k < 14; k++) {
    Ad[k + 14 * k] = 1.0;
  }
  for (i = 0; i < 14; i++) {
    for (i1 = 0; i1 < 14; i1++) {
      k = i1 + 14 * i;
      Ad[k] += delta[i1 + 21 * i];
    }
    for (i1 = 0; i1 < 6; i1++) {
      Bd[i1 + 6 * i] = delta[(i1 + 21 * i) + 14];
    }
    fd[i] = delta[21 * i + 20];
  }
}

/*
 * File trailer for tinympc14_zoh_embedded.c
 *
 * [EOF]
 */

/*
 * File: xgetrf.c
 *
 * MATLAB Coder version            : 24.1

 */

/* Include Files */


#include <math.h>

/* Function Definitions */
/*
 * Arguments    : double A[25]
 *                int ipiv[5]
 * Return Type  : int
 */
int xgetrf(double A[25], int ipiv[5])
{
  int i;
  int info;
  int j;
  int jA;
  int jp1j;
  int k;
  for (i = 0; i < 5; i++) {
    ipiv[i] = i + 1;
  }
  info = 0;
  for (j = 0; j < 4; j++) {
    double smax;
    int a;
    int b_tmp;
    int mmj_tmp;
    mmj_tmp = 3 - j;
    b_tmp = j * 6;
    jp1j = b_tmp + 2;
    jA = 5 - j;
    a = 0;
    smax = fabs(A[b_tmp]);
    for (k = 2; k <= jA; k++) {
      double s;
      s = fabs(A[(b_tmp + k) - 1]);
      if (s > smax) {
        a = k - 1;
        smax = s;
      }
    }
    if (A[b_tmp + a] != 0.0) {
      if (a != 0) {
        jA = j + a;
        ipiv[j] = jA + 1;
        for (k = 0; k < 5; k++) {
          a = j + k * 5;
          smax = A[a];
          i = jA + k * 5;
          A[a] = A[i];
          A[i] = smax;
        }
      }
      i = (b_tmp - j) + 5;
      for (jA = jp1j; jA <= i; jA++) {
        A[jA - 1] /= A[b_tmp];
      }
    } else {
      info = j + 1;
    }
    jA = b_tmp;
    for (jp1j = 0; jp1j <= mmj_tmp; jp1j++) {
      smax = A[(b_tmp + jp1j * 5) + 5];
      if (smax != 0.0) {
        i = jA + 7;
        a = (jA - j) + 10;
        for (k = i; k <= a; k++) {
          A[k - 1] += A[((b_tmp + k) - jA) - 6] * -smax;
        }
      }
      jA += 5;
    }
  }
  if ((info == 0) && (!(A[24] != 0.0))) {
    info = 5;
  }
  return info;
}

/*
 * File trailer for xgetrf.c
 *
 * [EOF]
 */

void Tinympc14_Lpv_Data_Update(float left, float right, float psi, tinympc14_model_t *mdl)
{
    /* One scheduler task owns this scratch; no large control-task stack frame. */
    static double ad[196], bd[84], fd[14], k[84], p[196], qu[36], at[196];
    tinympc14_lpv_kernel(&tinympc14_terminal, tinympc14_terminal_r, (double)left, (double)right, (double)psi, ad, bd, fd, k, p, qu, at);
    for (int i=0;i<196;i++) { mdl->Ad[i]=(float)ad[i]; mdl->Pinf[i]=(float)p[i]; mdl->AmBKt[i]=(float)at[i]; }
    for (int i=0;i<84;i++) { mdl->Bd[i]=(float)bd[i]; mdl->Kinf[i]=(float)k[i]; }
    for (int i=0;i<14;i++) mdl->fd[i]=(float)fd[i];
    for (int i=0;i<36;i++) mdl->QuuInv[i]=(float)qu[i];
}
