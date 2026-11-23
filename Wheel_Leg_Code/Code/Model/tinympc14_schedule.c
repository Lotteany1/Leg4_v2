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
extern const double dv[84];

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
  double b_I[25];
  double Nu[20];
  double dv1[20];
  double Nx[15];
  double b_dv[15];
  double a;
  double a_tmp;
  double rL_idx_0;
  double rL_idx_1;
  double rL_idx_2_tmp;
  double rR_idx_1;
  int a_tmp_tmp;
  int b_a_tmp;
  int k;
  /*  分段线性插值 + 端点钳位（与 Leg_Comp.c 同口径） */
  rR_idx_1 = fmin(fmax(lL, 0.12), 0.32);
  k = 1;
  while ((k < 20) && (rR_idx_1 >= dv[k << 2])) {
    k++;
  }
  a_tmp_tmp = (k - 1) << 2;
  a_tmp = dv[a_tmp_tmp];
  b_a_tmp = k << 2;
  a = (rR_idx_1 - a_tmp) / (dv[b_a_tmp] - a_tmp);
  rR_idx_1 = dv[a_tmp_tmp + 1];
  rL_idx_0 = rR_idx_1 + a * (dv[b_a_tmp + 1] - rR_idx_1);
  rR_idx_1 = dv[a_tmp_tmp + 2];
  rL_idx_1 = rR_idx_1 + a * (dv[b_a_tmp + 2] - rR_idx_1);
  rL_idx_2_tmp = dv[a_tmp_tmp + 3];
  /*  分段线性插值 + 端点钳位（与 Leg_Comp.c 同口径） */
  rR_idx_1 = fmin(fmax(lR, 0.12), 0.32);
  k = 1;
  while ((k < 20) && (rR_idx_1 >= dv[k << 2])) {
    k++;
  }
  double b_a;
  double rR_idx_0;
  a_tmp_tmp = (k - 1) << 2;
  a_tmp = dv[a_tmp_tmp];
  k <<= 2;
  b_a = (rR_idx_1 - a_tmp) / (dv[k] - a_tmp);
  rR_idx_1 = dv[a_tmp_tmp + 1];
  rR_idx_0 = rR_idx_1 + b_a * (dv[k + 1] - rR_idx_1);
  rR_idx_1 = dv[a_tmp_tmp + 2];
  rR_idx_1 += b_a * (dv[k + 2] - rR_idx_1);
  a_tmp = dv[a_tmp_tmp + 3];
  /*  分段线性插值 + 端点钳位（与 Leg_Comp.c 同口径） */
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
         (rL_idx_2_tmp + a * (dv[b_a_tmp + 3] - rL_idx_2_tmp));
  M[6] = (0.00072123659251607884 * lR / 0.060000002144914348 +
          0.035891517958839728 * lR) +
         0.079276605183840845 * rR_idx_1;
  M[8] = 1.3212767058302579 * rR_idx_0 * rR_idx_1 -
         (a_tmp + b_a * (dv[k + 3] - a_tmp));
  M[12] = -(0.079276605183840845 * rL_idx_0 + 0.64602748742033467 * lL / 2.0);
  M[13] = -(0.079276605183840845 * rR_idx_0 + 0.64602748742033467 * lR / 2.0);
  M[10] = -0.027012149543549524;
  M[15] = -0.0043296421992597822;
  M[11] = -0.027012149543549524;
  M[16] = -0.0043296421992597822;
  M[17] = 1.3212767058302579 * rL_idx_0 * -0.034041082972946314;
  M[18] = 1.3212767058302579 * rR_idx_0 * -0.034041082972946314;
  M[19] = -0.033195967438518492;
  M[20] = 0.02858919234796653;
  M[21] = -0.02858919234796653;
  M[22] = 0.18025379964890378 * lL / 0.41440003470893561;
  M[23] = -0.18025379964890378 * lR / 0.41440003470893561;
  memset(&Nx[0], 0, 15U * sizeof(double));
  /*  N_s = -(方程里除 ddq 外的其余项)，故整体取负 */
  Nx[0] =
      -((1.3212767058302579 * rL_idx_0 + 10.767124405429584 * lL / 2.0) * 9.81);
  Nx[4] =
      -((1.3212767058302579 * rR_idx_0 + 10.767124405429584 * lR / 2.0) * 9.81);
  Nx[11] = 3.5956060833522376;
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
  for (b_a_tmp = 0; b_a_tmp < 5; b_a_tmp++) {
    T[b_a_tmp + 10] = iv[b_a_tmp];
    T[b_a_tmp + 15] = iv1[b_a_tmp];
    T[b_a_tmp + 20] = iv2[b_a_tmp];
  }
  memset(&A[0], 0, 100U * sizeof(double));
  memset(&b_I[0], 0, 25U * sizeof(double));
  for (k = 0; k < 5; k++) {
    b_I[k + 5 * k] = 1.0;
  }
  for (b_a_tmp = 0; b_a_tmp < 5; b_a_tmp++) {
    for (k = 0; k < 5; k++) {
      A[((k << 1) + 10 * (b_a_tmp << 1)) + 1] = b_I[k + 5 * b_a_tmp];
    }
  }
  mldivide(M, Nx, b_dv);
  for (b_a_tmp = 0; b_a_tmp < 3; b_a_tmp++) {
    for (k = 0; k < 5; k++) {
      rR_idx_1 = 0.0;
      for (a_tmp_tmp = 0; a_tmp_tmp < 5; a_tmp_tmp++) {
        rR_idx_1 += b_dv[b_a_tmp + 3 * a_tmp_tmp] * T[a_tmp_tmp + 5 * k];
      }
      A[((b_a_tmp << 1) + 10 * ((k << 1) + 1)) + 4] = rR_idx_1;
    }
  }
  memset(&B[0], 0, 40U * sizeof(double));
  b_mldivide(M, Nu, dv1);
  for (b_a_tmp = 0; b_a_tmp < 4; b_a_tmp++) {
    for (k = 0; k < 5; k++) {
      rR_idx_1 = 0.0;
      for (a_tmp_tmp = 0; a_tmp_tmp < 5; a_tmp_tmp++) {
        rR_idx_1 += dv1[b_a_tmp + (a_tmp_tmp << 2)] * T[a_tmp_tmp + 5 * k];
      }
      B[b_a_tmp + (((k << 1) + 1) << 2)] = rR_idx_1;
    }
  }
  memset(&f[0], 0, 10U * sizeof(double));
  for (k = 0; k < 5; k++) {
    for (b_a_tmp = 0; b_a_tmp < 5; b_a_tmp++) {
      b_I[b_a_tmp + 5 * k] = M[k + 5 * b_a_tmp];
    }
  }
  int ipiv[5];
  xgetrf(b_I, ipiv);
  for (b_a_tmp = 0; b_a_tmp < 5; b_a_tmp++) {
    rR_idx_1 = 0.0;
    for (k = 0; k < 5; k++) {
      rR_idx_1 += 0.0 * T[k + 5 * b_a_tmp];
    }
    f[(b_a_tmp << 1) + 1] = rR_idx_1;
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
                         const double op_f_bal[10], double op_M_v,
                         double op_M_load, double op_K_eff, double op_I_roll,
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
 *                double op_M_v
 *                double op_M_load
 *                double op_K_eff
 *                double op_I_roll
 *                const double x[14]
 *                const double u[6]
 *                double xd[14]
 * Return Type  : void
 */
void tinympc14_fused_dyn(const double op_A_bal[100], const double op_B_bal[40],
                         const double op_f_bal[10], double op_M_v,
                         double op_M_load, double op_K_eff, double op_I_roll,
                         const double x[14], const double u[6], double xd[14])
{
  double d;
  int i;
  int xd_tmp;
  memset(&xd[0], 0, 14U * sizeof(double));
  for (i = 0; i < 7; i++) {
    xd_tmp = i << 1;
    xd[xd_tmp] = x[xd_tmp + 1];
  }
  for (i = 0; i < 5; i++) {
    d = 0.0;
    for (xd_tmp = 0; xd_tmp < 10; xd_tmp++) {
      d += x[xd_tmp] * op_A_bal[xd_tmp + 10 * ((i << 1) + 1)];
    }
    int i1;
    xd_tmp = (i << 1) + 1;
    i1 = xd_tmp << 2;
    xd[xd_tmp] = (d + (((u[0] * op_B_bal[i1] + u[1] * op_B_bal[i1 + 1]) +
                        u[2] * op_B_bal[i1 + 2]) +
                       u[3] * op_B_bal[i1 + 3])) +
                 op_f_bal[xd_tmp];
  }
  double b_u;
  double b_x;
  double height_arm_idx_0;
  double height_arm_idx_1;
  double load;
  double sec_phi;
  double wheel_force;
  double wheel_force_idx_0;
  double wheel_force_idx_1;
  double x_tmp;
  load = op_M_load * 9.81 + op_K_eff * x[12];
  /*  双轮平地闭链 L_i=(H±b sin(phi))/(cos(phi)cos(psi_i))。 */
  /*  固定两腿矢状角的虚位移 dL_i=height_arm_i*dH+roll_arm_i*dphi。 */
  /*  轴力与 +L 共轭，乘上述两臂分别得到虚升降力和虚横滚力矩。 */
  sec_phi = 1.0 / cos(x[10]);
  height_arm_idx_0 = sec_phi / cos(x[4]);
  height_arm_idx_1 = sec_phi / cos(x[6]);
  x_tmp = tan(x[10]);
  b_x = (x[12] + 0.27) * x_tmp;
  /*  名义竖直承重 load、两腿等分时的几何倾覆矩。其合成效果已由 */
  /*  tinympc14_roll_force 给定，内部仅保留偏离该预载引起的力矩变化。 */
  /*  固定髋部前向位置和腿角，轮心水平位移为 -sin(psi_i)*dL_i。 */
  /*  模型正轮矩与电机轮矩反号，故等价轴力为 +Tw_i*sin(psi_i)/R_w。 */
  wheel_force_idx_0 = u[0] * sin(x[4]) / 0.060000002144914348;
  wheel_force_idx_1 = u[1] * sin(x[6]) / 0.060000002144914348;
  d = height_arm_idx_0 * (b_x + 0.2072000173544678 * sec_phi);
  b_u = u[4] * d;
  wheel_force = wheel_force_idx_0 * d;
  d = height_arm_idx_1 * (b_x + -0.2072000173544678 * sec_phi);
  b_u += u[5] * d;
  wheel_force += wheel_force_idx_1 * d;
  xd[13] = ((((u[4] * height_arm_idx_0 + u[5] * height_arm_idx_1) - load) -
             10.0 * x[13]) +
            (wheel_force_idx_0 * height_arm_idx_0 +
             wheel_force_idx_1 * height_arm_idx_1)) /
           op_M_v;
  xd[11] = 0.0 * x[10] +
           ((b_u - load * (x[12] + 0.27) * x_tmp) + wheel_force) / op_I_roll;
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
  double kB;
  double kB_r;
} struct_T;
#endif /* typedef_struct_T */

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
double tinympc14_fused_op(double sched_H, double sched_lL, double sched_lR,
                          double sched_psi, double op_A_bal[100],
                          double op_B_bal[40], double op_f_bal[10],
                          struct_T *op_aux, double *op_lR, double *op_H,
                          double *op_M_v, double *op_M_load, double *op_K_eff,
                          double *op_I_roll, double *op_psi);

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
 *                double op_A_bal[100]
 *                double op_B_bal[40]
 *                double op_f_bal[10]
 *                struct_T *op_aux
 *                double *op_lR
 *                double *op_H
 *                double *op_M_v
 *                double *op_M_load
 *                double *op_K_eff
 *                double *op_I_roll
 *                double *op_psi
 * Return Type  : double
 */
double tinympc14_fused_op(double sched_H, double sched_lL, double sched_lR,
                          double sched_psi, double op_A_bal[100],
                          double op_B_bal[40], double op_f_bal[10],
                          struct_T *op_aux, double *op_lR, double *op_H,
                          double *op_M_v, double *op_M_load, double *op_K_eff,
                          double *op_I_roll, double *op_psi)
{
  static const double dv2[7] = {0.37115330339491,  0.406993078551576,
                                0.446537222449831, 0.58316671040296231,
                                0.802214836986652, 1.2061115540199536,
                                1.5427950764929956};
  static const double dv3[7] = {0.118052361461424,   0.129267808528255,
                                0.140378508747166,   0.17314375936985016,
                                0.21576900780200958, 0.27813848853111267,
                                0.32100322842597961};
  static const double dv4[5] = {0.129267808528255, 0.140378508747166,
                                0.17314375936985016, 0.21576900780200958,
                                0.27813848853111267};
  static const signed char iv[5] = {0, 0, 1, 0, 0};
  static const signed char iv1[5] = {0, 0, 0, 1, 0};
  static const signed char iv2[5] = {0, 0, 0, 0, 1};
  double M[25];
  double T[25];
  double b_I[25];
  double Nu[20];
  double dv1[20];
  double Nx[15];
  double b_dv[15];
  double C2g;
  double C2g_r;
  double cR_idx_1;
  double d;
  double kB;
  double kB_r;
  double op_lL;
  double rL_idx_0;
  double rL_idx_1;
  double rL_idx_2;
  int a_tmp_tmp;
  int i;
  int k;
  boolean_T x[5];
  /*  方程 (19)-(23) 单点线性化：两腿各按自身腿长取表，10 态 A/B + 仿射 f */
  /*  grav_mode: 'B'  = B 族线性口径（对拍用） */
  /*             'A0' = A 族在 theta=0 线性化（常数 +C2*sin(theta0)） */
  /*             'V*' = A 族在真实平衡点 theta* 线性化（常数 tau_g(0)+余项） */
  /*  分段线性插值 + 端点钳位（与 Leg_Comp.c 同口径） */
  C2g = fmin(fmax(sched_lL, 0.12), 0.32);
  k = 1;
  while ((k < 20) && (C2g >= dv[k << 2])) {
    k++;
  }
  a_tmp_tmp = (k - 1) << 2;
  cR_idx_1 = dv[a_tmp_tmp];
  k <<= 2;
  kB = (C2g - cR_idx_1) / (dv[k] - cR_idx_1);
  cR_idx_1 = dv[a_tmp_tmp + 1];
  rL_idx_0 = cR_idx_1 + kB * (dv[k + 1] - cR_idx_1);
  cR_idx_1 = dv[a_tmp_tmp + 2];
  rL_idx_1 = cR_idx_1 + kB * (dv[k + 2] - cR_idx_1);
  cR_idx_1 = dv[a_tmp_tmp + 3];
  rL_idx_2 = cR_idx_1 + kB * (dv[k + 3] - cR_idx_1);
  /*  分段线性插值 + 端点钳位（与 Leg_Comp.c 同口径） */
  C2g_r = fmin(fmax(sched_lR, 0.12), 0.32);
  k = 1;
  while ((k < 20) && (C2g_r >= dv[k << 2])) {
    k++;
  }
  double rR_idx_0;
  double rR_idx_1;
  double rR_idx_2;
  a_tmp_tmp = (k - 1) << 2;
  cR_idx_1 = dv[a_tmp_tmp];
  k <<= 2;
  kB = (C2g_r - cR_idx_1) / (dv[k] - cR_idx_1);
  cR_idx_1 = dv[a_tmp_tmp + 1];
  rR_idx_0 = cR_idx_1 + kB * (dv[k + 1] - cR_idx_1);
  cR_idx_1 = dv[a_tmp_tmp + 2];
  rR_idx_1 = cR_idx_1 + kB * (dv[k + 2] - cR_idx_1);
  cR_idx_1 = dv[a_tmp_tmp + 3];
  rR_idx_2 = cR_idx_1 + kB * (dv[k + 3] - cR_idx_1);
  /*  分段线性插值 + 端点钳位（与 Leg_Comp.c 同口径） */
  /*  分段线性插值 + 端点钳位（与 Leg_Comp.c 同口径） */
  /*  左腿（打印口径沿用左腿） */
  /*  A 族整机项系数 = (m_b/2 + m_leg)*g*l */
  /*  A 族腿自重项系数 = m_leg*g*l_d */
  kB = (1.3212767058302579 * rL_idx_0 + 10.767124405429584 * sched_lL / 2.0) *
       9.81;
  /*  B 族腿行重力刚度 */
  kB_r = (1.3212767058302579 * rR_idx_0 + 10.767124405429584 * sched_lR / 2.0) *
         9.81;
  /*  tau_g(0) = M_LEG*g*x_off(H) */
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
      -(0.079276605183840845 * rL_idx_0 + 0.64602748742033467 * sched_lL / 2.0);
  M[13] =
      -(0.079276605183840845 * rR_idx_0 + 0.64602748742033467 * sched_lR / 2.0);
  M[10] = -0.027012149543549524;
  M[15] = -0.0043296421992597822;
  M[11] = -0.027012149543549524;
  M[16] = -0.0043296421992597822;
  M[17] = 1.3212767058302579 * rL_idx_0 * -0.034041082972946314;
  M[18] = 1.3212767058302579 * rR_idx_0 * -0.034041082972946314;
  M[19] = -0.033195967438518492;
  M[20] = 0.02858919234796653;
  M[21] = -0.02858919234796653;
  M[22] = 0.18025379964890378 * sched_lL / 0.41440003470893561;
  M[23] = -0.18025379964890378 * sched_lR / 0.41440003470893561;
  memset(&Nx[0], 0, 15U * sizeof(double));
  /*  N_s = -(方程里除 ddq 外的其余项)，故整体取负 */
  Nx[0] = -kB;
  Nx[4] = -kB_r;
  Nx[11] = 3.5956060833522376;
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
  for (i = 0; i < 5; i++) {
    T[i + 10] = iv[i];
    T[i + 15] = iv1[i];
    T[i + 20] = iv2[i];
  }
  memset(&op_A_bal[0], 0, 100U * sizeof(double));
  memset(&b_I[0], 0, 25U * sizeof(double));
  for (k = 0; k < 5; k++) {
    b_I[k + 5 * k] = 1.0;
  }
  for (i = 0; i < 5; i++) {
    for (k = 0; k < 5; k++) {
      op_A_bal[((k << 1) + 10 * (i << 1)) + 1] = b_I[k + 5 * i];
    }
  }
  mldivide(M, Nx, b_dv);
  for (i = 0; i < 3; i++) {
    for (k = 0; k < 5; k++) {
      d = 0.0;
      for (a_tmp_tmp = 0; a_tmp_tmp < 5; a_tmp_tmp++) {
        d += b_dv[i + 3 * a_tmp_tmp] * T[a_tmp_tmp + 5 * k];
      }
      op_A_bal[((i << 1) + 10 * ((k << 1) + 1)) + 4] = d;
    }
  }
  memset(&op_B_bal[0], 0, 40U * sizeof(double));
  b_mldivide(M, Nu, dv1);
  for (i = 0; i < 4; i++) {
    for (k = 0; k < 5; k++) {
      d = 0.0;
      for (a_tmp_tmp = 0; a_tmp_tmp < 5; a_tmp_tmp++) {
        d += dv1[i + (a_tmp_tmp << 2)] * T[a_tmp_tmp + 5 * k];
      }
      op_B_bal[i + (((k << 1) + 1) << 2)] = d;
    }
  }
  memset(&op_f_bal[0], 0, 10U * sizeof(double));
  for (k = 0; k < 5; k++) {
    for (i = 0; i < 5; i++) {
      b_I[i + 5 * k] = M[k + 5 * i];
    }
  }
  int ipiv[5];
  xgetrf(b_I, ipiv);
  op_aux->kB = kB;
  op_aux->kB_r = kB_r;
  op_lL = sched_lL;
  *op_lR = sched_lR;
  *op_H = sched_H;
  /*  sched.H is absolute; only x(13) uses H-H_REF. */
  /*  H 行等效竖直惯量的三档口径（见 cfg §3） */
  *op_M_v = 12.256702386381329;
  /*  Old and training configurations use the wheel-excluded support mass. */
  *op_M_load = 12.503418379724124;
  /*  (dV/dH)/g of the coupled leg mechanism at nominal height */
  /*  气弹簧等效刚度：'spring' 按 H 中心差分 dFs/dL0；'fixed' 取 cfg.K_EFF */
  *op_K_eff = 0.0;
  cR_idx_1 = fmin(fmax(sched_H, 0.118052361461424), 0.32100322842597961);
  for (i = 0; i < 5; i++) {
    d = 0.0;
    for (k = 0; k < 5; k++) {
      d += 0.0 * T[k + 5 * i];
    }
    op_f_bal[(i << 1) + 1] = d;
    x[i] = (cR_idx_1 > dv4[i]);
  }
  k = (((x[0] + x[1]) + x[2]) + x[3]) + x[4];
  *op_I_roll = dv2[k] + (cR_idx_1 - dv3[k]) / (dv3[k + 1] - dv3[k]) *
                            (dv2[k + 1] - dv2[k]);
  *op_psi = sched_psi;
  return op_lL;
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
const double dv[84] = {
    0.12, 0.118075212670147,   0.001924787329853,    0.011926771138911,
    0.13, 0.122662755149485,   0.007337244850515,    0.012069553009353,
    0.14, 0.12761322575744,    0.01238677424256,     0.012213056337296,
    0.15, 0.13285333875533914, 0.017146661244660862, 0.012359332452696783,
    0.16, 0.13832829880805164, 0.021671701191948372, 0.012509732372223394,
    0.17, 0.14399631647046685, 0.026003683529533125, 0.0126651722672807,
    0.18, 0.14982496425048308, 0.0301750357495169,   0.012826288470439227,
    0.19, 0.15578869222869296, 0.034211307771307035, 0.012993531425197446,
    0.2,  0.16186709486557305, 0.038132905134426978, 0.013167224525353453,
    0.21, 0.16804367682905283, 0.04195632317094717,  0.01334760203271736,
    0.22, 0.17430495789531447, 0.045695042104685515, 0.013534834121520508,
    0.23, 0.18063981296543993, 0.049360187034560042, 0.013729043766967505,
    0.24, 0.18703897812580789, 0.052961021874192105, 0.013930318324216987,
    0.25, 0.19349467592850117, 0.056505324071498823, 0.014138717558854704,
    0.26, 0.20000032756592187, 0.059999672434078138, 0.014354279241189453,
    0.27, 0.20655032924602415, 0.063449670753975868, 0.014577023017239376,
    0.28, 0.21313987658811134, 0.066860123411888669, 0.014806953015266805,
    0.29, 0.21976482533409242, 0.070235174665907632, 0.015044059478583982,
    0.3,  0.22642157978723021, 0.07357842021276978,  0.015288319597319432,
    0.31, 0.23310700258609046, 0.076892997413909522, 0.015539697620921479,
    0.32, 0.23981834097748084, 0.080181659022519156, 0.015798144252348545};

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
  struct_T op_aux;
  double op_A_bal[100];
  double op_B_bal[40];
  double length_gain[20];
  double x0[14];
  double op_f_bal[10];
  double ue[6];
  double b_roll_gain[4];
  double cos_psi[2];
  double gravity_stiffness[2];
  double height_gain[2];
  double roll_gain[2];
  double b_expl_temp;
  double c_expl_temp;
  double d;
  double d1;
  double d2;
  double expl_temp;
  double heave_scale;
  double op_I_roll;
  double op_K_eff;
  double op_M_load;
  double op_M_v;
  double residual;
  double roll;
  double sched_H;
  double sched_H_tmp;
  double vertical_tmp;
  int i;
  int i1;
  int k;
  int side;
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
  tinympc14_fused_op(sched_H, left, right, psi, op_A_bal, op_B_bal, op_f_bal,
                     &op_aux, &expl_temp, &b_expl_temp, &op_M_v, &op_M_load,
                     &op_K_eff, &op_I_roll, &c_expl_temp);
  /*  Zero longitudinal and yaw acceleration imply zero wheel torque. */
  /*  The two leg gravity moments and the body pitch moment then fix the */
  /*  common leg lean and hip torques. Vertical and roll balance fix forces. */
  *terminal_center = -0.1797803041676119 / (op_aux.kB + op_aux.kB_r);
  expl_temp = -op_I_roll * 0.0 * roll;
  /*  两腿分担升降广义载荷。在该分担点的几何倾覆矩已归入外扰。 */
  heave_scale = sched_H_tmp * cos(*terminal_center);
  b_expl_temp = heave_scale * sched_H_tmp / 0.2072000173544678;
  memset(&x0[0], 0, 14U * sizeof(double));
  heave_scale *= op_M_load * 9.81 + op_K_eff * (sched_H - 0.27);
  ue[0] = 0.0;
  ue[1] = 0.0;
  x0[4] = *terminal_center;
  ue[2] = -op_aux.kB * *terminal_center;
  ue[4] = (heave_scale + expl_temp * b_expl_temp) / 2.0;
  x0[6] = *terminal_center;
  ue[3] = -op_aux.kB_r * *terminal_center;
  ue[5] = (heave_scale + -expl_temp * b_expl_temp) / 2.0;
  x0[8] = -0.05;
  x0[10] = roll;
  x0[12] = sched_H - 0.27;
  tinympc14_fused_dyn(op_A_bal, op_B_bal, op_f_bal, op_M_v, op_M_load, op_K_eff,
                      op_I_roll, x0, ue, f);
  residual = 0.0;
  for (k = 0; k < 14; k++) {
    heave_scale = fabs(f[k]);
    if (rtIsNaN(heave_scale) || (heave_scale > residual)) {
      residual = heave_scale;
    }
  }
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
  tinympc14_fused_op(sched_H, left, right, psi, op_A_bal, op_B_bal, op_f_bal,
                     &op_aux, &expl_temp, &b_expl_temp, &op_M_v, &op_M_load,
                     &op_K_eff, &op_I_roll, &c_expl_temp);
  memset(&x0[0], 0, 14U * sizeof(double));
  x0[4] = psi;
  x0[6] = psi;
  x0[8] = -0.05;
  x0[10] = roll;
  x0[12] = sched_H - 0.27;
  /*  input expansion point only; never replace the affine residual */
  memset(&A[0], 0, 196U * sizeof(double));
  memset(&B[0], 0, 84U * sizeof(double));
  for (i = 0; i < 10; i++) {
    memcpy(&A[i * 14], &op_A_bal[i * 10], 10U * sizeof(double));
    k = i << 2;
    B[6 * i] = op_B_bal[k];
    B[6 * i + 1] = op_B_bal[k + 1];
    B[6 * i + 2] = op_B_bal[k + 2];
    B[6 * i + 3] = op_B_bal[k + 3];
  }
  double b_sched_H;
  double side_arm_idx_0;
  double side_arm_idx_1;
  double sin_psi_tmp;
  double tan_phi_tmp;
  A[151] = 1.0;
  A[181] = 1.0;
  A[194] = -op_K_eff / op_M_v;
  A[195] = -10.0 / op_M_v;
  /*  双轮平地闭链 L_i=(H±b sin(phi))/(cos(phi)cos(psi_i))。 */
  /*  固定两腿矢状角的虚位移 dL_i=height_arm_i*dH+roll_arm_i*dphi。 */
  /*  轴力与 +L 共轭，乘上述两臂分别得到虚升降力和虚横滚力矩。 */
  heave_scale = 1.0 / sched_H_tmp;
  tan_phi_tmp = tan(roll);
  sin_psi_tmp = sin(psi);
  side_arm_idx_0 = 0.2072000173544678 * heave_scale;
  side_arm_idx_1 = -0.2072000173544678 * heave_scale;
  height_gain[0] = heave_scale / vertical_tmp;
  height_gain[1] = heave_scale / vertical_tmp;
  b_sched_H = sched_H * tan_phi_tmp;
  d = 0.060000002144914348 * op_M_v;
  d1 = 0.060000002144914348 * op_I_roll;
  d2 = op_M_load * 9.81 + op_K_eff * (sched_H - 0.27);
  sched_H_tmp = tan_phi_tmp * tan_phi_tmp;
  heave_scale = sched_H * (sched_H_tmp + heave_scale * heave_scale);
  expl_temp = height_gain[0] * (b_sched_H + side_arm_idx_0);
  roll_gain[0] = expl_temp;
  B[82] = height_gain[0] / op_M_v;
  B[70] = expl_temp / op_I_roll;
  b_expl_temp = ue[4] + 0.0 * sin_psi_tmp / 0.060000002144914348;
  B[78] = height_gain[0] * sin_psi_tmp / d;
  B[66] = expl_temp * sin_psi_tmp / d1;
  c_expl_temp = height_gain[0] * b_expl_temp;
  gravity_stiffness[0] = c_expl_temp;
  roll = tan(psi);
  A[186] =
      height_gain[0] * 0.0 * vertical_tmp / d + c_expl_temp * roll / op_M_v;
  A[158] = expl_temp * 0.0 * vertical_tmp / d1 +
           expl_temp * b_expl_temp * roll / op_I_roll;
  side_arm_idx_0 = height_gain[0] *
                   (heave_scale + 2.0 * side_arm_idx_0 * tan_phi_tmp) *
                   b_expl_temp;
  expl_temp = height_gain[1] * (b_sched_H + side_arm_idx_1);
  roll_gain[1] = expl_temp;
  B[83] = height_gain[1] / op_M_v;
  B[71] = expl_temp / op_I_roll;
  b_expl_temp = ue[5] + 0.0 * sin_psi_tmp / 0.060000002144914348;
  B[79] = height_gain[1] * sin_psi_tmp / d;
  B[67] = expl_temp * sin_psi_tmp / d1;
  c_expl_temp = height_gain[1] * b_expl_temp;
  A[188] =
      height_gain[1] * 0.0 * vertical_tmp / d + c_expl_temp * roll / op_M_v;
  A[160] = expl_temp * 0.0 * vertical_tmp / d1 +
           expl_temp * b_expl_temp * roll / op_I_roll;
  side_arm_idx_1 = height_gain[1] *
                   (heave_scale + 2.0 * side_arm_idx_1 * tan_phi_tmp) *
                   b_expl_temp;
  heave_scale = gravity_stiffness[0] + c_expl_temp;
  A[192] = heave_scale * tan_phi_tmp / op_M_v;
  A[164] =
      ((side_arm_idx_0 + side_arm_idx_1) - d2 * sched_H * (sched_H_tmp + 1.0)) /
      op_I_roll;
  A[166] = ((heave_scale - d2) - op_K_eff * sched_H) * tan_phi_tmp / op_I_roll;
  /*  Vary height and roll at fixed measured sagittal leg angles. Level-ground
   */
  /*  closure gives L_i = (H +/- half_track*sin(roll))/(cos(roll)*cos(psi_i)).
   */
  /*  m, half of the original hardware table spacing */
  cos_psi[0] = fmin(fmax(left, 0.12), 0.32);
  cos_psi[1] = fmin(fmax(right, 0.12), 0.32);
  d = ue[2];
  d1 = ue[3];
  for (side = 0; side < 2; side++) {
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
    gravity_stiffness[side] = fmax(0.12, cos_psi[side] - 0.005);
    sin_psi[side] = fmin(0.32, cos_psi[side] + 0.005);
    tinympc14_bal_block(gravity_stiffness[0], gravity_stiffness[1], A0, B0, f0);
    tinympc14_bal_block(sin_psi[0], sin_psi[1], A1, B1, f1);
    for (i = 0; i < 100; i++) {
      A1[i] -= A0[i];
    }
    for (i = 0; i < 10; i++) {
      d2 = 0.0;
      for (i1 = 0; i1 < 10; i1++) {
        d2 += x0[i1] * A1[i1 + 10 * i];
      }
      b_x0[i] = d2;
    }
    for (i = 0; i < 40; i++) {
      B1[i] -= B0[i];
    }
    heave_scale = sin_psi[side] - gravity_stiffness[side];
    for (i = 0; i < 10; i++) {
      i1 = i << 2;
      length_gain[side + (i << 1)] =
          (((b_x0[i] + (((0.0 * B1[i1] + 0.0 * B1[i1 + 1]) + d * B1[i1 + 2]) +
                        d1 * B1[i1 + 3])) +
            f1[i]) -
           f0[i]) /
          heave_scale;
    }
    k = side << 1;
    b_roll_gain[k] = roll_gain[side];
    b_roll_gain[k + 1] = height_gain[side];
  }
  for (i = 0; i < 2; i++) {
    d = b_roll_gain[i];
    d1 = b_roll_gain[i + 2];
    for (i1 = 0; i1 < 10; i1++) {
      k = i1 << 1;
      side = ((i << 1) + 14 * i1) + 10;
      A[side] += d * length_gain[k] + d1 * length_gain[k + 1];
    }
  }
  tinympc14_fused_dyn(op_A_bal, op_B_bal, op_f_bal, op_M_v, op_M_load, op_K_eff,
                      op_I_roll, x0, ue, f);
  for (i = 0; i < 14; i++) {
    d = 0.0;
    for (i1 = 0; i1 < 14; i1++) {
      d += x0[i1] * A[i1 + 14 * i];
    }
    d1 = 0.0;
    for (i1 = 0; i1 < 6; i1++) {
      d1 += ue[i1] * B[i1 + 6 * i];
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
