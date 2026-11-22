#ifndef TEST_DWT_H
#define TEST_DWT_H
#include <stdint.h>
typedef struct
{
    volatile uint32_t CYCCNT;
} test_dwt_t;
extern test_dwt_t test_dwt;
extern uint32_t SystemCoreClock;
#define DWT (&test_dwt)
#endif
