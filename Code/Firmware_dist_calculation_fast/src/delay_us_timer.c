#include "stdint.h"
#include "stm32f10x.h"
#include "delay_us_timer.h"

void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT       = 0;                            // reset
    DWT->CTRL        |= 1;                            // enable the counter
}

static inline uint32_t dwt_dt(uint32_t t0, uint32_t t1)
{
    return (t1 - t0);
}

void dwt_delay(uint32_t us) // microseconds (max = 2^32 / (fcpu*1.0E-6) - 1)
{
    uint32_t t0 = DWT->CYCCNT;
    uint32_t dt = us * (F_CPU/1000000UL);

    while (dwt_dt(t0, DWT->CYCCNT) < dt)  {;} //(DWT->CYCCNT - t0) < dt
}

uint32_t get_dwt_value(void)
{
  return DWT->CYCCNT;
}

