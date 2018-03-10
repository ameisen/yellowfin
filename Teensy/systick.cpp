#include "core_pins.h"

extern "C" volatile uint32_t systick_millis_count;
void systick_isr(void)
{
  ++systick_millis_count;
}
