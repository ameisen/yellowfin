#include "core_pins.h"

// TODO consider changing this to a uint64_t.
extern "C" volatile uint32_t systick_millis_count;
__attribute__((interrupt)) void systick_isr(void)
{
  ++systick_millis_count;
}
