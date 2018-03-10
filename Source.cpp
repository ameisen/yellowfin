// We neee to communicate over pin 0 (rx) and pin 1 (tx)

//#include "cortex.hpp"
#include "Teensy/Arduino.h"
#include "Teensy/kinetis.h"
#include "defines.hpp"
#include "movement.h"
#include "motion.hpp"
#include "pins.hpp"

#include <new>

static constexpr const auto MT = motion::motion_type::floating_point;

template <bool state>
void set_led()
{
  set_pin<pin::LED, state>();
}

// PIT timers run at F_BUS.
// By default, this is 60 MHz

constexpr uint32_t seconds_to_ticks(double seconds)
{
  return uint32_t((uint32_t(F_BUS) * seconds) + 0.5);
}

static motion::data<MT> motion_data;
static motion::trapezoid<MT> _motion;

static constexpr const uint32 pit0_idx = 64;

void pit0_isr2();

template <bool state>
void toggle_pit0_isr()
{
  if constexpr (state)
  {
    PIT_TCTRL0 |= PIT_TCTRL_TIE;
  }
  else
  {
    PIT_TCTRL0 &= ~PIT_TCTRL_TIE;
  }
}

template <bool state>
void toggle_pit0_timer()
{
  if constexpr (state)
  {
    PIT_TCTRL0 |= PIT_TCTRL_TEN;
  }
  else
  {
    PIT_TCTRL0 &= ~PIT_TCTRL_TEN;
  }
}

template <bool state>
void toggle_pit0()
{
  if constexpr (state)
  {
    PIT_TCTRL0 = PIT_TCTRL_TIE | PIT_TCTRL_TEN;
  }
  else
  {
    PIT_TCTRL0 = 0;
  }
}

uint32 get_pit0_timer()
{
  return PIT_CVAL0;
}

bool stepper_pin_state = false;
static constexpr const uint32 pulse_width = 50;
volatile uint32 total_steps = 0;

void set_pit0_next(uint32 ticks)
{
  PIT_LDVAL0 = ticks;
}

extern "C" void checkpulse_isr();
void endpulse_isr();

template <bool state>
void toggle_pulse_isr()
{
  if constexpr(state)
  {
    set_pin<pin::STEP_PULSE, false>();
    PIT_LDVAL0 = 1;
    toggle_pit0<true>();
    PIT_TFLG0 = 1;
    stepper_pin_state = false;
    _VectorsRam[pit0_idx] = &checkpulse_isr;
  }
  else
  {
    set_pin<pin::STEP_PULSE, false>();
    toggle_pit0<false>();
    PIT_LDVAL0 = 1;
    stepper_pin_state = false;
    _VectorsRam[pit0_idx] = &checkpulse_isr;
  }
}

float acceleration = mm_in_steps(2000.0f) / (frequency * frequency);

void set_new_acceleration(float _acceleration)
{
  acceleration = mm_in_steps(_acceleration) / (frequency * frequency);
}

void set_new_motion(float distance, float velocity)
{
  total_steps = 0;

  PIT_CVAL0 = 0;

  motion_data.distance = mm_in_steps(fabsf(distance));
  motion_data.velocity = mm_in_steps(velocity) / frequency;
  motion_data.start_velocity = mm_in_steps(1.0f) / frequency;
  motion_data.end_velocity = mm_in_steps(1.0f) / frequency;
  motion_data.max_acceleration = acceleration;
  
  if (distance < 0)
  {
    // We are moving backwards.
    set_pin<pin::STEP_DIRECTION, false>();
  }
  else
  {
    set_pin<pin::STEP_DIRECTION, true>();
  }
  
  new (&_motion) motion::trapezoid<MT>(motion_data);

  toggle_pulse_isr<true>();
}

extern "C" void checkpulse_isr()
{
  if (total_steps == motion_data.distance)
  {
    // Disable ISR.
    toggle_pulse_isr<false>();
    return;
  }

  bool pulse = _motion.pulse_time();
  if (__unlikely(pulse))
  {
    stepper_pin_state = true;

    set_pin<pin::STEP_PULSE, true>();

    _VectorsRam[pit0_idx] = &endpulse_isr;
    set_pit0_next(pulse_width);
    set_led<true>();
  }
  else
  {
    set_pit0_next(_motion.tick_time * frequency_scale);
  }

  PIT_TFLG0 = 1;
}

void endpulse_isr()
{
  stepper_pin_state = false;

  set_pin<pin::STEP_PULSE, false>();

  _VectorsRam[pit0_idx] = &checkpulse_isr;
  set_pit0_next((_motion.tick_time * frequency_scale) - pulse_width);
  set_led<false>();

  ++total_steps;

  PIT_TFLG0 = 1;
}

extern "C" void pit0_isr(void)		__attribute__((alias("checkpulse_isr")));

extern "C"
__attribute__((used, noreturn))
void lmain()
{
  enable_output_pin<pin::LED>();
  enable_output_pin<pin::STEP_DIRECTION>();
  enable_output_pin<pin::STEP_ENABLE>();
  enable_output_pin<pin::STEP_PULSE>();

  set_pin<pin::STEP_ENABLE, false>();

  // Set up PIT0
  SIM_SCGC6 |= SIM_SCGC6_PIT;
  PIT_MCR = 0x00;
  NVIC_ENABLE_IRQ(IRQ_PIT_CH0);
  PIT_LDVAL0 = 1;
  //PIT_TCTRL0 = PIT_TCTRL_TIE | PIT_TCTRL_TEN;

  //v = (uint32_t)mcg_clk_hz;
  //v = v / 1000000;

  bool state = false;
  constexpr const uint32_t pause_cap = 240'000;
  uint32_t pause = pause_cap;

  Serial.begin(230'400);
  
  // movement state machine
  bool speed = false;
  bool abort = false;
  bool acceleration = false;
  char buffer[64];
  size_t buffer_idx = 0;
  float distance = 0.0f;
  float velocity = 0.0f;
  bool first_char = true;
  for (;;)
  {
    if (Serial.dtr())
    { 
      static bool once = true;
      if (once)
      {
        Serial.println("Teensy Online");
        Serial.send_now();
        once = false;
      }

      if (Serial.available() > 0)
      {
        int incoming = Serial.read();
        if (incoming != -1)
        {
          if (first_char)
          {
            if (incoming == 'A')
            {
              acceleration = true;
              goto next;
            }
            first_char = false;
          }

          if (acceleration)
          {
            if (incoming == '\n')
            {
              buffer[buffer_idx] = '\0';
              float accel = strtof(buffer, nullptr);
              buffer_idx = 0;

              char tbuffer[256];

              set_new_acceleration(accel);
              sprintf(tbuffer, "Acceleration %fmm/ss/s", accel);
              Serial.println(tbuffer);

              Serial.send_now();

              acceleration = false;
              first_char = true;
            }
            else
            {
              buffer[buffer_idx++] = incoming;
            }
          }
          else
          {
            if (incoming == ' ')
            {
              buffer[buffer_idx] = '\0';
              distance = strtof(buffer, nullptr);
              buffer_idx = 0;
              speed = true;
            }
            else if (incoming == '\n')
            {
              buffer[buffer_idx] = '\0';
              velocity = strtof(buffer, nullptr);
              buffer_idx = 0;
              speed = false;

              char tbuffer[256];
              if (abort)
              {
                toggle_pulse_isr<false>();
                Serial.println("Aborting Move");
              }
              else
              {
                set_new_motion(distance, velocity);
                sprintf(tbuffer, "Distance %fmm at Velocity %fmm/s", distance, velocity);
                Serial.println(tbuffer);
              }
              Serial.send_now();

              distance = 0.0f;
              velocity = 0.0f;
              abort = false;
              first_char = true;
            }
            else if (incoming == 'a')
            {
              abort = true;
            }
            else
            {
              buffer[buffer_idx++] = incoming;
            }
          }
        next:;
        }
      }
    }
  }

  /*
  while (1)
  {
    for (n = 0; n<1000000; n++);	// dumb delay
    mask = 0x80;
    while (mask != 0)
    {
      LED_ON;
      for (n = 0; n<1000; n++);		// base delay
      if ((v & mask) == 0)  LED_OFF;	// for 0 bit, all done
      for (n = 0; n<2000; n++);		// (for 1 bit, LED is still on)
      LED_OFF;
      for (n = 0; n<1000; n++);
      mask = mask >> 1;
    }
  }*/
}
