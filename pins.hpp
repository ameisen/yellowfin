#pragma once

// https://www.pjrc.com/teensy/schematic.html

enum class pin : uint8
{
  LED = 13,
  STEP_ENABLE = 35,
  STEP_DIRECTION = 34,
  STEP_PULSE = 33,
};

struct port_def final
{
  volatile uint32 & __restrict psor;
  volatile uint32 & __restrict pcor;
  volatile uint32 (& __restrict pcr)[32];
  volatile uint32 & __restrict pddr;
};

static constexpr const port_def port_a{
  GPIOA_PSOR,
  GPIOA_PCOR,
  (volatile uint32(&)[32])PORTA_PCR0,
  GPIOA_PDDR,
};
static constexpr const port_def port_b{
  GPIOB_PSOR,
  GPIOB_PCOR,
  (volatile uint32(&)[32])PORTB_PCR0,
  GPIOB_PDDR,
};
static constexpr const port_def port_c{
  GPIOC_PSOR,
  GPIOC_PCOR,
  (volatile uint32(&)[32])PORTC_PCR0,
  GPIOC_PDDR,
};
static constexpr const port_def port_d{
  GPIOD_PSOR,
  GPIOD_PCOR,
  (volatile uint32(&)[32])PORTD_PCR0,
  GPIOD_PDDR,
};
static constexpr const port_def port_e{
  GPIOE_PSOR,
  GPIOE_PCOR,
  (volatile uint32(&)[32])PORTE_PCR0,
  GPIOE_PDDR,
};

struct pin_def final
{
  const port_def port_;
  const uint32 bit_;

  constexpr pin_def(const port_def &port, uint32 bit) : port_(port), bit_(bit) {}
};

constexpr const pin_def dummy = { port_a, 0 };

static constexpr const pin_def pin_defs[] = {
  dummy,
  dummy,
  dummy,
  dummy,
  dummy,
  dummy,
  dummy,
  dummy,
  dummy,
  dummy,
  dummy,
  dummy,
  dummy,
  { port_c, 5 },
  dummy,
  dummy,
  dummy,
  dummy,
  dummy,
  dummy,
  dummy,
  dummy,
  dummy,
  dummy,
  dummy,
  dummy,
  dummy,
  dummy,
  dummy,
  dummy,
  dummy,
  dummy,
  dummy,
  { port_e, 8 },
  { port_e, 24 },
  { port_c, 25 },
};

template <pin pin, bool state>
void set_pin()
{
  uint32 bit;
  volatile uint32 * __restrict reg;

  switch (pin)
  {
  case pin::LED:
    reg = state ? &GPIOC_PSOR : &GPIOC_PCOR;
    bit = 5;
    break;
  case pin::STEP_ENABLE:
    reg = state ? &GPIOC_PSOR : &GPIOC_PCOR;
    bit = 8;
    break;
  case pin::STEP_DIRECTION:
    reg = state ? &GPIOE_PSOR : &GPIOE_PCOR;
    bit = 25;
    break;
  case pin::STEP_PULSE:
    reg = state ? &GPIOE_PSOR : &GPIOE_PCOR;
    bit = 24;
    break;
  }

  *reg = (1 << bit);
}

template <pin pin>
void enable_output_pin()
{
  uint32 bit;
  volatile uint32 * __restrict pcr;
  volatile uint32 * __restrict pddr;

  switch (pin)
  {
  case pin::LED:
    pcr = &PORTC_PCR5;
    pddr = &GPIOC_PDDR;
    bit = 5;
    break;
  case pin::STEP_ENABLE:
    pcr = &PORTC_PCR8;
    pddr = &GPIOC_PDDR;
    bit = 8;
    break;
  case pin::STEP_DIRECTION:
    pcr = &PORTE_PCR25;
    pddr = &GPIOE_PDDR;
    bit = 25;
    break;
  case pin::STEP_PULSE:
    pcr = &PORTE_PCR24;
    pddr = &GPIOE_PDDR;
    bit = 24;
    break;
  }

  *pcr = PORT_PCR_MUX(0x1);
  *pddr |= (1 << bit);
}
