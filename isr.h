#pragma once

static constexpr const uint32 ref_rate = 60'000'000;

constexpr uint32 operator "" _timer_us(unsigned long long value)
{
  return (value) * (ref_rate / 1'000'000);
}

constexpr uint32 operator "" _timer_ns(unsigned long long value)
{
  return uint32((double((value) * (double(ref_rate) / 1'000'000'000.0))) + 0.5);
}

static constexpr const uint32 pulse_width = 3_timer_us;

constexpr uint32 operator "" _MHz(unsigned long long value)
{
  return ref_rate / (value * 1'000'000);
}

constexpr uint32 operator "" _KHz(unsigned long long value)
{
  return ref_rate / (value * 1'000);
}

constexpr uint32 operator "" _Hz(unsigned long long value)
{
  return ref_rate / (value);
}

constexpr static uint32 to_isr_timer(uint32 value)
{
  __assume(value <= ref_rate);
  if (value == 0)
  {
    value = 1;
  }
  return uint32(uint32(ref_rate) / value);
}

static uint32 to_isr_timer(float value)
{
  __assume(value <= ref_rate);
  __assume(value >= 0.0f);
  if (value < 1.0f)
  {
    value = 1.0f;
  }
  return round<uint32>(float(ref_rate) / value);
}

static uint32 to_timer_from_sec(float value)
{
  __assume(value >= 0.0f);
  return round<uint32>(value * ref_rate);
}
