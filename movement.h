#pragma once

static constexpr const uint32 microsteps = 16;
static constexpr const uint32 steps_per_rev = 200;
static constexpr const float mm_per_rev = 12.7;//4;
static constexpr const float steps_per_mm = float(uint32(steps_per_rev) * microsteps) / mm_per_rev;

constexpr int32 mm_in_steps(float mm)
{
  if (mm < 0)
  {
    return (mm * steps_per_mm) - 0.5f;
  }
  else
  {
    return (mm * steps_per_mm) + 0.5f;
  }
}

enum class direction : bool
{
  forth = true,
  back = false
};
