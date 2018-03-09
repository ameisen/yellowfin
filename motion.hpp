#pragma once

#include "Teensy/Arduino.h"

using tick_t = uint32;
using mtime_t = float;
using step_t = uint32; // could be uint16 for us.
using velocity_t = int32; // steps/sec.
using accel_t = uint32; // steps/sec.

static constexpr const float frequency_scale = 60.0f;
static constexpr const float frequency = 60'000'000.0f / frequency_scale;

static constexpr const float MCA4 = 70.82001273823335;
static constexpr const float MCA3 = -40.01531643260777;
static constexpr const float MCA2 = -19.84299479567418;
static constexpr const float MCA1 = float(2.0 * 3.14159265358979323846264338327950288);
static constexpr const float MCA0 = 1.0;

constexpr inline __forceinline __flatten float _sq(float t)
{
  return t * t;
}

constexpr inline __forceinline __flatten float _fcos(float t)
{
  // returns cos(2*pi*t) for -0.125 <= t <= 0.125
  float t2 = _sq(t);
  return (MCA4 * t2 + MCA2) * t2 + MCA0;
}

constexpr inline __forceinline __flatten float _fsin(float t)
{
  // returns sin(2*pi*t) for -0.125 <= t <= 0.125
  float t2 = _sq(t);
  return (MCA3 * t2 + MCA1) * t;
}

constexpr inline __forceinline __flatten float fcos(float t)
{
  //t = t - floor(t); // get t in the range 0 <= t < 1	
  __assume(t >= 0.0f && t < 1.0f);
  if (__unlikely(t == 1.0f))
  {
    return _fcos(0.0f);
  }
  else if (t > 0.875f)
  {
    return _fcos(t - 1.0f);
  }
  else if (t > 0.625f)
  {
    return _fsin(t - 0.75f);
  }
  else if (t > 0.375f)
  {
    return -_fcos(t - 0.5f);
  }
  else if (t > 0.125f)
  {
    return -_fsin(t - 0.25f);
  }
  else
  {
    return _fcos(t);
  }
}

constexpr inline __forceinline __flatten float fsin(float t)
{
  //t = t - floor(t); // get t in the range 0 <= t < 1	
  __assume(t >= 0.0f && t < 1.0f);
  if (__unlikely(t == 1.0f))
  {
    return _fcos(0.0f);
  }
  else if (t > 0.875f)
  {
    return _fsin(t - 1.0f);
  }
  else if (t > 0.625f) 
  {
    return -_fcos(t - 0.75f);
  }
  else if (t > 0.375f)
  {
    return -_fsin(t - 0.5f);
  }
  else if (t > 0.125f)
  {
    return _fcos(t - 0.25f);
  }
  else
  {
    return _fsin(t);
  }
}

float sine_func(float v)
{
  // orig size: 10056
  // orig assume size: 10128
  // new size: 11404
  // new assume size: 11408

#if 0
  static constexpr const float pi = 3.1415926535897932384626433832795028841971693993751058209749445923078164062f;
  __assume(v >= 0.0f && v <= 1.0f);
  v = sin((v * pi) + (1.5f * pi));
  __assume(v >= -1.0f && v <= 1.0f);
  return (v + 1.0f) * 0.5f;
#else
  __assume(v >= 0.0f && v <= 1.0f);
  v *= 0.5f;
  __assume(v >= 0.0f && v <= 0.5f);
  v += 0.75f;
  __assume(v >= 0.75f && v <= 1.25f);
  v = fsin(v);
  __assume(v >= -1.0f && v <= 1.0f);
  v = (v + 1.0f) * 0.5f;
  __assume(v >= 0.0f && v <= 1.0f);
  return v;
#endif
};

float frac(float val)
{
  // 49964	   2656	   2668	  55288
  //double i;
  //return modf(val, &i);
  return val - floorf(val);
};

float lerp(float x, float y, float s)
{
  return x + s * (y - x);
}

namespace motion
{
  enum class motion_type : uint8
  {
    floating_point = 0,
    fixed_point = 1,
  };

  template <motion_type MT = motion_type::floating_point>
  struct data final
  {
    step_t distance;
    float velocity;

    float start_velocity;
    float end_velocity;

    float max_acceleration = 0;

    bool force_trapezoid = false;
  };
}

#include "triangle.hpp"
#include "trapezoid.hpp"
