#pragma once

#include <stdint.h>
#include <math.h>

#undef round
#undef __unreachable

using uint8 = uint8_t;
using uint16 = uint16_t;
using uint32 = uint32_t;
using uint64 = uint64_t;

using int8 = int8_t;
using int16 = int16_t;
using int32 = int32_t;
using int64 = int64_t;

#if __INTELLISENSE__
# define __unreachable
# define __forceinline
# define __assume(c)
# define __likely(c) (c)
# define __unlikely(c) (c)
# define __flatten
#else
# define __unreachable __builtin_unreachable()
# define __forceinline __attribute__((always_inline))
# define __assume(c) { if (!(c)) { __unreachable; } }
# define __likely(c) (__builtin_expect(c, true))
# define __unlikely(c) (__builtin_expect(c, false))
# define __flatten __attribute__((flatten))
#endif

namespace Tuna
{

  struct ce_only { ce_only() = delete; };

  template <typename T, T v>
  struct integral_constant : ce_only
  {
    static constexpr const T value = v;
  };
  struct false_type : integral_constant<bool, false> {};
  struct true_type : integral_constant<bool, true> {};

  template <typename T, typename U>
  struct _is_same final : false_type {};
  template <typename T>
  struct _is_same<T, T> final : true_type {};
  template <typename T, typename U> constexpr const bool is_same = _is_same<T, U>::value;

  inline float round(float x)
  {
    return roundf(x);
  }

  template <typename T>
  constexpr inline __forceinline __flatten T round(float value)
  {
    if constexpr (is_same<T, float> || is_same<T, double>)
    {
      // We are doing float->float rounding.
      return ::roundf(value);
    }
    else// if constexpr(type_trait<T>::is_unsigned)
    {
      // We are doing float->unsigned rounding.
      //__assume(T(value) >= 0 && T(value + 0.5f) <= type_trait<T>::max);
      //__assume(value >= 0.0 && value <= type_trait<T>::max);
      const auto result = lround(value);
      //__assume(result >= 0 && result <= type_trait<T>::max);
      return result;
    }
    /*
    else
    {
      // We are doing float->signed rounding.
      __assume(T(value) >= type_trait<T>::min && T(value + 0.5f) <= type_trait<T>::max);
      __assume(value >= type_trait<T>::min && value <= type_trait<T>::max);
      const auto result = lround(value);
      __assume(result >= type_trait<T>::min && result <= type_trait<T>::max);
      return result;
    }
    */
  }
}
