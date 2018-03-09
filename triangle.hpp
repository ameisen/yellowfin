#pragma once

namespace motion
{
  template <motion_type MT>
  class triangle;

  template<> class triangle<motion_type::floating_point> final
  {
  public:
    // Movement Data

    step_t distance_;
    tick_t time_;

    float start_velocity_;
    float end_velocity_;
    float peak_velocity_;

    float post_duration_inv_;
    tick_t peak_time_;
    float peak_time_inv_;

    // Execution Data
    uint32 current_tick_ = 0;
    float prev_steps_ = 0.0f;

    triangle() = default;

#if 0
    triangle(const data<motion_type::floating_point> &__restrict _data)
    {
      distance_ = _data.distance;
      const float distance = _data.distance;
      const float start_velocity = _data.start_velocity;
      const float end_velocity = _data.end_velocity;
      const float time = (distance / _data.velocity) * frequency;

      float mean_velocity = (distance / time);

      float peak_velocity = mean_velocity;
      float peak_fraction = 0.5f;

      // integrate
      static constexpr const uint32 num_integrations = 5;
      for (uint32 integration = 0; integration < num_integrations; ++integration)
      {
        // velocity should still be double the mean velocity, I believe?

        float accel_delta = fabsf(peak_velocity - start_velocity);
        float decel_delta = fabsf(peak_velocity - end_velocity);

        float accel_time = (peak_fraction * time);
        float decel_time = ((1.0f - peak_fraction) * time);

        float acceleration = (accel_delta / accel_time);
        float deceleration = (decel_delta / decel_time);

        float calc_accel_steps = (start_velocity * accel_time) + 0.5f * (acceleration * (accel_time * accel_time));
        float calc_decel_component = 0.5f * (-deceleration * (decel_time * decel_time));
        float calc_decel_steps = (peak_velocity * decel_time) + calc_decel_component;
        float calc_total_steps = calc_accel_steps + calc_decel_steps;
        float ratio = distance / calc_total_steps;

        peak_velocity = peak_velocity * ratio;
        peak_fraction = 0.5f;
        if (start_velocity || end_velocity)
        {
          float start_diff = fabs(lround(peak_velocity) - start_velocity);
          float end_diff = fabs(lround(peak_velocity) - end_velocity);
          float total_velocity = start_diff + end_diff;
          float start_fraction = float(start_diff) / float(total_velocity);

          peak_fraction = (0.5f + start_fraction) * 0.5f; // This specifies how far into the motion's time the top of the triangle is.
        }
      }

      time_ = lround(time);

      peak_velocity_ = peak_velocity;
      peak_time_ = lround(peak_fraction * time);
      peak_time_inv_ = 1.0f / peak_time_;
      post_duration_inv_ = 1.0f / (time_ - peak_time_);

      start_velocity_ = start_velocity;
      end_velocity_ = end_velocity;
    }
#endif

    triangle(const data<motion_type::floating_point> &__restrict _data)
    {
      distance_ = _data.distance;
      const float distance = _data.distance;
      const float start_velocity = _data.start_velocity;
      const float end_velocity = _data.end_velocity;
      const float time = (distance / _data.velocity) * frequency;

      float mean_velocity = (distance / time);

      float peak_velocity = mean_velocity;
      float peak_fraction = 0.5f;

      // integrate
      static constexpr const uint32 num_integrations = 5;
      for (uint32 integration = 0; integration < num_integrations; ++integration)
      {
        // velocity should still be double the mean velocity, I believe?

        float accel_delta = fabsf(peak_velocity - start_velocity);
        float decel_delta = fabsf(peak_velocity - end_velocity);

        float accel_time = (peak_fraction * time);
        float decel_time = ((1.0f - peak_fraction) * time);

        float acceleration = (accel_delta / accel_time);
        float deceleration = (decel_delta / decel_time);

        float calc_accel_steps = (start_velocity * accel_time) + 0.5f * (acceleration * (accel_time * accel_time));
        float calc_decel_component = 0.5f * (-deceleration * (decel_time * decel_time));
        float calc_decel_steps = (peak_velocity * decel_time) + calc_decel_component;
        float calc_total_steps = calc_accel_steps + calc_decel_steps;
        float ratio = distance / calc_total_steps;

        //peak_velocity = peak_velocity * ratio;
        peak_fraction = 0.5f;
        if (start_velocity || end_velocity)
        {
          float start_diff = fabs(lround(peak_velocity) - start_velocity);
          float end_diff = fabs(lround(peak_velocity) - end_velocity);
          float total_velocity = start_diff + end_diff;
          float start_fraction = float(start_diff) / float(total_velocity);

          peak_fraction = (0.5f + start_fraction) * 0.5f; // This specifies how far into the motion's time the top of the triangle is.
        }
      }

      time_ = lround(time);

      peak_velocity_ = peak_velocity;
      peak_time_ = lround(peak_fraction * time);
      peak_time_inv_ = 1.0f / peak_time_;
      post_duration_inv_ = 1.0f / (time_ - peak_time_);

      start_velocity_ = start_velocity;
      end_velocity_ = end_velocity;
    }

    uint32 get_current_tick() const __restrict
    {
      return current_tick_;
    }

    template <uint32 iterations = 2>
    static float get_sine(float fract)
    {
      enum class move_type : uint8
      {
        sine = 0,
        tight_sine,
        constant
      };
      static constexpr const move_type TYPE = move_type::sine;

      __assume(fract >= 0.0f && fract <= 1.0f);

      if constexpr(TYPE == move_type::sine)
      {
        return sine_func(fract);
      }
      else if constexpr(TYPE == move_type::tight_sine)
      {
        float val = sine_func(fract);
        __assume(val >= 0.0f && val <= 1.0f);
        for (uint32 i = 0; i < iterations; ++i)
        {
          val = sine_func(val);
          __assume(val >= 0.0f && val <= 1.0f);
        }

        val = pow(val, float((iterations + 1) - iterations) / float(iterations + 1));
        __assume(val >= 0.0f && val <= 1.0f);

        return val;
      }
      else
      {
        return fract;
      }
    }

    static constexpr const uint32 tick_time = 200;

    uint32 current_step = 0;

    bool pulse_time() __restrict
    {
      __assume(current_tick_ >= 0.0f);

      if (__likely(current_tick_ < peak_time_))
      {
        __assume(peak_time_inv_ <= 1.0f);
        float time_fract = float(current_tick_) * peak_time_inv_;
        __assume(time_fract >= 0.0f && time_fract <= 1.0f);
        float sine = get_sine(time_fract);
        __assume(sine >= 0.0f && sine <= 1.0f);
        __assume(start_velocity_ >= 0.0f);
        __assume(peak_velocity_ > 0.0f);
        float sine_velocity = lerp(start_velocity_, peak_velocity_, sine);
        __assume(sine_velocity >= start_velocity_ && sine_velocity <= peak_velocity_);

        float steps = prev_steps_ + (sine_velocity * tick_time);
        __assume(steps >= prev_steps_);
        __assume(steps >= 0.0f && steps <= distance_);

        prev_steps_ = steps;
        bool result = __unlikely(round<uint32>(steps) > current_step);

        if (__unlikely(result))
        {
          ++current_step;
        }

        current_tick_ += tick_time;
        return result;
      }
      else if (__likely(current_tick_ < time_))
      {
        __assume(post_duration_inv_ <= 1.0f);
        __assume(current_tick_ >= peak_time_);
        float time_fract = float(current_tick_ - peak_time_) * post_duration_inv_;
        __assume(time_fract >= 0.0f && time_fract <= 1.0f);
        float sine = sine_func(time_fract);
        __assume(sine >= 0.0f && sine <= 1.0f);
        __assume(end_velocity_ >= 0.0f);
        __assume(peak_velocity_ > 0.0f);
        float sine_velocity = lerp(peak_velocity_, end_velocity_, sine);
        __assume(sine_velocity >= start_velocity_ && sine_velocity <= peak_velocity_);

        float steps = prev_steps_ + (sine_velocity * tick_time);
        __assume(steps >= prev_steps_);
        __assume(steps >= 0.0f && steps <= distance_);

        prev_steps_ = steps;
        bool result = __unlikely(round<uint32>(steps) > current_step);

        if (__unlikely(result))
        {
          ++current_step;
        }

        current_tick_ += tick_time;
        return result;
      }
      else
      {
        __assume(end_velocity_ >= 0.0f);
        float steps = prev_steps_ + (end_velocity_ * tick_time);
        __assume(steps >= prev_steps_);
        __assume(steps >= 0.0f && steps <= distance_);
        prev_steps_ = steps;
      
        bool result = __unlikely(round<uint32>(steps) > current_step);
        if (__unlikely(result))
        {
          ++current_step;
        }

        current_tick_ += tick_time;
        return result;
      }
    }
  };
}
