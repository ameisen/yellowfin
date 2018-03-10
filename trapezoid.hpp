#pragma once

#include <algorithm>

namespace motion
{
  template <motion_type MT>
  class trapezoid;

  template<> class trapezoid<motion_type::floating_point> final
  {
  public:
    // Movement Data

    step_t distance_;
    tick_t time_;

    float start_velocity_;
    float end_velocity_;
    float plateau_velocity_;

    tick_t up_time_;
    float up_time_inv_;
    tick_t plateau_time_;
    float plateau_time_inv_;
    tick_t down_time_;
    float down_time_inv_;

    // Execution Data
    uint32 current_tick_ = 0;
    float prev_steps_ = 0.0f;

    trapezoid() = default;

    trapezoid(const data<motion_type::floating_point> &__restrict _data)
    {
      distance_ = _data.distance;
      const float distance = _data.distance;
      const float start_velocity = _data.start_velocity;
      const float end_velocity = _data.end_velocity;
      float target_velocity = _data.velocity;
      const float acceleration = _data.max_acceleration;

      const auto dump_var = [](const char *name, float value)
      {
        char tbuffer[256];
        sprintf(tbuffer, "%s %f", name, value);
        Serial.println(tbuffer);
        Serial.send_now();
      };

      float velocity_diffs[2] = {
        target_velocity - start_velocity,
        end_velocity - target_velocity
      };

      float ramp_times[2] = {
        abs(velocity_diffs[0]) / acceleration,
        abs(velocity_diffs[1]) / acceleration
      };

      float ramp_accelerations[2] = {
        (velocity_diffs[0] > 0.0f) ? acceleration : -acceleration,
        (velocity_diffs[1] > 0.0f) ? acceleration : -acceleration,
      };

      const auto square = [](float value)
      {
        return value * value;
      };

      float ramp_distances[2] = {
        abs((start_velocity * ramp_times[0]) + (0.5f * ramp_accelerations[0] * square(ramp_times[0]))),
        abs((target_velocity * ramp_times[1]) + (0.5f * ramp_accelerations[1] * square(ramp_times[1])))
      };

      float total_ramp_distance = ramp_distances[0] + ramp_distances[1];

      float total_time = 0.0;
      float plateau_time = 0.0;

      if (total_ramp_distance < distance)
      {
        // Trapezoid
        const float remaining_distance = distance - total_ramp_distance;
        const float start_pos = ramp_distances[0];
        // (s - s0) / v0 = t
        plateau_time = remaining_distance / target_velocity;

        total_time = ramp_times[0] + ramp_times[1] + plateau_time;
      }
      else if (total_ramp_distance == distance)
      {
        // Unlikely perfect triangle
        total_time = ramp_times[0] + ramp_times[1];
      }
      else
      {
        // Triangle
        if (velocity_diffs[0] == 0.0f)
        {
          // Down Ramp Only
          // s = v0t + 1/2at^2
          // v0 = b
          // t = (-sqrt(2as+a0^2) + a0) / a
          // t = (sqrt(2as+a0^2) - a0) / a
          // whichever is greater.
          
          const float in_value = (2.0f * ramp_accelerations[1] * distance) + square(start_velocity);
          __assume(in_value >= 0.0f);
          const float expression = sqrtf(in_value);
          const float time = std::max(
            -((expression + start_velocity) / ramp_accelerations[1]),
             ((expression - start_velocity) / ramp_accelerations[1])
          );
          ramp_times[1] = time;
          total_time = time;
        }
        else if (velocity_diffs[1] == 0.0f)
        {
          // Up Ramp Only
          const float in_value = (2.0f * ramp_accelerations[0] * distance) + square(start_velocity);
          __assume(in_value >= 0.0f);
          const float expression = sqrtf(in_value);
          const float time = std::max(
            -((expression + start_velocity) / ramp_accelerations[0]),
            ((expression - start_velocity) / ramp_accelerations[0])
          );
          ramp_times[0] = time;
          total_time = time;
        }
        else
        {
          // Both Ramps
          // t = t0 + t1
          // s0 = v0t0 + 1/2a0t0^2
          // v = v0 + a0t0
          // s1 = vt1 + 1/2a1t1^2
          // s = s0 + s1
          // reduce
          // t = t0 + t1
          // s = s0 + s1
          // s0 = v0t0 + 1/2a0t0^2
          // s1 = (v0 + a0t0)t1 + 1/2a1t1^2
          // reduce
          // t = t0 + t1
          // s = (v0t0 + 1/2a0t0^2) + ((v0 + a0t0)t1 + 1/2a1t1^2)
          // make variables single
          // v0 = b
          // t0 = c
          // a0 = d
          // a1 = g
          // t0 = h
          // t1 = j
          //
          // t = h + j
          //s = (b * h + 1/2 * d * h^2) + ((b + d * h) * j + 1/2 * g * j^2)

          static constexpr const uint32 iterations = 8;
          for (uint32 i = 0; i < iterations; ++i)
          {
            const float ratio = ((distance / total_ramp_distance) + 1.0) * 0.5;

            target_velocity *= ratio;


            velocity_diffs[0] = target_velocity - start_velocity;
            velocity_diffs[1] = end_velocity - target_velocity;

            ramp_times[0] = abs(velocity_diffs[0]) / acceleration;
            ramp_times[1] = abs(velocity_diffs[1]) / acceleration;

            ramp_accelerations[0] = (velocity_diffs[0] > 0.0f) ? acceleration : -acceleration;
            ramp_accelerations[1] = (velocity_diffs[1] > 0.0f) ? acceleration : -acceleration;

            const auto square = [](float value)
            {
              return value * value;
            };

            ramp_distances[0] = abs((start_velocity * ramp_times[0]) + (0.5f * ramp_accelerations[0] * square(ramp_times[0])));
            ramp_distances[1] = abs((target_velocity * ramp_times[1]) + (0.5f * ramp_accelerations[1] * square(ramp_times[1])));

            total_ramp_distance = ramp_distances[0] + ramp_distances[1];
          }

          total_time = ramp_times[0] + ramp_times[1];
        }
      }

      dump_var("velocity", _data.velocity);
      dump_var("target_velocity", target_velocity);

      time_ = lround(total_time);
      plateau_velocity_ = target_velocity;
      start_velocity_ = start_velocity;
      end_velocity_ = end_velocity;

      up_time_ = lround(ramp_times[0]);
      up_time_inv_ = ramp_times[0] ? (1.0f / ramp_times[0]) : 0.0f;
      down_time_ = lround(ramp_times[1]);
      down_time_inv_ = ramp_times[1] ? (1.0f / ramp_times[1]) : 0.0f;
      plateau_time_ = lround(plateau_time);
      plateau_time_inv_ = plateau_time ? (1.0f / plateau_time) : 0.0f;
    }

    float cur_velocity = 0.0;

    float get_velocity_dbg() const __restrict
    {
      return cur_velocity;
    }

    uint32 get_current_tick() const __restrict
    {
      return current_tick_ * frequency_scale;
    }

    static constexpr const uint32 tick_time = 8;

    uint32 current_step = 0;

    bool pulse_time() __restrict
    {
      __assume(current_tick_ >= 0.0f);

      if (__likely(current_tick_ < up_time_))
      {
        // Up ramp
        __assume(up_time_inv_ <= 1.0f);
        float time_fract = float(current_tick_) * up_time_inv_;
        __assume(time_fract >= 0.0f && time_fract <= 1.0f);
        float sine = get_sine(time_fract);
        __assume(sine >= 0.0f && sine <= 1.0f);
        __assume(start_velocity_ >= 0.0f);
        __assume(plateau_velocity_ > 0.0f);
        float sine_velocity = lerp(start_velocity_, plateau_velocity_, sine);
        __assume(sine_velocity >= start_velocity_ && sine_velocity <= plateau_velocity_);

        cur_velocity = sine_velocity;

        float steps = prev_steps_ + (sine_velocity * tick_time);
        __assume(steps >= prev_steps_);
        __assume(steps >= 0.0f && steps <= float(distance_));

        prev_steps_ = steps;
        bool result = __unlikely(round<uint32>(steps) > current_step);

        if (__unlikely(result))
        {
          ++current_step;
        }

        current_tick_ += tick_time;
        return result;
      }
      else if (__likely(current_tick_ < (up_time_ + plateau_time_)))
      {
        // Plateau
        __assume(plateau_time_inv_ <= 1.0f);
        __assume(current_tick_ >= (up_time_));
        float time_fract = float(current_tick_ - up_time_) * plateau_time_inv_;
        __assume(time_fract >= 0.0f && time_fract <= 1.0f);

        cur_velocity = plateau_velocity_;

        float steps = prev_steps_ + (plateau_velocity_ * tick_time);
        __assume(steps >= prev_steps_);
        __assume(steps >= 0.0f && steps <= float(distance_));

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
        __assume(down_time_inv_ <= 1.0f);
        __assume(current_tick_ >= (up_time_ + plateau_time_));
        float time_fract = float(current_tick_ - (up_time_ + plateau_time_)) * down_time_inv_;
        __assume(time_fract >= 0.0f && time_fract <= 1.0f);
        float sine = sine_func(time_fract);
        __assume(sine >= 0.0f && sine <= 1.0f);
        __assume(end_velocity_ >= 0.0f);
        __assume(plateau_velocity_ > 0.0f);
        float sine_velocity = lerp(plateau_velocity_, end_velocity_, sine);
        __assume(sine_velocity >= start_velocity_ && sine_velocity <= plateau_velocity_);

        cur_velocity = sine_velocity;

        float steps = prev_steps_ + (sine_velocity * tick_time);
        __assume(steps >= prev_steps_);
        __assume(steps >= 0.0f && steps <= float(distance_));

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
        __assume(steps >= 0.0f && steps <= float(distance_));
        prev_steps_ = steps;

        cur_velocity = end_velocity_;

        bool result = __unlikely(round<uint32>(steps) > current_step);
        if (__unlikely(result))
        {
          ++current_step;
        }

        current_tick_ += tick_time;
        return result;
      }
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
  };
}
