#pragma once
#include <Arduino.h>
#include <math.h>

class SCurvePosVelController {
public:
  struct Limits {
    float v_max = 0.0f;      // [deg/s]
    float a_max = 0.0f;      // [deg/s^2]
    float out_max = -1.0f;   // [deg/s] final clamp (default = v_max)
  };

  struct Gains {
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;         // derivative on measured position
    float ff_vel = 1.0f;
  };

  struct Tolerances {
    float pos = 0.0f;        // [deg]
    float vel = 0.0f;        // [deg/s]
  };

  struct Trajectory {
    bool active = false;
    float elapsed = 0.0f;
    float duration = 0.0f;
    float c0 = 0.0f;
    float c1 = 0.0f;
    float c2 = 0.0f;
    float c3 = 0.0f;
    float c4 = 0.0f;
    float c5 = 0.0f;
  };

  enum class FaultCode : uint8_t {
    None = 0,
    PositionLimitExceeded = 1,
    BadLimits = 2,
    EmergencyStop = 3
  };

  SCurvePosVelController() = default;

  // ---------- Configuration ----------
  void setLimits(float v_max, float a_max) {
    _lim.v_max = fabsf(v_max);
    _lim.a_max = fabsf(a_max);
    if (_out_auto) _lim.out_max = _lim.v_max;
  }

  void setOutputMax(float out_max) {
    if (out_max <= 0.0f) {
      _out_auto = true;
      _lim.out_max = 0.0f;
    } else {
      _out_auto = false;
      _lim.out_max = fabsf(out_max);
    }
  }

  void setGains(float kp, float ki, float kd, float ff_vel = 1.0f) {
    _g.kp = kp;
    _g.ki = ki;
    _g.kd = kd;
    _g.ff_vel = ff_vel;
  }

  void setIntegratorLimit(float i_abs_max) { _i_abs_max = fabsf(i_abs_max); }

  void setTolerances(float pos_tol, float vel_tol) {
    _tol.pos = fabsf(pos_tol);
    _tol.vel = fabsf(vel_tol);
  }

  // Hard mechanical limits: measured position must never exceed this range.
  void setPositionLimits(float pos_min_deg,
                         float pos_max_deg,
                         float stop_margin_deg = 0.5f,
                         float fault_margin_deg = 0.0f) {
    _pos_min = pos_min_deg;
    _pos_max = pos_max_deg;
    _stop_margin = fabsf(stop_margin_deg);
    _fault_margin = fabsf(fault_margin_deg);
    _limits_enabled = true;

    if (_pos_max <= _pos_min) {
      _fault = FaultCode::BadLimits;
      _fault_latched = true;
      _i_term = 0.0f;
      _ref_vel = 0.0f;
      _ref_acc = 0.0f;
      _target = _ref_pos;
      clearTrajectory();
    }
  }

  void disablePositionLimits() { _limits_enabled = false; }

  // ---------- Safety / Fault ----------
  bool fault() const { return _fault_latched; }
  FaultCode faultCode() const { return _fault; }

  void clearFault() {
    _fault_latched = false;
    _fault = FaultCode::None;
    _i_term = 0.0f;
    _ref_vel = 0.0f;
    _ref_acc = 0.0f;
    _target = _ref_pos;
    clearTrajectory();
  }

  void latchPositionLimitFault(float measured_pos) {
    latchStoppedFault(FaultCode::PositionLimitExceeded, measured_pos);
  }

  void latchEmergencyStop(float measured_pos) {
    latchStoppedFault(FaultCode::EmergencyStop, measured_pos);
  }

  // ---------- Lifecycle ----------
  void reset(float measured_pos) {
    if (_limits_enabled) measured_pos = clampf(measured_pos, _pos_min, _pos_max);

    _target = measured_pos;
    _ref_pos = measured_pos;
    _ref_vel = 0.0f;
    _ref_acc = 0.0f;
    clearTrajectory();

    _i_term = 0.0f;
    _prev_meas = measured_pos;
    _has_prev = false;

    _fault = FaultCode::None;
    _fault_latched = false;

    _meas_vel_f = 0.0f;
    _last_meas_vel = 0.0f;
    _last_meas_pos = measured_pos;
    _target_gen = 0;
    _meas_gen = 0;
    _db_active = false;

    _last_us = micros();
  }

  // Plan from the current analytic reference state. The normal planner resets
  // the controller first, so ordinary moves are rest-to-rest trajectories.
  void setTarget(float target_pos) {
    const float target = clampedTarget(target_pos);
    Trajectory candidate;

    if (!makeTrajectory(_ref_pos, _ref_vel, _ref_acc, target, candidate)) {
      // The non-blended API cannot report failure. Fall back to a safe
      // rest-to-rest plan at the current reference position.
      _ref_vel = 0.0f;
      _ref_acc = 0.0f;
      if (!makeTrajectory(_ref_pos, 0.0f, 0.0f, target, candidate)) {
        _fault = FaultCode::BadLimits;
        _fault_latched = true;
        _target = _ref_pos;
        clearTrajectory();
        return;
      }
    }

    acceptTarget(target, candidate);
  }

  // Install a continuous quintic only when it is monotonic and admissible.
  // Failure is transactional: target and current trajectory remain unchanged.
  bool setTargetBlended(float target_pos) {
    const float target = clampedTarget(target_pos);
    Trajectory candidate;
    if (!makeTrajectory(_ref_pos, _ref_vel, _ref_acc, target, candidate)) {
      return false;
    }

    acceptTarget(target, candidate);
    return true;
  }

  // Rebuild the active remainder after vmax/amax changes. The current analytic
  // position, velocity and acceleration are used as exact boundary conditions.
  bool replanActiveTrajectory() {
    if (!_trajectory.active) {
      return true;
    }

    Trajectory candidate;
    if (!makeTrajectory(_ref_pos, _ref_vel, _ref_acc, _target, candidate)) {
      return false;
    }

    _trajectory = candidate;
    return true;
  }

  float target() const { return _target; }
  float refPos() const { return _ref_pos; }
  float refVel() const { return _ref_vel; }
  float refAcc() const { return _ref_acc; }
  bool trajectoryActive() const { return _trajectory.active; }
  float trajectoryDuration() const { return _trajectory.duration; }
  float trajectoryElapsed() const { return _trajectory.elapsed; }

  // ---------- Control Update ----------
  float update(float measured_pos) {
    const uint32_t now = micros();
    float dt = (now - _last_us) * 1e-6f;
    _last_us = now;
    return update(measured_pos, dt);
  }

  float update(float measured_pos, float dt_s) {
    const float dt = clampf(dt_s, MIN_UPDATE_DT_S, MAX_UPDATE_DT_S);

    // Hard measured-position safety check.
    if (_limits_enabled &&
        (measured_pos < (_pos_min - _fault_margin) ||
         measured_pos > (_pos_max + _fault_margin))) {
      latchStoppedFault(FaultCode::PositionLimitExceeded, measured_pos);
      return 0.0f;
    }

    if (_fault_latched) {
      return 0.0f;
    }

    // Measured velocity is used by derivative-on-measurement, deadband and
    // settled detection. It is not used to evolve the analytic trajectory.
    float measured_velocity = 0.0f;
    if (_has_prev) {
      measured_velocity = (measured_pos - _prev_meas) / dt;
    }

    float measured_velocity_used = measured_velocity;
    if (_vel_f_tau > 0.0f) {
      const float alpha = dt / (_vel_f_tau + dt);
      _meas_vel_f += alpha * (measured_velocity - _meas_vel_f);
      measured_velocity_used = _meas_vel_f;
    } else {
      _meas_vel_f = measured_velocity;
    }

    _last_meas_pos = measured_pos;
    _last_meas_vel = measured_velocity_used;
    _meas_gen = _target_gen;

    // Deadband is deliberately disabled while an analytic profile is active.
    if (_db_enabled && !_trajectory.active) {
      const float target = clampedTarget(_target);
      const float position_error = fabsf(target - measured_pos);
      const float velocity_abs = fabsf(measured_velocity_used);

      if (_db_active) {
        if (position_error <= _db_exit && velocity_abs <= _db_vel) {
          freezeInDeadband(target, measured_pos, measured_velocity_used);
          return 0.0f;
        }
        _db_active = false;
      } else if (position_error <= _db_enter && velocity_abs <= _db_vel) {
        _db_active = true;
        freezeInDeadband(target, measured_pos, measured_velocity_used);
        return 0.0f;
      }
    }

    stepTrajectory(dt);

    const float error = _ref_pos - measured_pos;
    const float derivative = -_g.kd * measured_velocity_used;

    _prev_meas = measured_pos;
    _has_prev = true;

    _i_term += _g.ki * error * dt;
    if (_i_abs_max > 0.0f) {
      _i_term = clampf(_i_term, -_i_abs_max, +_i_abs_max);
    }

    const float proportional = _g.kp * error;
    float command = (_g.ff_vel * _ref_vel) +
                    proportional +
                    _i_term +
                    derivative;

    const float output_limit = (_lim.out_max > 0.0f)
      ? _lim.out_max
      : _lim.v_max;
    if (output_limit > 0.0f) {
      const float saturated = clampf(command, -output_limit, +output_limit);
      if (saturated != command && _g.ki != 0.0f) {
        _i_term -= _g.ki * error * dt;
      }
      command = saturated;
    }

    // Prevent an output command from pushing farther through a hard limit.
    if (_limits_enabled && _stop_margin > 0.0f) {
      const float lower_guard = _pos_min + _stop_margin;
      const float upper_guard = _pos_max - _stop_margin;
      if (measured_pos <= lower_guard && command < 0.0f) command = 0.0f;
      if (measured_pos >= upper_guard && command > 0.0f) command = 0.0f;
    }

    return command;
  }

  // ---------- Deadband and settled detection ----------
  void setDeadband(float enter_deg,
                   float exit_deg = 0.0f,
                   float vel_deg_s = 0.5f) {
    _db_enabled = (enter_deg > 0.0f);
    _db_enter = fabsf(enter_deg);
    _db_exit = (exit_deg > 0.0f) ? fabsf(exit_deg) : _db_enter;
    if (_db_exit < _db_enter) _db_exit = _db_enter;
    _db_vel = fabsf(vel_deg_s);
    _db_active = false;
  }

  void setVelocityFilterTau(float tau_s) {
    _vel_f_tau = fmaxf(tau_s, 0.0f);
  }

  void disableDeadband() {
    _db_enabled = false;
    _db_active = false;
  }

  bool inDeadband() const { return _db_active; }
  float getLastMeasuredVel() const { return _last_meas_vel; }

  bool isSettled() const {
    if (_fault_latched || _trajectory.active) return false;
    if (_meas_gen != _target_gen) return false;
    if (_db_enabled && _db_active) return true;
    if (_tol.pos <= 0.0f || _tol.vel <= 0.0f) return false;

    const float position_error = fabsf(clampedTarget(_target) - _last_meas_pos);
    const float velocity_abs = fabsf(_last_meas_vel);
    return position_error <= _tol.pos && velocity_abs <= _tol.vel;
  }

private:
  static constexpr float MIN_UPDATE_DT_S = 1.0e-6f;
  static constexpr float MAX_UPDATE_DT_S = 0.2f;
  static constexpr float MIN_TRAJECTORY_DURATION_S = 1.0e-3f;
  static constexpr float POSITION_EPSILON_DEG = 1.0e-6f;
  static constexpr float VELOCITY_EPSILON_DEG_S = 1.0e-5f;
  static constexpr float ACCELERATION_EPSILON_DEG_S2 = 1.0e-4f;
  static constexpr float QUINTIC_PEAK_VELOCITY = 1.875f;
  static constexpr float QUINTIC_PEAK_ACCELERATION = 5.7735027f;
  static constexpr uint16_t PROFILE_SAMPLES = 160;
  static constexpr uint16_t DURATION_SEARCH_STEPS = 120;
  static constexpr float DURATION_GROWTH = 1.08f;

  Limits _lim{};
  Gains _g{};
  Tolerances _tol{};
  Trajectory _trajectory{};

  bool _out_auto = true;
  float _target = 0.0f;

  float _ref_pos = 0.0f;
  float _ref_vel = 0.0f;
  float _ref_acc = 0.0f;

  float _i_term = 0.0f;
  float _i_abs_max = 0.0f;

  float _prev_meas = 0.0f;
  bool _has_prev = false;

  bool _limits_enabled = false;
  float _pos_min = -INFINITY;
  float _pos_max = +INFINITY;
  float _stop_margin = 0.5f;
  float _fault_margin = 0.0f;

  FaultCode _fault = FaultCode::None;
  bool _fault_latched = false;

  uint32_t _last_us = 0;

  bool _db_enabled = false;
  bool _db_active = false;
  float _db_enter = 0.0f;
  float _db_exit = 0.0f;
  float _db_vel = 0.5f;

  float _last_meas_pos = 0.0f;
  float _last_meas_vel = 0.0f;
  uint32_t _target_gen = 0;
  uint32_t _meas_gen = 0;

  float _meas_vel_f = 0.0f;
  float _vel_f_tau = 0.05f;

  static float clampf(float value, float lower, float upper) {
    if (value < lower) return lower;
    if (value > upper) return upper;
    return value;
  }

  float clampedTarget(float target) const {
    return _limits_enabled ? clampf(target, _pos_min, _pos_max) : target;
  }

  void clearTrajectory() {
    _trajectory = Trajectory{};
  }

  void latchStoppedFault(FaultCode code, float measured_pos) {
    _fault = code;
    _fault_latched = true;
    _i_term = 0.0f;
    _ref_vel = 0.0f;
    _ref_acc = 0.0f;
    _ref_pos = _limits_enabled
      ? clampf(measured_pos, _pos_min, _pos_max)
      : measured_pos;
    _target = _ref_pos;
    clearTrajectory();
    _prev_meas = _ref_pos;
    _has_prev = false;
    _meas_vel_f = 0.0f;
    _last_meas_pos = _ref_pos;
    _last_meas_vel = 0.0f;
    _db_active = false;
  }

  void freezeInDeadband(float target,
                        float measured_pos,
                        float measured_velocity) {
    _ref_pos = target;
    _ref_vel = 0.0f;
    _ref_acc = 0.0f;
    _i_term = 0.0f;
    _prev_meas = measured_pos;
    _has_prev = true;
    _last_meas_pos = measured_pos;
    _last_meas_vel = measured_velocity;
  }

  void acceptTarget(float target, const Trajectory& trajectory) {
    _target = target;
    _trajectory = trajectory;
    _target_gen++;
    _db_active = false;

    if (!_trajectory.active) {
      _ref_pos = target;
      _ref_vel = 0.0f;
      _ref_acc = 0.0f;
    }
  }

  float initialDuration(float distance_abs, float initial_velocity) const {
    const float velocity_duration =
      QUINTIC_PEAK_VELOCITY * distance_abs / _lim.v_max;
    const float acceleration_duration =
      sqrtf(QUINTIC_PEAK_ACCELERATION * distance_abs / _lim.a_max);
    const float boundary_velocity_duration =
      2.0f * fabsf(initial_velocity) / _lim.a_max;
    return fmaxf(MIN_TRAJECTORY_DURATION_S,
                 fmaxf(boundary_velocity_duration,
                       fmaxf(velocity_duration, acceleration_duration)));
  }

  static bool buildCoefficients(float p0,
                                float v0,
                                float a0,
                                float pf,
                                float duration,
                                Trajectory& trajectory) {
    if (!isfinite(p0) || !isfinite(v0) || !isfinite(a0) ||
        !isfinite(pf) || !isfinite(duration) || duration <= 0.0f) {
      return false;
    }

    trajectory = Trajectory{};
    trajectory.active = true;
    trajectory.duration = duration;
    trajectory.c0 = p0;
    trajectory.c1 = v0 * duration;
    trajectory.c2 = 0.5f * a0 * duration * duration;

    const float position_remainder =
      pf - (trajectory.c0 + trajectory.c1 + trajectory.c2);
    const float velocity_remainder =
      -(trajectory.c1 + 2.0f * trajectory.c2);
    const float acceleration_remainder =
      -(2.0f * trajectory.c2);

    trajectory.c3 = 10.0f * position_remainder -
                    4.0f * velocity_remainder +
                    0.5f * acceleration_remainder;
    trajectory.c4 = -15.0f * position_remainder +
                    7.0f * velocity_remainder -
                    acceleration_remainder;
    trajectory.c5 = 6.0f * position_remainder -
                    3.0f * velocity_remainder +
                    0.5f * acceleration_remainder;

    return isfinite(trajectory.c0) && isfinite(trajectory.c1) &&
           isfinite(trajectory.c2) && isfinite(trajectory.c3) &&
           isfinite(trajectory.c4) && isfinite(trajectory.c5);
  }

  static void evaluate(const Trajectory& trajectory,
                       float normalized_time,
                       float& position,
                       float& velocity,
                       float& acceleration) {
    const double u = static_cast<double>(
      clampf(normalized_time, 0.0f, 1.0f));
    // Horner evaluation reduces cancellation near the exact endpoint.
    const double position_value =
      static_cast<double>(trajectory.c0) +
      u * (static_cast<double>(trajectory.c1) +
      u * (static_cast<double>(trajectory.c2) +
      u * (static_cast<double>(trajectory.c3) +
      u * (static_cast<double>(trajectory.c4) +
      u * static_cast<double>(trajectory.c5)))));

    const double position_du =
      static_cast<double>(trajectory.c1) +
      u * (2.0 * static_cast<double>(trajectory.c2) +
      u * (3.0 * static_cast<double>(trajectory.c3) +
      u * (4.0 * static_cast<double>(trajectory.c4) +
      u * 5.0 * static_cast<double>(trajectory.c5))));

    const double position_du2 =
      2.0 * static_cast<double>(trajectory.c2) +
      u * (6.0 * static_cast<double>(trajectory.c3) +
      u * (12.0 * static_cast<double>(trajectory.c4) +
      u * 20.0 * static_cast<double>(trajectory.c5)));

    const double duration = static_cast<double>(trajectory.duration);
    position = static_cast<float>(position_value);
    velocity = static_cast<float>(position_du / duration);
    acceleration = static_cast<float>(
      position_du2 / (duration * duration));
  }

  bool trajectoryAdmissible(const Trajectory& trajectory,
                            float p0,
                            float v0,
                            float a0,
                            float pf) const {
    const float distance = pf - p0;
    const float distance_abs = fabsf(distance);
    const float direction = distance >= 0.0f ? 1.0f : -1.0f;
    const float lower_position = fminf(p0, pf);
    const float upper_position = fmaxf(p0, pf);
    const float position_slack =
      1.0e-4f + 1.0e-5f * fmaxf(1.0f, distance_abs);
    const float velocity_limit = fmaxf(_lim.v_max, fabsf(v0));
    const float acceleration_limit = fmaxf(_lim.a_max, fabsf(a0));
    const float velocity_slack = 1.0e-4f + 1.0e-3f * velocity_limit;
    const float acceleration_slack =
      1.0e-4f + 1.0e-3f * acceleration_limit;

    for (uint16_t sample = 0; sample <= PROFILE_SAMPLES; ++sample) {
      const float u = static_cast<float>(sample) /
                      static_cast<float>(PROFILE_SAMPLES);
      float position = 0.0f;
      float velocity = 0.0f;
      float acceleration = 0.0f;
      evaluate(trajectory, u, position, velocity, acceleration);

      if (!isfinite(position) || !isfinite(velocity) ||
          !isfinite(acceleration)) {
        return false;
      }
      if (position < lower_position - position_slack ||
          position > upper_position + position_slack) {
        return false;
      }
      if (direction * velocity < -velocity_slack) {
        return false;
      }
      if (_limits_enabled &&
          (position < _pos_min - position_slack ||
           position > _pos_max + position_slack)) {
        return false;
      }
      if (fabsf(velocity) > velocity_limit + velocity_slack ||
          fabsf(acceleration) > acceleration_limit + acceleration_slack) {
        return false;
      }
    }

    return true;
  }

  bool makeTrajectory(float p0,
                      float v0,
                      float a0,
                      float pf,
                      Trajectory& trajectory) const {
    if (!isfinite(_lim.v_max) || !isfinite(_lim.a_max) ||
        _lim.v_max <= 0.0f || _lim.a_max <= 0.0f ||
        !isfinite(p0) || !isfinite(v0) || !isfinite(a0) ||
        !isfinite(pf)) {
      return false;
    }

    const float distance = pf - p0;
    if (fabsf(distance) <= POSITION_EPSILON_DEG) {
      if (fabsf(v0) <= VELOCITY_EPSILON_DEG_S &&
          fabsf(a0) <= ACCELERATION_EPSILON_DEG_S2) {
        trajectory = Trajectory{};
        return true;
      }
      return false;
    }

    if (fabsf(v0) > VELOCITY_EPSILON_DEG_S && distance * v0 < 0.0f) {
      return false;
    }

    float duration = initialDuration(fabsf(distance), v0);
    for (uint16_t attempt = 0; attempt < DURATION_SEARCH_STEPS; ++attempt) {
      Trajectory candidate;
      if (buildCoefficients(p0, v0, a0, pf, duration, candidate) &&
          trajectoryAdmissible(candidate, p0, v0, a0, pf)) {
        trajectory = candidate;
        return true;
      }
      duration *= DURATION_GROWTH;
      if (!isfinite(duration)) {
        break;
      }
    }

    return false;
  }

  void stepTrajectory(float dt) {
    if (!_trajectory.active) {
      return;
    }

    _trajectory.elapsed =
      fminf(_trajectory.elapsed + clampf(dt, MIN_UPDATE_DT_S, MAX_UPDATE_DT_S),
            _trajectory.duration);

    if (_trajectory.elapsed >= _trajectory.duration) {
      _ref_pos = _target;
      _ref_vel = 0.0f;
      _ref_acc = 0.0f;
      _trajectory.active = false;
      return;
    }

    const float normalized_time =
      _trajectory.elapsed / _trajectory.duration;
    evaluate(_trajectory,
             normalized_time,
             _ref_pos,
             _ref_vel,
             _ref_acc);

    // Candidate validation guarantees this interval analytically. Clamp only
    // sub-millidegree floating-point residue so endpoint cleanup cannot move
    // the reference backward on the final update.
    _ref_pos = clampf(_ref_pos,
                      fminf(_trajectory.c0, _target),
                      fmaxf(_trajectory.c0, _target));
  }
};
