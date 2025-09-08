
#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace wust_vl_algorithm {

template<typename T = double>
class PID {
public:
    // Constructor:
    // Kp, Ki, Kd: proportional/integral/derivative gains
    // dt_seconds: optional fixed sample time (if <= 0, you must pass dt to update())
    explicit PID(T Kp = T(0), T Ki = T(0), T Kd = T(0), T dt_seconds = T(-1)):
        kp(Kp),
        ki(Ki),
        kd(Kd),
        dt_fixed(dt_seconds),
        out_min(-std::numeric_limits<T>::infinity()),
        out_max(std::numeric_limits<T>::infinity()),
        integrator(T(0)),
        prev_error(T(0)),
        prev_meas(T(0)),
        deriv_filtered(T(0)),
        tau(T(0.01)), // default small filter time constant
        integ_limit(std::numeric_limits<T>::infinity()),
        anti_windup_gain(T(0.0)),
        setpoint_weight_p(T(1.0)),
        setpoint_weight_d(T(1.0)),
        derivative_on_measurement(false),
        first_update(true) {}

    // --- setters ---
    void setGains(T Kp, T Ki, T Kd) {
        kp = Kp;
        ki = Ki;
        kd = Kd;
    }
    void setOutputLimits(T lo, T hi) {
        out_min = lo;
        out_max = hi;
    }
    void setIntegratorLimit(T abs_limit) {
        integ_limit = std::abs(abs_limit);
    }
    void setDerivativeFilterTau(T tau_seconds) {
        if (tau_seconds < T(0))
            throw std::invalid_argument("tau must be >= 0");
        tau = tau_seconds;
    }
    void setAntiWindupGain(T kw) {
        anti_windup_gain = kw;
    } // 0 = clamping-only behavior; >0 enables back-calculation
    void setSetpointWeighting(T wp, T wd = T(1.0)) {
        setpoint_weight_p = wp;
        setpoint_weight_d = wd;
    }
    void setDerivativeOnMeasurement(bool on) {
        derivative_on_measurement = on;
    }
    void setFixedDt(T dt_seconds) {
        dt_fixed = dt_seconds;
    }

    // Reset internal state
    void reset() {
        integrator = T(0);
        prev_error = T(0);
        prev_meas = T(0);
        deriv_filtered = T(0);
        first_update = true;
    }

    // Main update function:
    // If dt_seconds <= 0, uses fixed dt set in constructor or setFixedDt() (must be >0).
    // Returns control output (clamped to output limits).
    T update(T setpoint, T measurement, T dt_seconds = T(-1)) {
        T dt = dt_seconds;
        if (dt <= T(0))
            dt = dt_fixed;
        if (dt <= T(0))
            throw std::invalid_argument("dt must be > 0 (either pass dt or set fixed dt)");

        // compute error
        T error = setpoint - measurement;

        // Proportional (with setpoint weighting)
        T P = kp * (setpoint_weight_p * setpoint - measurement); // note: this form allows weighting
        // Some users prefer P = kp * error; both are supported by setpoint_weight_p

        // Integral
        // Integrator accumulates Ki * error * dt
        integrator += ki * error * dt;
        // integrator clamping (hard limit)
        if (integ_limit < std::numeric_limits<T>::infinity()) {
            if (integrator > integ_limit)
                integrator = integ_limit;
            else if (integrator < -integ_limit)
                integrator = -integ_limit;
        }

        // Derivative (with filtering)
        T raw_deriv = T(0);
        if (first_update) {
            // initialize derivative and prev values on first call to avoid spikes
            prev_error = error;
            prev_meas = measurement;
            deriv_filtered = T(0);
            first_update = false;
        }

        if (derivative_on_measurement) {
            // derivative of measurement -> negative of derivative of measurement contributes to D
            raw_deriv = -(measurement - prev_meas) / dt;
        } else {
            raw_deriv = (error - prev_error) / dt;
        }

        // Low-pass filter for derivative: simple first-order filter
        // alpha = tau / (tau + dt)
        T alpha = (tau > T(0)) ? (tau / (tau + dt)) : T(0);
        deriv_filtered = alpha * deriv_filtered + (T(1) - alpha) * raw_deriv;

        T D = kd
            * (setpoint_weight_d * deriv_filtered
            ); // apply D gain and optional setpoint weighting for derivative

        // Unsaturated output
        T u_unsat = P + integrator + D;

        // Saturate output
        T u_sat = std::clamp(u_unsat, out_min, out_max);

        // Anti-windup (back-calculation)
        // Adjust integrator if u was saturated to reduce windup
        if (anti_windup_gain != T(0)) {
            // Integrator corrects by an amount proportional to (u_sat - u_unsat)
            integrator += anti_windup_gain * (u_sat - u_unsat) * dt;
            // re-clamp integrator after back-calculation
            if (integ_limit < std::numeric_limits<T>::infinity()) {
                if (integrator > integ_limit)
                    integrator = integ_limit;
                else if (integrator < -integ_limit)
                    integrator = -integ_limit;
            }
            // Recompute output (optional)
            u_unsat = P + integrator + D;
            u_sat = std::clamp(u_unsat, out_min, out_max);
        } else {
            // If anti_windup_gain==0, integrator remains as-is (clamped earlier)
        }

        // save state
        prev_error = error;
        prev_meas = measurement;

        return u_sat;
    }

    // Accessors (read-only)
    T getP() const {
        return kp;
    }
    T getI() const {
        return ki;
    }
    T getD() const {
        return kd;
    }
    T getIntegrator() const {
        return integrator;
    }
    T getDerivative() const {
        return deriv_filtered;
    }

private:
    // Gains
    T kp;
    T ki;
    T kd;

    // Fixed dt (if <=0, caller must pass dt on update)
    T dt_fixed;

    // Output limits
    T out_min, out_max;

    // Integral storage and limits
    T integrator;
    T integ_limit;

    // Previous input for derivative calc
    T prev_error;
    T prev_meas;
    T deriv_filtered;
    T tau; // derivative filter time constant

    // Anti windup back-calculation gain (Kw). If 0, only integrator clamp is applied.
    T anti_windup_gain;

    // setpoint weighting: P term uses setpoint_weight_p, D uses setpoint_weight_d
    T setpoint_weight_p;
    T setpoint_weight_d;

    // derivative on measurement vs on error (bool)
    bool derivative_on_measurement;

    // first update flag
    bool first_update;
};

} // namespace wust_vl_algorithm
